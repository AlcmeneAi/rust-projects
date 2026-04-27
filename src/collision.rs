use crate::vehicle::{Vehicle, VelocityLevel, Route};
use crate::intersection::{Intersection, Direction};
use crate::statistics::Statistics;
use std::collections::HashSet;

const SAFETY_DISTANCE_FAST: f32 = 120.0;  // same-lane following: beyond this → Fast
const SAFETY_DISTANCE_STOP: f32 = 50.0;   // same-lane following: below this → Stopped (~vehicle body)
// Planning horizon for the ATW algorithm (pixels beyond intersection boundary).
const ETA_LOOKAHEAD: f32 = 500.0;
// Safety buffer added to both ends of each vehicle's crossing time window (seconds).
const WINDOW_BUFFER_SECS: f32 = 0.3;
// Distance from the intersection entry edge where a yielding vehicle must fully stop.
// Vehicles with a detected conflict decelerate to Normal beyond this line and halt here.
const STOP_LINE_DISTANCE: f32 = 60.0;
// Near zone used for close-call counting in Pass 2.
const INTERSECTION_PADDING: f32 = 50.0;
// Distance below which two crossing vehicles inside the intersection count as a close call.
const CLOSE_CALL_DISTANCE: f32 = 60.0;

pub fn check_collisions_and_apply_strategy(
    vehicles: &mut Vec<Vehicle>,
    intersection: &Intersection,
    statistics: &mut Statistics,
) {
    // Snapshot velocity levels BEFORE any pass modifies them so that Pass-3 transition
    // prints compare against the true start-of-frame state, not Pass-1's overwrite.
    let prev_levels: Vec<VelocityLevel> = vehicles.iter().map(|v| v.get_velocity_level()).collect();

    // Pass 1 — Arrival-Time-Window (ATW) velocity control:
    // Instead of stopping every vehicle that shares a near-zone with a conflicting peer,
    // we only stop a vehicle when its crossing time window OVERLAPS with a conflicting
    // vehicle’s window.  Non-overlapping pairs proceed at full speed with zero delay.
    //
    // For each approaching vehicle:
    //   eta_entry = dist_to_entry_edge / velocity
    //   window    = [eta_entry − BUFFER, eta_entry + crossing_distance/vel + BUFFER]
    //
    // When two conflicting paths have overlapping windows the farther vehicle (larger eta)
    // must stop.  Tiebreak: lower ID has priority.
    for i in 0..vehicles.len() {
        let pos_i    = vehicles[i].get_position();
        let dir_i    = vehicles[i].get_direction();
        let route_i  = vehicles[i].get_route();
        let exited_i = vehicles[i].is_route_applied();
        let in_isect_i   = intersection.contains_point(pos_i);
        let in_lookahead = is_in_or_near_intersection(pos_i, intersection, ETA_LOOKAHEAD);

        // Outside planning horizon or already completed turn → free speed.
        if !in_lookahead || exited_i {
            vehicles[i].set_velocity_level(VelocityLevel::Fast);
            continue;
        }

        // Inside the intersection box and not yet turned → caution speed, absolute priority.
        if in_isect_i {
            vehicles[i].set_velocity_level(VelocityLevel::Normal);
            continue;
        }

        // Approaching (or stopped outside): compute hypothetical window at Fast speed so
        // that stopped vehicles are re-evaluated each frame and un-stop when path clears.
        let hyp_vel   = VelocityLevel::Fast.to_pixels_per_frame();
        let my_window = compute_time_window(pos_i, dir_i, route_i, hyp_vel, false, intersection);
        let my_eta    = dist_to_entry_edge(pos_i, dir_i, intersection) / hyp_vel;
        let my_id     = vehicles[i].get_id();

        let mut should_stop: bool       = false;
        let mut stop_reason_id: Option<u32> = None;

        'conflict_scan: for j in 0..vehicles.len() {
            if i == j { continue; }
            if vehicles[j].is_route_applied() { continue; } // turned → no longer conflicts

            let pos_j      = vehicles[j].get_position();
            let dir_j      = vehicles[j].get_direction();
            let route_j    = vehicles[j].get_route();
            let vel_j      = vehicles[j].get_velocity();
            let in_isect_j = intersection.contains_point(pos_j);

            if !is_in_or_near_intersection(pos_j, intersection, ETA_LOOKAHEAD) { continue; }
            if !will_paths_conflict(dir_i, route_i, dir_j, route_j) { continue; }

            let j_id = vehicles[j].get_id();

            // j is inside the intersection and stopped → occupies it indefinitely.
            if in_isect_j && vel_j <= 0.0 {
                should_stop    = true;
                stop_reason_id = Some(j_id);
                break 'conflict_scan;
            }

            // Compute j’s window (use actual vel if moving, else Fast as upper bound).
            let vel_j_w  = if vel_j > 0.0 { vel_j } else { hyp_vel };
            let j_window = compute_time_window(pos_j, dir_j, route_j, vel_j_w, in_isect_j, intersection);

            if !windows_overlap(my_window, j_window) {
                continue; // no temporal conflict — both can proceed freely
            }

            // Windows overlap → lower eta wins; tiebreak by lower ID.
            let j_eta = if in_isect_j {
                -1.0_f32 // already inside = highest priority
            } else {
                let v = if vel_j > 0.0 { vel_j } else { hyp_vel };
                dist_to_entry_edge(pos_j, dir_j, intersection) / v
            };

            if !(my_eta < j_eta || (my_eta == j_eta && my_id < j_id)) {
                should_stop    = true;
                stop_reason_id = Some(j_id);
                break 'conflict_scan;
            }
        }

        if should_stop {
            let dist = dist_to_entry_edge(pos_i, dir_i, intersection);
            if dist <= STOP_LINE_DISTANCE {
                // At or inside the stop line — must halt here.
                if vehicles[i].get_velocity_level() != VelocityLevel::Stopped {
                    eprintln!(
                        "[ATW P1] id={} STOPPED at stop line (dist={:.1}) for id={:?}",
                        my_id, dist, stop_reason_id
                    );
                }
                vehicles[i].set_velocity_level(VelocityLevel::Stopped);
            } else {
                // Still approaching stop line — decelerate to Normal.
                if vehicles[i].get_velocity_level() == VelocityLevel::Fast {
                    eprintln!(
                        "[ATW P1] id={} DECEL toward stop line (dist={:.1}) for id={:?}",
                        my_id, dist, stop_reason_id
                    );
                }
                vehicles[i].set_velocity_level(VelocityLevel::Normal);
            }
        } else {
            if vehicles[i].get_velocity_level() == VelocityLevel::Stopped
                || vehicles[i].get_velocity_level() == VelocityLevel::Normal
            {
                eprintln!("[ATW P1] id={} CLEARED pos=({:.0},{:.0})", my_id, pos_i.0, pos_i.1);
            }
            vehicles[i].set_velocity_level(VelocityLevel::Fast);
        }
    }

    // Pass 2: reduce velocity for at-risk pairs — set Normal for conflicting vehicles;
    //         damp relative velocity for non-conflicting pairs that are close together.
    let mut collision_risks = Vec::new();
    // Track currently-active close-call pairs so we count once per encounter, not once per frame.
    let mut current_close_call_pairs: HashSet<(u32, u32)> = HashSet::new();
    for i in 0..vehicles.len() {
        for j in (i + 1)..vehicles.len() {
            let v1_pos = vehicles[i].get_position();
            let v2_pos = vehicles[j].get_position();
            let distance = ((v1_pos.0 - v2_pos.0).powi(2) + (v1_pos.1 - v2_pos.1).powi(2)).sqrt();

            let v1_dir = vehicles[i].get_direction();
            let v2_dir = vehicles[j].get_direction();
            let v1_route = vehicles[i].get_route();
            let v2_route = vehicles[j].get_route();

            let v1_near = is_in_or_near_intersection(v1_pos, intersection, INTERSECTION_PADDING);
            let v2_near = is_in_or_near_intersection(v2_pos, intersection, INTERSECTION_PADDING);

            // Vehicles that have already completed their turn are out of the conflict
                // zone — exclude them from both collision_risks and close-call detection.
                let v1_exited = vehicles[i].is_route_applied();
                let v2_exited = vehicles[j].is_route_applied();

                if v1_near && v2_near {
                if !v1_exited && !v2_exited && will_paths_conflict(v1_dir, v1_route, v2_dir, v2_route) {
                    collision_risks.push((i, j, distance, true));
                    // Only count as a close call when vehicles are genuinely dangerously close
                    // and both are inside the intersection (not just approaching)
                    let v1_in = intersection.contains_point(v1_pos);
                    let v2_in = intersection.contains_point(v2_pos);
                    if v1_in && v2_in && distance < CLOSE_CALL_DISTANCE {
                        let id_a = vehicles[i].get_id();
                        let id_b = vehicles[j].get_id();
                        let pair = (id_a.min(id_b), id_a.max(id_b));
                        // Print only on the first frame of each new encounter
                        // (compare against previous frame's active set stored in statistics)
                        if !statistics.is_active_close_call(&pair) {
                            eprintln!(
                                "[CLOSE CALL] id={} & id={} dist={:.1} pos_a=({:.0},{:.0}) pos_b=({:.0},{:.0})",
                                id_a, id_b, distance,
                                v1_pos.0, v1_pos.1, v2_pos.0, v2_pos.1
                            );
                        }
                        current_close_call_pairs.insert(pair);
                    }
                } else if distance < 200.0 {
                    collision_risks.push((i, j, distance, false));
                }
            }
        }
    }

    statistics.update_close_calls(&current_close_call_pairs);

    // Pass 1 (ATW) already handles all intersection priority and stopping.
    // Pass 2 here only damps relative velocity between non-conflicting close pairs
    // as a secondary safety net.
    for (i, j, _distance, is_conflicting) in collision_risks {
        if !is_conflicting {
            let relative_velocity = vehicles[i].get_velocity() - vehicles[j].get_velocity();
            if relative_velocity > 5.0 {
                let current_level = vehicles[i].get_velocity_level();
                let new_level = match current_level {
                    VelocityLevel::Stopped => VelocityLevel::Stopped,
                    VelocityLevel::Fast    => VelocityLevel::Normal,
                    VelocityLevel::Normal  => VelocityLevel::Normal,
                };
                vehicles[i].set_velocity_level(new_level);
            } else if relative_velocity < -5.0 {
                let current_level = vehicles[j].get_velocity_level();
                let new_level = match current_level {
                    VelocityLevel::Stopped => VelocityLevel::Stopped,
                    VelocityLevel::Fast    => VelocityLevel::Normal,
                    VelocityLevel::Normal  => VelocityLevel::Normal,
                };
                vehicles[j].set_velocity_level(new_level);
            }
        }
    }

    // Pass 3: enforce safety distance — reduce follower velocity when too close to leader.
    // Use pure distance buckets so the follower velocity is always proportional to the
    // gap, regardless of what Pass 1 set.  This prevents fast vehicles from drifting into
    // stopped leaders because Pass 1 kept resetting them to Fast.
    for i in 0..vehicles.len() {
        let v_i_pos = vehicles[i].get_position();
        let v_i_dir = vehicles[i].get_direction();
        let v_i_lane = vehicles[i].get_assigned_lane();

        for j in 0..vehicles.len() {
            if i == j {
                continue;
            }

            let v_j_pos = vehicles[j].get_position();
            let v_j_dir = vehicles[j].get_direction();
            let v_j_lane = vehicles[j].get_assigned_lane();

            if v_i_dir == v_j_dir && v_i_lane == v_j_lane {
                let distance = ((v_i_pos.0 - v_j_pos.0).powi(2) + (v_i_pos.1 - v_j_pos.1).powi(2)).sqrt();

                let is_ahead = match v_i_dir {
                    Direction::North => v_j_pos.1 < v_i_pos.1,
                    Direction::South => v_j_pos.1 > v_i_pos.1,
                    Direction::East => v_j_pos.0 > v_i_pos.0,
                    Direction::West => v_j_pos.0 < v_i_pos.0,
                };

                // Use <= so vehicles exactly at the safety threshold are also caught
                if is_ahead && distance <= SAFETY_DISTANCE_FAST {
                    // Pick level purely from distance using hysteresis to avoid oscillation
                    // at bucket boundaries.  The current level determines which direction
                    // the threshold applies: we only step DOWN when distance drops 5px
                    // below the bucket edge, and only step UP when it rises 5px above.
                    let cur = prev_levels[i];
                    let new_level = if distance < SAFETY_DISTANCE_STOP {
                        VelocityLevel::Stopped
                    } else if distance < SAFETY_DISTANCE_FAST - if cur == VelocityLevel::Fast { 5.0 } else { 0.0 } {
                        VelocityLevel::Normal
                    } else {
                        // Far enough — let Pass 1 decide (don't override upward)
                        cur
                    };
                    // Only log on transition — compare against the pre-pass snapshot so that
                    // Pass-1's per-frame Fast reset doesn't register as a new transition.
                    if prev_levels[i] != new_level {
                        eprintln!(
                            "[COL P3] id={} dir={:?} lane={} too close to id={} dist={:.1} {:?}->{:?}",
                            vehicles[i].get_id(), v_i_dir, v_i_lane,
                            vehicles[j].get_id(), distance,
                            prev_levels[i], new_level
                        );
                    }
                    vehicles[i].set_velocity_level(new_level);
                }
            }
        }
    }
}

pub fn will_paths_conflict(dir1: Direction, route1: Route, dir2: Direction, route2: Route) -> bool {
    // Vehicles from the same entry direction never block each other at the intersection
    if dir1 == dir2 {
        return false;
    }
    paths_physically_cross(dir1, route1, dir2, route2)
}

/// Returns true if the two vehicles' paths physically cross inside the intersection.
fn paths_physically_cross(dir1: Direction, route1: Route, dir2: Direction, route2: Route) -> bool {
    // Right turns stay on the outer edge and never cross other paths
    if route1 == Route::Right || route2 == Route::Right {
        return false;
    }
    // Opposite directions going straight use separate parallel lanes — they don't cross
    match (dir1, dir2) {
        (Direction::North, Direction::South) | (Direction::South, Direction::North) => {
            // Only conflict when at least one turns Left (into oncoming lane)
            route1 == Route::Left || route2 == Route::Left
        }
        (Direction::East, Direction::West) | (Direction::West, Direction::East) => {
            // Only conflict when at least one turns Left (into oncoming lane)
            route1 == Route::Left || route2 == Route::Left
        }
        // Perpendicular directions always cross
        _ => true,
    }
}

pub fn is_in_or_near_intersection(pos: (f32, f32), intersection: &Intersection, padding: f32) -> bool {
    let dx = (pos.0 - intersection.center.0).abs();
    let dy = (pos.1 - intersection.center.1).abs();
    dx < intersection.size + padding && dy < intersection.size + padding
}

// === Arrival-Time-Window helper functions ===

/// Distance (px) from vehicle's position to the intersection entry edge along its direction.
/// Returns 0.0 if already at or past the entry edge.
fn dist_to_entry_edge(pos: (f32, f32), dir: Direction, intersection: &Intersection) -> f32 {
    match dir {
        Direction::North => (pos.1 - (intersection.center.1 + intersection.size)).max(0.0),
        Direction::South => ((intersection.center.1 - intersection.size) - pos.1).max(0.0),
        Direction::East  => ((intersection.center.0 - intersection.size) - pos.0).max(0.0),
        Direction::West  => (pos.0 - (intersection.center.0 + intersection.size)).max(0.0),
    }
}

/// Distance (px) from vehicle's current position to the exit edge along its direction.
/// Used for vehicles already inside the intersection to estimate remaining crossing time.
fn dist_to_exit_edge(pos: (f32, f32), dir: Direction, intersection: &Intersection) -> f32 {
    match dir {
        Direction::North => (pos.1 - (intersection.center.1 - intersection.size)).max(0.0),
        Direction::South => ((intersection.center.1 + intersection.size) - pos.1).max(0.0),
        Direction::East  => ((intersection.center.0 + intersection.size) - pos.0).max(0.0),
        Direction::West  => (pos.0 - (intersection.center.0 - intersection.size)).max(0.0),
    }
}

/// Approximate path length through the intersection for each route (pixels).
fn crossing_distance(route: Route) -> f32 {
    match route {
        Route::Right    => 100.0, // short quarter-arc on outer edge
        Route::Straight => 400.0, // full diameter
        Route::Left     => 500.0, // wider arc across the intersection
    }
}

/// Compute a vehicle’s time window [entry_t, exit_t] in seconds relative to now,
/// with WINDOW_BUFFER_SECS applied to both ends.
///
/// * `in_intersection` – true when the vehicle is already inside the intersection box.
/// * `vel`            – velocity in px/s.
fn compute_time_window(
    pos: (f32, f32),
    dir: Direction,
    route: Route,
    vel: f32,
    in_intersection: bool,
    intersection: &Intersection,
) -> (f32, f32) {
    if in_intersection {
        let remaining = dist_to_exit_edge(pos, dir, intersection);
        let exit_t = if vel > 0.0 { remaining / vel } else { f32::MAX };
        (-WINDOW_BUFFER_SECS, exit_t + WINDOW_BUFFER_SECS)
    } else {
        if vel <= 0.0 {
            return (f32::MAX, f32::MAX); // stopped outside → no future window
        }
        let d_entry = dist_to_entry_edge(pos, dir, intersection);
        let cross   = crossing_distance(route);
        (
            d_entry / vel - WINDOW_BUFFER_SECS,
            (d_entry + cross) / vel + WINDOW_BUFFER_SECS,
        )
    }
}

/// True when two time windows [a.0, a.1] and [b.0, b.1] overlap.
fn windows_overlap(a: (f32, f32), b: (f32, f32)) -> bool {
    a.0 < b.1 && b.0 < a.1
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_intersection() -> Intersection {
        Intersection::new(700.0, 450.0, 200.0)
    }

    #[test]
    fn extracted_module_compiles() {
        let intersection = make_intersection();
        let mut vehicles: Vec<Vehicle> = Vec::new();
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &intersection, &mut stats);
    }

    // Geometry reference:
    // Intersection center: (700.0, 450.0), size: 200.0, padding: INTERSECTION_PADDING=200.0
    // Near zone: |x - 700| < 400 AND |y - 450| < 400  → for x=700: y in (50, 850) exclusive
    // In-intersection: |x - 700| < 200 AND |y - 450| < 200
    // Far zone: |y - 450| >= 400 → for x=700: y <= 50 or y >= 850 (boundary is FAR)

    #[test]
    fn vehicle_far_from_intersection_gets_fast_velocity() {
        // y=1000 → |1000 - 450| = 550 > 400 → far zone
        let mut vehicle = Vehicle::new(0, Direction::North, Route::Straight);
        vehicle.set_position(700.0, 1000.0);
        let mut vehicles = vec![vehicle];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(vehicles[0].get_velocity_level(), VelocityLevel::Fast);
    }

    #[test]
    fn vehicle_in_intersection_gets_normal_velocity() {
        // (700, 450) is the center — inside on both axes (|0| < 200)
        let mut vehicle = Vehicle::new(0, Direction::North, Route::Straight);
        vehicle.set_position(700.0, 450.0);
        let mut vehicles = vec![vehicle];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(vehicles[0].get_velocity_level(), VelocityLevel::Normal);
    }

    #[test]
    fn single_vehicle_approaching_with_no_conflicts_gets_fast() {
        // With ATW algorithm a single approaching vehicle (no conflicts) gets Fast,
        // not Normal — no reason to slow down when the intersection is clear.
        // y=700: dist_to_entry = 700-(450+200)=50px, inside ETA_LOOKAHEAD(500), no conflicts.
        let mut vehicle = Vehicle::new(0, Direction::North, Route::Straight);
        vehicle.set_position(700.0, 700.0);
        let mut vehicles = vec![vehicle];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(vehicles[0].get_velocity_level(), VelocityLevel::Fast);
    }

    #[test]
    fn safety_distance_reduction_not_overridden_by_baseline_pass() {
        // Two North/Straight vehicles (same lane=1, since Straight → assigned_lane=1)
        // Leader at y=870 (far: |870-450|=420 > 400 → outside near zone)
        // Follower at y=920 (far: |920-450|=470 > 400 → outside near zone)
        // Gap = 50px < SAFETY_DISTANCE_FAST (100.0) → safety distance enforcement must reduce follower
        //
        // BUG (current code): baseline runs LAST → resets follower to Fast after safety pass ran
        // FIX (Task 4):        baseline runs FIRST → safety pass reduces follower from Fast to Normal
        // Vehicle::new initializes velocity_level to Normal, so required_safety_distance=70.0 during
        // the current (buggy) pass 3 — gap 50px < 70px triggers reduction, then baseline overrides to Fast.
        let mut leader = Vehicle::new(0, Direction::North, Route::Straight);
        let mut follower = Vehicle::new(1, Direction::North, Route::Straight);
        leader.set_position(700.0, 870.0);   // ahead (North moves toward smaller y, so leader has smaller y)
        follower.set_position(700.0, 920.0); // 50px behind leader
        let mut vehicles = vec![leader, follower];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(
            vehicles[1].get_velocity_level(),
            VelocityLevel::Normal,
            "follower within safety distance of leader must be reduced from Fast to Normal (no Slow level exists)"
        );
    }

    #[test]
    fn conflicting_vehicles_atw_priority_closer_proceeds_farther_stops() {
        // North/Straight at (700,700): dist_to_entry=50px, eta=50/160=0.3125s
        // East/Straight  at (400,450): dist_to_entry=100px, eta=100/160=0.625s
        // Windows overlap (both short eta, crossing=400px) → North has lower eta → North proceeds,
        // East stops.
        let mut north = Vehicle::new(0, Direction::North, Route::Straight);
        let mut east  = Vehicle::new(1, Direction::East,  Route::Straight);
        north.set_position(700.0, 700.0);
        east.set_position(400.0, 450.0);
        let mut vehicles = vec![north, east];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(vehicles[0].get_velocity_level(), VelocityLevel::Fast,    "North (closer) should proceed at Fast");
        assert_eq!(vehicles[1].get_velocity_level(), VelocityLevel::Stopped, "East (farther) should stop and yield");
    }
}
