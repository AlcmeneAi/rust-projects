use crate::vehicle::{Vehicle, VelocityLevel, Route};
use crate::intersection::{Intersection, Direction};
use crate::statistics::Statistics;
use std::collections::HashSet;

const SAFETY_DISTANCE_FAST: f32 = 120.0;  // same-lane following: beyond this → Fast
const SAFETY_DISTANCE_STOP: f32 = 50.0;   // same-lane following: below this → Stopped (~vehicle body)
// Planning horizon for the ATW algorithm (pixels beyond intersection boundary).
const ETA_LOOKAHEAD: f32 = 1000.0;
// Safety buffer added to both ends of each vehicle's crossing time window (seconds).
const WINDOW_BUFFER_SECS: f32 = 0.3;
// Stop-line distance: set to 0.0 to disable the physical stop line — ATW target-velocity
// modulation is the sole speed-control mechanism; vehicles never hard-stop at a line.
#[allow(dead_code)]
const STOP_LINE_DISTANCE: f32 = 0.0;
// Near zone used for close-call counting in Pass 2.
const INTERSECTION_PADDING: f32 = 50.0;
// Distance below which two crossing vehicles inside the intersection count as a close call.
const CLOSE_CALL_DISTANCE: f32 = 60.0;
// Minimum speed enforced by ATW: vehicles always keep moving at at least this rate
// so they never fully halt due to a computed target velocity (Pass 3 still stops them
// on hard physical contact via SAFETY_DISTANCE_STOP).
const CRAWL_SPEED: f32 = 20.0;
// Approach zone thresholds: distance from intersection entry edge (pixels).
// Yielding vehicles drop to Slow below CRAWL_ZONE; priority vehicles cap at Normal below SLOW_ZONE.
const APPROACH_CRAWL_ZONE: f32 = 80.0;
const APPROACH_SLOW_ZONE: f32 = 200.0;

pub fn check_collisions_and_apply_strategy(
    vehicles: &mut Vec<Vehicle>,
    intersection: &Intersection,
    statistics: &mut Statistics,
) {
    // Snapshot velocity levels BEFORE any pass modifies them so that Pass-3 transition
    // prints compare against the true start-of-frame state, not Pass-1's overwrite.
    let prev_levels: Vec<VelocityLevel> = vehicles.iter().map(|v| v.get_velocity_level()).collect();

    // Pass 1 — Closest-to-intersection wins:
    // Each approaching vehicle checks all conflicting peers. Whoever is physically
    // closer to the intersection entry edge has priority and proceeds at full speed.
    // The farther vehicle yields by slowing down. Vehicles already inside always
    // have top priority. Approach speed is graded: Fast → Normal as
    // the vehicle enters the slow-down zone near the intersection.
    for i in 0..vehicles.len() {
        let pos_i    = vehicles[i].get_position();
        let dir_i    = vehicles[i].get_direction();
        let route_i  = vehicles[i].get_route();
        let exited_i = vehicles[i].is_route_applied();
        let in_isect_i = intersection.contains_point(pos_i);

        // Already turned / outside planning horizon → free speed.
        if exited_i || !is_in_or_near_intersection(pos_i, intersection, ETA_LOOKAHEAD) {
            vehicles[i].set_velocity_level(VelocityLevel::Fast);
            continue;
        }

        // Inside the intersection and not yet turned → Normal speed, highest priority.
        if in_isect_i {
            vehicles[i].set_velocity_level(VelocityLevel::Normal);
            continue;
        }

        let dist_i = dist_to_entry_edge(pos_i, dir_i, intersection);
        let my_id  = vehicles[i].get_id();

        // Check all conflicting peers — yield if any peer is closer (or inside).
        let mut must_yield = false;
        for j in 0..vehicles.len() {
            if i == j { continue; }
            if vehicles[j].is_route_applied() { continue; }

            let pos_j      = vehicles[j].get_position();
            let dir_j      = vehicles[j].get_direction();
            let route_j    = vehicles[j].get_route();
            let in_isect_j = intersection.contains_point(pos_j);

            if !is_in_or_near_intersection(pos_j, intersection, ETA_LOOKAHEAD) { continue; }
            if !will_paths_conflict(dir_i, route_i, dir_j, route_j) { continue; }

            // Any vehicle already inside has absolute priority.
            if in_isect_j {
                must_yield = true;
                break;
            }

            let dist_j = dist_to_entry_edge(pos_j, dir_j, intersection);
            let j_id   = vehicles[j].get_id();

            // j is closer (or equal distance with lower ID) → i must yield.
            if dist_j < dist_i || (dist_j == dist_i && j_id < my_id) {
                must_yield = true;
                break;
            }
        }

        let new_level = if must_yield {
            // Yield: slow more the closer we are to the entry.
            if dist_i < APPROACH_CRAWL_ZONE {
                VelocityLevel::Slow
            } else {
                VelocityLevel::Normal
            }
        } else {
            // Has priority: grade approach speed as we near the intersection.
            if dist_i < APPROACH_SLOW_ZONE {
                VelocityLevel::Normal
            } else {
                VelocityLevel::Fast
            }
        };
        vehicles[i].set_velocity_level(new_level);
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
                    VelocityLevel::Stopped   => VelocityLevel::Stopped,
                    VelocityLevel::Slow      => VelocityLevel::Slow,
                    VelocityLevel::Custom(_) => current_level,  // already ATW-modulated
                    VelocityLevel::Fast      => VelocityLevel::Normal,
                    VelocityLevel::Normal    => VelocityLevel::Normal,
                };
                vehicles[i].set_velocity_level(new_level);
            } else if relative_velocity < -5.0 {
                let current_level = vehicles[j].get_velocity_level();
                let new_level = match current_level {
                    VelocityLevel::Stopped   => VelocityLevel::Stopped,
                    VelocityLevel::Slow      => VelocityLevel::Slow,
                    VelocityLevel::Custom(_) => current_level,  // already ATW-modulated
                    VelocityLevel::Fast      => VelocityLevel::Normal,
                    VelocityLevel::Normal    => VelocityLevel::Normal,
                };
                vehicles[j].set_velocity_level(new_level);
            }
        }
    }

    // --- Virtual Buffering: pre-compute effective ATW windows at post-Pass-1/2 velocities ---
    // When Pass 1 slows a vehicle to resolve an intersection conflict its crossing window
    // shifts later (larger eta_exit) at the reduced speed.  Storing these updated windows
    // lets Pass 3 pre-emptively slow same-lane followers proportionally instead of waiting
    // until the physical SAFETY_DISTANCE_FAST / SAFETY_DISTANCE_STOP thresholds are hit.
    let effective_windows: Vec<Option<(f32, f32)>> = vehicles.iter().map(|v| {
        if v.is_route_applied() { return None; }
        let pos = v.get_position();
        if !is_in_or_near_intersection(pos, intersection, ETA_LOOKAHEAD) { return None; }
        let vel = v.get_velocity();
        if vel <= 0.0 { return None; }
        let in_isect = intersection.contains_point(pos);
        Some(compute_time_window(pos, v.get_direction(), v.get_route(), vel, in_isect, intersection))
    }).collect();

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

                // Virtual Buffering: propagate a slowed leader's updated crossing window to
                // its follower.  Rather than waiting for a physical gap closure, i slows down
                // proportionally so it arrives at the intersection only after j has cleared.
                // This creates smooth cascading deceleration down the queue.
                if is_ahead && !vehicles[i].is_route_applied() {
                    if let Some(j_win) = effective_windows[j] {
                        if is_in_or_near_intersection(v_i_pos, intersection, ETA_LOOKAHEAD) {
                            // Re-read velocity each iteration: a previous j may have already
                            // reduced it, so we always apply the tightest constraint.
                            let i_vel  = vehicles[i].get_velocity();
                            let i_dist = dist_to_entry_edge(v_i_pos, v_i_dir, intersection);
                            let i_eta  = if i_vel > 0.0 { i_dist / i_vel } else { f32::MAX };
                            // i would arrive before j clears the intersection — modulate.
                            if i_eta < j_win.1 {
                                let slow_vel   = VelocityLevel::Slow.to_pixels_per_frame();
                                let target_vel = if i_dist > 0.0 {
                                    i_dist / (j_win.1 + WINDOW_BUFFER_SECS)
                                } else {
                                    slow_vel
                                };
                                let clamped = target_vel.max(slow_vel);
                                // Only slow down — never override a Stopped decision.
                                if clamped < i_vel
                                    && !matches!(vehicles[i].get_velocity_level(),
                                        VelocityLevel::Stopped | VelocityLevel::Custom(_) if vehicles[i].get_velocity() <= CRAWL_SPEED)
                                {
                                    eprintln!(
                                        "[VB P3] id={} vbuf vel {:.1}->{:.1} (leader={} j_exit={:.2})",
                                        vehicles[i].get_id(), i_vel, clamped,
                                        vehicles[j].get_id(), j_win.1
                                    );
                                    vehicles[i].set_modulated_velocity(clamped);
                                }
                            }
                        }
                    }
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

// === Public ATW prediction API ===

/// Return a vehicle's projected intersection-occupancy time window `(entry_t, exit_t)` in
/// seconds relative to now, padded by `WINDOW_BUFFER_SECS` on each side.
///
/// The calculation uses the vehicle's **actual current velocity** so that the window
/// automatically shifts later as the vehicle decelerates.
///
/// * Vehicles already inside the intersection receive `entry_t = -WINDOW_BUFFER_SECS`
///   (they have priority); `exit_t` is derived from distance remaining to exit edge.
/// * Vehicles outside but with `velocity == 0` receive `(f32::MAX, f32::MAX)` — no
///   meaningful future window can be computed.
///
/// Formula:
/// $$t_{entry} = \frac{d_{entry}}{v} - B,\quad t_{exit} = \frac{d_{entry} + d_{cross}}{v} + B$$
/// where $B$ = `WINDOW_BUFFER_SECS`.
pub fn vehicle_time_window(v: &Vehicle, intersection: &Intersection) -> (f32, f32) {
    let pos      = v.get_position();
    let vel      = v.get_velocity();
    let in_isect = intersection.contains_point(pos);
    compute_time_window(pos, v.get_direction(), v.get_route(), vel, in_isect, intersection)
}

/// Return `true` when two half-open time windows `[a.0, a.1)` and `[b.0, b.1)` overlap.
///
/// $$\text{overlap} \iff a_0 < b_1 \land b_0 < a_1$$
pub fn check_overlap(window_a: (f32, f32), window_b: (f32, f32)) -> bool {
    windows_overlap(window_a, window_b)
}

/// Predict whether vehicles `a` and `b` will conflict at the intersection.
///
/// Returns `true` when **both** conditions hold:
/// 1. `will_paths_conflict` — the vehicles' physical paths cross inside the intersection.
/// 2. `check_overlap`       — their projected time windows overlap, meaning they are
///    predicted to occupy the crossing region at the same time.
///
/// This is the primary entry point for ATW conflict detection from outside this module.
pub fn predict_atw_conflict(a: &Vehicle, b: &Vehicle, intersection: &Intersection) -> bool {
    if !will_paths_conflict(a.get_direction(), a.get_route(), b.get_direction(), b.get_route()) {
        return false;
    }
    let win_a = vehicle_time_window(a, intersection);
    let win_b = vehicle_time_window(b, intersection);
    check_overlap(win_a, win_b)
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
        let mut leader   = Vehicle::new(0, Direction::North, Route::Straight);
        let mut follower  = Vehicle::new(1, Direction::North, Route::Straight);
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
        let mut east  = Vehicle::new(1, Direction::East, Route::Straight);
        north.set_position(700.0, 700.0);
        east.set_position(400.0, 450.0);
        let mut vehicles = vec![north, east];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(vehicles[0].get_velocity_level(), VelocityLevel::Fast,    "North (closer) should proceed at Fast");
        assert_eq!(vehicles[1].get_velocity_level(), VelocityLevel::Stopped, "East (farther) should stop and yield");
    }

    // --- Public ATW API unit tests ---

    #[test]
    fn check_overlap_non_overlapping_windows() {
        assert!(!check_overlap((0.0, 1.0), (2.0, 3.0)));
        assert!(!check_overlap((2.0, 3.0), (0.0, 1.0)));
    }

    #[test]
    fn check_overlap_touching_windows_are_not_overlapping() {
        // [0,1) and [1,2) share only the boundary — not considered overlapping.
        assert!(!check_overlap((0.0, 1.0), (1.0, 2.0)));
    }

    #[test]
    fn check_overlap_overlapping_windows() {
        assert!(check_overlap((0.0, 2.0), (1.0, 3.0)));
        assert!(check_overlap((1.0, 3.0), (0.0, 2.0)));
        // One window fully inside the other
        assert!(check_overlap((0.0, 10.0), (2.0, 5.0)));
    }

    #[test]
    fn vehicle_time_window_approaching_vehicle() {
        // North/Straight at y=850: dist_to_entry = 850-(450+200) = 200px
        // vel = Fast = 160 px/frame; crossing = 400px
        // entry_t = 200/160 - 0.3 = 1.25 - 0.3 = 0.95
        // exit_t  = (200+400)/160 + 0.3 = 3.75 + 0.3 = 4.05
        let mut v = Vehicle::new(0, Direction::North, Route::Straight);
        v.set_position(700.0, 850.0);
        v.set_velocity_level(VelocityLevel::Fast);
        let (entry_t, exit_t) = vehicle_time_window(&v, &make_intersection());
        let eps = 0.01;
        assert!((entry_t - 0.95).abs() < eps, "entry_t={entry_t}");
        assert!((exit_t  - 4.05).abs() < eps, "exit_t={exit_t}");
    }

    #[test]
    fn vehicle_time_window_stopped_outside_returns_max() {
        let mut v = Vehicle::new(0, Direction::North, Route::Straight);
        v.set_position(700.0, 850.0);
        v.set_velocity_level(VelocityLevel::Stopped);
        let (entry_t, exit_t) = vehicle_time_window(&v, &make_intersection());
        assert_eq!(entry_t, f32::MAX);
        assert_eq!(exit_t,  f32::MAX);
    }

    #[test]
    fn vehicle_time_window_inside_intersection() {
        // Vehicle already inside: entry_t = -WINDOW_BUFFER_SECS = -0.3
        let mut v = Vehicle::new(0, Direction::North, Route::Straight);
        v.set_position(700.0, 450.0);  // intersection centre
        v.set_velocity_level(VelocityLevel::Normal);
        let (entry_t, _exit_t) = vehicle_time_window(&v, &make_intersection());
        let eps = 0.001;
        assert!((entry_t - (-WINDOW_BUFFER_SECS)).abs() < eps, "entry_t={entry_t}");
    }

    #[test]
    fn predict_atw_conflict_non_conflicting_paths() {
        // Two vehicles from the same direction: paths never cross.
        let mut a = Vehicle::new(0, Direction::North, Route::Straight);
        let mut b = Vehicle::new(1, Direction::North, Route::Right);
        a.set_position(700.0, 850.0);
        b.set_position(750.0, 900.0);
        assert!(!predict_atw_conflict(&a, &b, &make_intersection()));
    }

    #[test]
    fn predict_atw_conflict_conflicting_paths_overlapping_windows() {
        // North/Straight and East/Straight — perpendicular paths, both close.
        let mut north = Vehicle::new(0, Direction::North, Route::Straight);
        let mut east  = Vehicle::new(1, Direction::East, Route::Straight);
        north.set_position(700.0, 700.0);  // 50px from entry
        east.set_position(400.0, 475.0);   // 100px from entry
        north.set_velocity_level(VelocityLevel::Fast);
        east.set_velocity_level(VelocityLevel::Fast);
        assert!(predict_atw_conflict(&north, &east, &make_intersection()));
    }

    #[test]
    fn predict_atw_conflict_conflicting_paths_non_overlapping_windows() {
        // North is very close (nearly through); East is far away → windows don't overlap.
        let mut north = Vehicle::new(0, Direction::North, Route::Straight);
        let mut east  = Vehicle::new(1, Direction::East, Route::Straight);
        // North at entry edge — exits quickly.
        north.set_position(700.0, 652.0);   // ~2px from entry (450+200=650)
        // East very far away — arrives much later.
        east.set_position(-500.0, 475.0);
        north.set_velocity_level(VelocityLevel::Fast);
        east.set_velocity_level(VelocityLevel::Fast);
        assert!(!predict_atw_conflict(&north, &east, &make_intersection()));
    }
}
