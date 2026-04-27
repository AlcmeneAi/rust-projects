use crate::vehicle::{Vehicle, VelocityLevel, Route};
use crate::intersection::{Intersection, Direction};
use crate::statistics::Statistics;
use std::collections::HashSet;

const SAFETY_DISTANCE_FAST: f32 = 120.0;  // beyond this → Fast
const SAFETY_DISTANCE_NORMAL: f32 = 80.0;  // within this → Normal; below STOP_DISTANCE → Stopped
const SAFETY_DISTANCE_STOP: f32 = 50.0;   // must exceed vehicle body length (~30px)
// Near zone = intersection boundary + this padding.  Keep small so vehicles only
// enter the wait zone when genuinely close to the intersection.
const INTERSECTION_PADDING: f32 = 50.0;
// Distance below which two crossing vehicles inside the intersection count as a close call
const CLOSE_CALL_DISTANCE: f32 = 60.0;

pub fn check_collisions_and_apply_strategy(
    vehicles: &mut Vec<Vehicle>,
    intersection: &Intersection,
    statistics: &mut Statistics,
) {
    // Snapshot velocity levels BEFORE any pass modifies them so that Pass-3 transition
    // prints compare against the true start-of-frame state, not Pass-1's overwrite.
    let prev_levels: Vec<VelocityLevel> = vehicles.iter().map(|v| v.get_velocity_level()).collect();

    // Pass 1: set baseline velocity by intersection zone.
    // - In intersection: Normal (caution speed)
    // - Near intersection with a conflicting vehicle ALREADY IN the intersection: Stopped
    // - Near intersection with conflict but no vehicle inside: Normal
    // - Near intersection, no conflict: Normal
    // - Far from intersection: Fast
    for i in 0..vehicles.len() {
        let pos = vehicles[i].get_position();
        let dir = vehicles[i].get_direction();
        let route = vehicles[i].get_route();
        let exited = vehicles[i].is_route_applied();

        let in_intersection = intersection.contains_point(pos);
        let near_intersection = is_in_or_near_intersection(pos, intersection, INTERSECTION_PADDING);

        if in_intersection && !exited {
            // Vehicle is physically inside and hasn't turned yet → caution speed
            vehicles[i].set_velocity_level(VelocityLevel::Normal);
        } else if near_intersection && !exited {
            let my_dist = intersection.get_distance_to_center(pos);
            let my_id = vehicles[i].get_id();
            let mut should_stop = false;
            let mut has_conflicts = false;
            let mut stop_reason_id: Option<u32> = None;
            for j in 0..vehicles.len() {
                if i != j {
                    let other_pos = vehicles[j].get_position();
                    let other_dir = vehicles[j].get_direction();
                    let other_route = vehicles[j].get_route();

                    if is_in_or_near_intersection(other_pos, intersection, INTERSECTION_PADDING) {
                        // Only treat vehicles that haven't turned yet as blockers.
                        // A vehicle with route_applied=true has already exited the conflict zone.
                        if !vehicles[j].is_route_applied()
                            && will_paths_conflict(dir, route, other_dir, other_route)
                        {
                            has_conflicts = true;
                            let other_dist = intersection.get_distance_to_center(other_pos);
                            let other_id = vehicles[j].get_id();
                            // Stop if the conflicting vehicle is inside, closer to center,
                            // or same distance with a lower ID (deterministic priority)
                            if intersection.contains_point(other_pos)
                                || other_dist < my_dist
                                || (other_dist == my_dist && other_id < my_id)
                            {
                                should_stop = true;
                                stop_reason_id = Some(other_id);
                                break;
                            }
                        }
                    }
                }
            }

            if should_stop {
                // Only log the first frame a vehicle becomes Stopped (not every frame)
                if vehicles[i].get_velocity_level() != VelocityLevel::Stopped {
                    eprintln!(
                        "[COL P1] id={} STOPPED waiting for id={:?} pos=({:.0},{:.0}) dist={:.0}",
                        my_id, stop_reason_id, pos.0, pos.1, my_dist
                    );
                }
                vehicles[i].set_velocity_level(VelocityLevel::Stopped);
            } else if has_conflicts {
                if vehicles[i].get_velocity_level() == VelocityLevel::Stopped {
                    eprintln!("[COL P1] id={} CLEARED (conflict remains but yielded) pos=({:.0},{:.0})", my_id, pos.0, pos.1);
                }
                vehicles[i].set_velocity_level(VelocityLevel::Normal);
            } else {
                if vehicles[i].get_velocity_level() == VelocityLevel::Stopped {
                    eprintln!("[COL P1] id={} CLEARED (no conflict) pos=({:.0},{:.0})", my_id, pos.0, pos.1);
                }
                vehicles[i].set_velocity_level(VelocityLevel::Normal);
            }
        } else {
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
                } else if distance < SAFETY_DISTANCE_NORMAL * 2.5 {
                    collision_risks.push((i, j, distance, false));
                }
            }
        }
    }

    statistics.update_close_calls(&current_close_call_pairs);

    for (i, j, _distance, is_conflicting) in collision_risks {
        if is_conflicting {
            // Pass 1 already correctly Stopped the lower-priority vehicle; never promote
            // a Stopped back to Normal here — that would un-stop the yielding vehicle.
            if vehicles[i].get_velocity_level() != VelocityLevel::Stopped {
                vehicles[i].set_velocity_level(VelocityLevel::Normal);
            }
            if vehicles[j].get_velocity_level() != VelocityLevel::Stopped {
                vehicles[j].set_velocity_level(VelocityLevel::Normal);
            }
        } else {
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
    fn single_vehicle_near_intersection_gets_normal_velocity() {
        // y=700 → |700 - 450| = 250 < 400 (near zone) AND 250 > 200 (not inside)
        let mut vehicle = Vehicle::new(0, Direction::North, Route::Straight);
        vehicle.set_position(700.0, 700.0);
        let mut vehicles = vec![vehicle];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(vehicles[0].get_velocity_level(), VelocityLevel::Normal);
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
    fn two_conflicting_vehicles_near_intersection_both_get_normal() {
        // North/Straight and East/Straight have crossing paths.
        // Both near intersection → Pass 2 detects conflict → both set to Normal.
        let mut north = Vehicle::new(0, Direction::North, Route::Straight);
        let mut east = Vehicle::new(1, Direction::East, Route::Straight);
        // y=700: |700-450|=250 < 400 → near zone; |250| > 200 → not in intersection
        north.set_position(700.0, 700.0);
        // x=400: |400-700|=300 < 400 → near zone; |300| > 200 → not in intersection
        east.set_position(400.0, 450.0);
        let mut vehicles = vec![north, east];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(vehicles[0].get_velocity_level(), VelocityLevel::Normal, "North vehicle should be Normal due to conflict");
        assert_eq!(vehicles[1].get_velocity_level(), VelocityLevel::Normal, "East vehicle should be Normal due to conflict");
    }
}
