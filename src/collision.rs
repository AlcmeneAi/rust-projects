use crate::vehicle::{Vehicle, VelocityLevel, Route};
use crate::intersection::{Intersection, Direction};
use crate::statistics::Statistics;

const SAFETY_DISTANCE_FAST: f32 = 100.0;
const SAFETY_DISTANCE_NORMAL: f32 = 70.0;
const SAFETY_DISTANCE_SLOW: f32 = 40.0;
const INTERSECTION_PADDING: f32 = 200.0;

pub fn check_collisions_and_apply_strategy(
    vehicles: &mut Vec<Vehicle>,
    intersection: &Intersection,
    statistics: &mut Statistics,
) {
    // First pass: detect all potential collisions based on actual paths through intersection
    let mut collision_risks = Vec::new();
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

            if v1_near && v2_near {
                if will_paths_conflict(v1_dir, v1_route, v2_dir, v2_route) {
                    collision_risks.push((i, j, distance, true));
                    statistics.record_close_call();
                } else if distance < SAFETY_DISTANCE_NORMAL * 2.5 {
                    collision_risks.push((i, j, distance, false));
                }
            }
        }
    }

    // Apply velocity level control for collision avoidance
    for (i, j, _distance, is_conflicting) in collision_risks {
        if is_conflicting {
            vehicles[i].set_velocity_level(VelocityLevel::Slow);
            vehicles[j].set_velocity_level(VelocityLevel::Slow);
        } else {
            let relative_velocity = vehicles[i].get_velocity() - vehicles[j].get_velocity();
            if relative_velocity > 5.0 {
                let current_level = vehicles[i].get_velocity_level();
                let new_level = match current_level {
                    VelocityLevel::Fast => VelocityLevel::Normal,
                    VelocityLevel::Normal => VelocityLevel::Slow,
                    VelocityLevel::Slow => VelocityLevel::Slow,
                };
                vehicles[i].set_velocity_level(new_level);
            } else if relative_velocity < -5.0 {
                let current_level = vehicles[j].get_velocity_level();
                let new_level = match current_level {
                    VelocityLevel::Fast => VelocityLevel::Normal,
                    VelocityLevel::Normal => VelocityLevel::Slow,
                    VelocityLevel::Slow => VelocityLevel::Slow,
                };
                vehicles[j].set_velocity_level(new_level);
            }
        }
    }

    // Third pass: enforce safety distance on same lane/direction
    for i in 0..vehicles.len() {
        let v_i_pos = vehicles[i].get_position();
        let v_i_dir = vehicles[i].get_direction();
        let v_i_lane = vehicles[i].get_assigned_lane();
        let v_i_level = vehicles[i].get_velocity_level();

        let required_safety_distance = match v_i_level {
            VelocityLevel::Fast => SAFETY_DISTANCE_FAST,
            VelocityLevel::Normal => SAFETY_DISTANCE_NORMAL,
            VelocityLevel::Slow => SAFETY_DISTANCE_SLOW,
        };

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

                if is_ahead && distance < required_safety_distance {
                    let v_j_level = vehicles[j].get_velocity_level();
                    let new_level = match (v_i_level, v_j_level) {
                        (VelocityLevel::Fast, VelocityLevel::Fast) => VelocityLevel::Normal,
                        (VelocityLevel::Fast, _) => VelocityLevel::Normal,
                        (VelocityLevel::Normal, VelocityLevel::Slow) => VelocityLevel::Slow,
                        (VelocityLevel::Normal, _) => VelocityLevel::Normal,
                        (VelocityLevel::Slow, _) => VelocityLevel::Slow,
                    };
                    vehicles[i].set_velocity_level(new_level);
                    statistics.record_close_call();
                }
            }
        }
    }

    // Second pass: apply velocity based on traffic conditions and intersection state
    for i in 0..vehicles.len() {
        let pos = vehicles[i].get_position();
        let dir = vehicles[i].get_direction();
        let route = vehicles[i].get_route();

        let in_intersection = intersection.contains_point(pos);
        let near_intersection = is_in_or_near_intersection(pos, intersection, INTERSECTION_PADDING);

        if in_intersection {
            vehicles[i].set_velocity_level(VelocityLevel::Slow);
        } else if near_intersection {
            let mut has_conflicts = false;
            for j in 0..vehicles.len() {
                if i != j {
                    let other_pos = vehicles[j].get_position();
                    let other_dir = vehicles[j].get_direction();
                    let other_route = vehicles[j].get_route();

                    if is_in_or_near_intersection(other_pos, intersection, INTERSECTION_PADDING) {
                        if will_paths_conflict(dir, route, other_dir, other_route) {
                            has_conflicts = true;
                            break;
                        }
                    }
                }
            }

            if has_conflicts {
                vehicles[i].set_velocity_level(VelocityLevel::Slow);
            } else {
                vehicles[i].set_velocity_level(VelocityLevel::Normal);
            }
        } else {
            vehicles[i].set_velocity_level(VelocityLevel::Fast);
        }
    }
}

pub fn will_paths_conflict(dir1: Direction, route1: Route, dir2: Direction, route2: Route) -> bool {
    let final_dir1 = apply_route_to_direction(dir1, route1);
    let final_dir2 = apply_route_to_direction(dir2, route2);
    are_directions_conflicting(final_dir1, final_dir2)
        || are_paths_intersecting_at_center(dir1, route1, dir2, route2)
}

fn apply_route_to_direction(direction: Direction, route: Route) -> Direction {
    match (direction, route) {
        (Direction::North, Route::Left) => Direction::West,
        (Direction::North, Route::Straight) => Direction::North,
        (Direction::North, Route::Right) => Direction::East,
        (Direction::South, Route::Left) => Direction::East,
        (Direction::South, Route::Straight) => Direction::South,
        (Direction::South, Route::Right) => Direction::West,
        (Direction::East, Route::Left) => Direction::North,
        (Direction::East, Route::Straight) => Direction::East,
        (Direction::East, Route::Right) => Direction::South,
        (Direction::West, Route::Left) => Direction::South,
        (Direction::West, Route::Straight) => Direction::West,
        (Direction::West, Route::Right) => Direction::North,
    }
}

fn are_paths_intersecting_at_center(dir1: Direction, route1: Route, dir2: Direction, route2: Route) -> bool {
    match (dir1, route1, dir2, route2) {
        (Direction::North, Route::Straight, Direction::East, Route::Straight) => true,
        (Direction::East, Route::Straight, Direction::North, Route::Straight) => true,
        (Direction::North, Route::Right, Direction::South, Route::Left) => true,
        (Direction::South, Route::Left, Direction::North, Route::Right) => true,
        (Direction::North, Route::Right, Direction::East, Route::Straight) => true,
        (Direction::East, Route::Straight, Direction::North, Route::Right) => true,
        (Direction::East, Route::Right, Direction::North, Route::Right) => true,
        (Direction::North, Route::Right, Direction::East, Route::Right) => true,
        (Direction::South, Route::Straight, Direction::West, Route::Straight) => true,
        (Direction::West, Route::Straight, Direction::South, Route::Straight) => true,
        (Direction::South, Route::Right, Direction::North, Route::Left) => true,
        (Direction::North, Route::Left, Direction::South, Route::Right) => true,
        (Direction::West, Route::Right, Direction::South, Route::Right) => true,
        (Direction::South, Route::Right, Direction::West, Route::Right) => true,
        (Direction::East, Route::Left, Direction::West, Route::Right) => true,
        (Direction::West, Route::Right, Direction::East, Route::Left) => true,
        _ => false,
    }
}

pub fn is_in_or_near_intersection(pos: (f32, f32), intersection: &Intersection, padding: f32) -> bool {
    let dx = (pos.0 - intersection.center.0).abs();
    let dy = (pos.1 - intersection.center.1).abs();
    dx < intersection.size + padding && dy < intersection.size + padding
}

fn are_directions_conflicting(dir1: Direction, dir2: Direction) -> bool {
    match (dir1, dir2) {
        (Direction::North, Direction::South) | (Direction::South, Direction::North) => true,
        (Direction::East, Direction::West) | (Direction::West, Direction::East) => true,
        (Direction::North, Direction::East) | (Direction::East, Direction::North) => true,
        (Direction::North, Direction::West) | (Direction::West, Direction::North) => true,
        (Direction::South, Direction::East) | (Direction::East, Direction::South) => true,
        (Direction::South, Direction::West) | (Direction::West, Direction::South) => true,
        _ => false,
    }
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
    // Near zone: |x - 700| < 400 AND |y - 450| < 400  → for x=700: y in (50, 850)
    // In-intersection: |x - 700| < 200 AND |y - 450| < 200
    // Far zone: |y - 450| >= 400 → for x=700: y <= 50 or y >= 850

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
    fn vehicle_in_intersection_gets_slow_velocity() {
        // (700, 450) is the center — inside on both axes (|0| < 200)
        let mut vehicle = Vehicle::new(0, Direction::North, Route::Straight);
        vehicle.set_position(700.0, 450.0);
        let mut vehicles = vec![vehicle];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(vehicles[0].get_velocity_level(), VelocityLevel::Slow);
    }

    #[test]
    fn vehicle_near_intersection_no_conflicts_gets_normal_velocity() {
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
            "follower within safety distance of leader must be reduced from Fast to Normal"
        );
    }
}
