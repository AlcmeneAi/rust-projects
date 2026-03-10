mod vehicle;
mod intersection;
mod renderer;
mod input;
mod statistics;
mod physics;
mod animation;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::time::{Duration, Instant};
use vehicle::{Vehicle, Route, VelocityLevel};
use intersection::{Intersection, Direction};
use renderer::Renderer;
use input::InputHandler;
use statistics::Statistics;

const WINDOW_WIDTH: u32 = 1400;
const WINDOW_HEIGHT: u32 = 900;
const FPS: u32 = 60;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let sdl_context = sdl2::init()?;
    let video_subsystem = sdl_context.video()?;

    let window = video_subsystem
        .window("Smart Road - Traffic Simulation", WINDOW_WIDTH, WINDOW_HEIGHT)
        .position_centered()
        .build()?;

    let canvas = window.into_canvas().build()?;
    let texture_creator = canvas.texture_creator();

    let mut renderer = Renderer::new(canvas, &texture_creator, WINDOW_WIDTH, WINDOW_HEIGHT)?;
    let mut input_handler = InputHandler::new();
    let mut event_pump = sdl_context.event_pump()?;

    let intersection = Intersection::new(
        WINDOW_WIDTH as f32 / 2.0,
        WINDOW_HEIGHT as f32 / 2.0,
        200.0,
    );

    let mut vehicles: Vec<Vehicle> = Vec::new();
    let mut statistics = Statistics::new();
    let mut frame_count = 0;
    let mut vehicle_counter = 0u32;
    let mut total_time = 0.0f32;  // Total elapsed time in seconds

    let frame_time = Duration::from_millis(1000 / FPS as u64);
    let dt = 1.0 / FPS as f32;  // Delta time per frame

    'running: loop {
        let frame_start = Instant::now();

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. } => {
                    break 'running;
                }
                Event::KeyDown {
                    keycode: Some(code),
                    ..
                } => {
                    match code {
                        Keycode::Escape => {
                            break 'running;
                        }
                        Keycode::Up => {
                            input_handler.request_vehicle_from_direction(Direction::North);
                        }
                        Keycode::Down => {
                            input_handler.request_vehicle_from_direction(Direction::South);
                        }
                        Keycode::Right => {
                            input_handler.request_vehicle_from_direction(Direction::East);
                        }
                        Keycode::Left => {
                            input_handler.request_vehicle_from_direction(Direction::West);
                        }
                        Keycode::R => {
                            input_handler.toggle_random_generation();
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }

        if input_handler.should_generate_random() && frame_count % 30 == 0 {
            let direction = Direction::random();
            let route = vehicle::Route::random();
            let vehicle = Vehicle::new(vehicle_counter, direction, route);
            vehicles.push(vehicle);
            vehicle_counter += 1;
        }

        while let Some(direction) = input_handler.next_vehicle_request() {
            let route = vehicle::Route::random();
            let vehicle = Vehicle::new(vehicle_counter, direction, route);
            vehicles.push(vehicle);
            vehicle_counter += 1;
        }

        let mut vehicles_to_remove = Vec::new();

        for (i, vehicle) in vehicles.iter_mut().enumerate() {
            vehicle.update(dt);
            
            // Track intersection entry
            let in_intersection = intersection.contains_point(vehicle.get_position());
            if in_intersection && !vehicle.has_entered_intersection() {
                vehicle.mark_intersection_entry(total_time);
            }
            
            // Apply route turn when vehicle reaches intersection center
            if vehicle.should_apply_route_turn(intersection.center, 50.0) {
                vehicle.apply_route_turn();
            }
            
            // Record statistics when vehicle leaves intersection
            if vehicle.has_left_intersection(&intersection) && !vehicle.has_exited_intersection() {
                vehicle.mark_intersection_exit(total_time);
                statistics.record_vehicle(vehicle);
            }
            
            // Remove vehicle when it's completely off-screen
            let pos = vehicle.get_position();
            let off_screen = pos.0 < -300.0 || pos.0 > (WINDOW_WIDTH as f32 + 300.0) || 
                            pos.1 < -300.0 || pos.1 > (WINDOW_HEIGHT as f32 + 300.0);
            if off_screen {
                vehicles_to_remove.push(i);
            }
        }

        for i in vehicles_to_remove.iter().rev() {
            vehicles.remove(*i);
        }

        check_collisions_and_apply_strategy(&mut vehicles, &intersection, &mut statistics);

        renderer.clear();
        renderer.draw_intersection(&intersection)?;

        for vehicle in &vehicles {
            renderer.draw_vehicle(vehicle)?;
        }

        renderer.present();

        let elapsed = frame_start.elapsed();
        if elapsed < frame_time {
            std::thread::sleep(frame_time - elapsed);
        }

        total_time += dt;
        frame_count += 1;
    }

    display_statistics(&statistics)?;
    Ok(())
}

fn check_collisions_and_apply_strategy(
    vehicles: &mut Vec<Vehicle>,
    intersection: &Intersection,
    statistics: &mut Statistics,
) {
    // Safety distances based on velocity level (pixels)
    // Formula: distance = velocity * reaction_time + braking_distance
    const SAFETY_DISTANCE_FAST: f32 = 100.0;   // For 160 px/f vehicles
    const SAFETY_DISTANCE_NORMAL: f32 = 70.0;  // For 100 px/f vehicles
    const SAFETY_DISTANCE_SLOW: f32 = 40.0;    // For 40 px/f vehicles
    const INTERSECTION_PADDING: f32 = 200.0;   // Zone to detect approaching vehicles

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

            // Check if vehicles are near intersection and could collide
            let v1_near = is_in_or_near_intersection(v1_pos, intersection, INTERSECTION_PADDING);
            let v2_near = is_in_or_near_intersection(v2_pos, intersection, INTERSECTION_PADDING);

            if v1_near && v2_near {
                // Check if paths would conflict in the intersection
                if will_paths_conflict(v1_dir, v1_route, v2_dir, v2_route) {
                    // High risk of collision
                    collision_risks.push((i, j, distance, true));
                    statistics.record_close_call();
                } else if distance < SAFETY_DISTANCE_NORMAL * 2.5 {
                    // Close vehicles - apply conservative approach
                    collision_risks.push((i, j, distance, false));
                }
            }
        }
    }

    // Apply velocity level control for collision avoidance
    for (i, j, _distance, is_conflicting) in collision_risks {
        if is_conflicting {
            // Conflicting paths - set both to slow velocity
            vehicles[i].set_velocity_level(VelocityLevel::Slow);
            vehicles[j].set_velocity_level(VelocityLevel::Slow);
        } else {
            // Same direction or close - reduce one vehicle's speed
            let relative_velocity = vehicles[i].get_velocity() - vehicles[j].get_velocity();
            if relative_velocity > 5.0 {
                // Vehicle i is faster, slow it down
                let current_level = vehicles[i].get_velocity_level();
                let new_level = match current_level {
                    VelocityLevel::Fast => VelocityLevel::Normal,
                    VelocityLevel::Normal => VelocityLevel::Slow,
                    VelocityLevel::Slow => VelocityLevel::Slow,
                };
                vehicles[i].set_velocity_level(new_level);
            } else if relative_velocity < -5.0 {
                // Vehicle j is faster, slow it down
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

        // Determine required safety distance based on velocity level
        let required_safety_distance = match v_i_level {
            VelocityLevel::Fast => SAFETY_DISTANCE_FAST,
            VelocityLevel::Normal => SAFETY_DISTANCE_NORMAL,
            VelocityLevel::Slow => SAFETY_DISTANCE_SLOW,
        };

        // Check against all other vehicles on same lane and direction
        for j in 0..vehicles.len() {
            if i == j {
                continue;
            }

            let v_j_pos = vehicles[j].get_position();
            let v_j_dir = vehicles[j].get_direction();
            let v_j_lane = vehicles[j].get_assigned_lane();

            // Only check vehicles on same lane and direction
            if v_i_dir == v_j_dir && v_i_lane == v_j_lane {
                let distance = ((v_i_pos.0 - v_j_pos.0).powi(2) + (v_i_pos.1 - v_j_pos.1).powi(2)).sqrt();

                // Check if vehicle i is approaching vehicle j (i.e., j is ahead in the current direction)
                let is_ahead = match v_i_dir {
                    Direction::North => v_j_pos.1 < v_i_pos.1,   // j is ahead if its y is smaller
                    Direction::South => v_j_pos.1 > v_i_pos.1,   // j is ahead if its y is larger
                    Direction::East => v_j_pos.0 > v_i_pos.0,    // j is ahead if its x is larger
                    Direction::West => v_j_pos.0 < v_i_pos.0,    // j is ahead if its x is smaller
                };

                // If vehicle j is ahead and too close, slow down vehicle i
                if is_ahead && distance < required_safety_distance {
                    // Force vehicle i to slower or equal speed to maintain distance
                    let v_j_level = vehicles[j].get_velocity_level();
                    
                    // Set i's velocity to match or be slower than j's
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
            // Already in intersection - maintain slow speed for safety
            vehicles[i].set_velocity_level(VelocityLevel::Slow);
        } else if near_intersection {
            // Approaching intersection - check for conflicts with other approaching vehicles
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
                // Conflicts detected, slow down
                vehicles[i].set_velocity_level(VelocityLevel::Slow);
            } else {
                // No conflicts, use normal speed
                vehicles[i].set_velocity_level(VelocityLevel::Normal);
            }
        } else {
            // Far from intersection - allow fast speed
            vehicles[i].set_velocity_level(VelocityLevel::Fast);
        }
    }
}

fn will_paths_conflict(dir1: Direction, route1: Route, dir2: Direction, route2: Route) -> bool {
    let final_dir1 = apply_route_to_direction(dir1, route1);
    let final_dir2 = apply_route_to_direction(dir2, route2);
    
    // Check if the final directions would collide
    are_directions_conflicting(final_dir1, final_dir2)
        || are_paths_intersecting_at_center(dir1, route1, dir2, route2)
}

fn apply_route_to_direction(direction: Direction, route: Route) -> Direction {
    match (direction, route) {
        // From North
        (Direction::North, Route::Left) => Direction::West,
        (Direction::North, Route::Straight) => Direction::North,
        (Direction::North, Route::Right) => Direction::East,
        
        // From South
        (Direction::South, Route::Left) => Direction::East,
        (Direction::South, Route::Straight) => Direction::South,
        (Direction::South, Route::Right) => Direction::West,
        
        // From East
        (Direction::East, Route::Left) => Direction::North,
        (Direction::East, Route::Straight) => Direction::East,
        (Direction::East, Route::Right) => Direction::South,
        
        // From West
        (Direction::West, Route::Left) => Direction::South,
        (Direction::West, Route::Straight) => Direction::West,
        (Direction::West, Route::Right) => Direction::North,
    }
}

fn are_paths_intersecting_at_center(dir1: Direction, route1: Route, dir2: Direction, route2: Route) -> bool {
    // Check if paths will physically cross through the center of the intersection
    match (dir1, route1, dir2, route2) {
        // North straight with East straight - they cross
        (Direction::North, Route::Straight, Direction::East, Route::Straight) => true,
        (Direction::East, Route::Straight, Direction::North, Route::Straight) => true,
        
        // North right (to East) with South left (to East) - they cross
        (Direction::North, Route::Right, Direction::South, Route::Left) => true,
        (Direction::South, Route::Left, Direction::North, Route::Right) => true,
        
        // North right (to East) with East straight - they cross
        (Direction::North, Route::Right, Direction::East, Route::Straight) => true,
        (Direction::East, Route::Straight, Direction::North, Route::Right) => true,
        
        // East right (to South) with North right (to East) - they cross
        (Direction::East, Route::Right, Direction::North, Route::Right) => true,
        (Direction::North, Route::Right, Direction::East, Route::Right) => true,
        
        // Similar patterns for other directions
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

fn is_vehicle_ahead(vehicle: &Vehicle, other: &Vehicle) -> bool {
    let my_pos = vehicle.get_position();
    let other_pos = other.get_position();
    let my_dir = vehicle.get_direction();
    let other_dir = other.get_direction();

    match my_dir {
        Direction::North => other_pos.1 < my_pos.1 && other_dir == my_dir,
        Direction::South => other_pos.1 > my_pos.1 && other_dir == my_dir,
        Direction::East => other_pos.0 > my_pos.0 && other_dir == my_dir,
        Direction::West => other_pos.0 < my_pos.0 && other_dir == my_dir,
    }
}

fn is_approaching_intersection(vehicle: &Vehicle, intersection: &Intersection) -> bool {
    let pos = vehicle.get_position();
    let approaching_distance = 150.0;

    match vehicle.get_direction() {
        Direction::North => {
            pos.1 > intersection.center.1 - intersection.size - approaching_distance
                && pos.1 > intersection.center.1
        }
        Direction::South => {
            pos.1 < intersection.center.1 + intersection.size + approaching_distance
                && pos.1 < intersection.center.1
        }
        Direction::East => {
            pos.0 < intersection.center.0 + intersection.size + approaching_distance
                && pos.0 < intersection.center.0
        }
        Direction::West => {
            pos.0 > intersection.center.0 - intersection.size - approaching_distance
                && pos.0 > intersection.center.0
        }
    }
}

fn is_in_or_near_intersection(pos: (f32, f32), intersection: &Intersection, padding: f32) -> bool {
    let dx = (pos.0 - intersection.center.0).abs();
    let dy = (pos.1 - intersection.center.1).abs();
    dx < intersection.size + padding && dy < intersection.size + padding
}

fn are_directions_conflicting(dir1: Direction, dir2: Direction) -> bool {
    match (dir1, dir2) {
        // Opposite directions always conflict
        (Direction::North, Direction::South) | (Direction::South, Direction::North) => true,
        (Direction::East, Direction::West) | (Direction::West, Direction::East) => true,
        
        // Perpendicular directions conflict at intersection
        (Direction::North, Direction::East) | (Direction::East, Direction::North) => true,
        (Direction::North, Direction::West) | (Direction::West, Direction::North) => true,
        (Direction::South, Direction::East) | (Direction::East, Direction::South) => true,
        (Direction::South, Direction::West) | (Direction::West, Direction::South) => true,
        
        // Same direction doesn't conflict
        _ => false,
    }
}

fn display_statistics(statistics: &Statistics) -> Result<(), Box<dyn std::error::Error>> {
    println!("\n========== TRAFFIC SIMULATION STATISTICS ==========");
    println!("Total Vehicles Passed: {}", statistics.get_vehicle_count());
    println!("Max Velocity: {:.2} pixels/frame", statistics.get_max_velocity());
    println!("Min Velocity: {:.2} pixels/frame", statistics.get_min_velocity());
    println!("Max Time in Intersection: {:.2} seconds", statistics.get_max_time());
    println!("Min Time in Intersection: {:.2} seconds", statistics.get_min_time());
    println!("Close Calls: {}", statistics.get_close_calls());
    println!("===================================================\n");
    Ok(())
}
