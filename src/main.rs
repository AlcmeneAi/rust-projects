mod vehicle;
mod intersection;
mod renderer;
mod input;
mod statistics;
mod physics;

use minifb::{Window, WindowOptions, Key};
use vehicle::Vehicle;
use intersection::{Intersection, Direction};
use renderer::Renderer;
use input::InputHandler;
use statistics::Statistics;

const WINDOW_WIDTH: usize = 1400;
const WINDOW_HEIGHT: usize = 900;
const FPS: u32 = 60;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut window = Window::new(
        "Smart Road - Traffic Simulation",
        WINDOW_WIDTH,
        WINDOW_HEIGHT,
        WindowOptions::default(),
    )?;

    window.limit_update_rate(Some(std::time::Duration::from_millis(1000 / FPS as u64)));

    let mut renderer = Renderer::new(WINDOW_WIDTH as u32, WINDOW_HEIGHT as u32)?;
    let mut input_handler = InputHandler::new();

    let intersection = Intersection::new(
        WINDOW_WIDTH as f32 / 2.0,
        WINDOW_HEIGHT as f32 / 2.0,
        200.0,
    );

    let mut vehicles: Vec<Vehicle> = Vec::new();
    let mut statistics = Statistics::new();
    let mut frame_count = 0u64;
    let mut vehicle_counter = 0u32;
    let mut total_time = 0.0f32;

    'running: loop {
        if !window.is_open() {
            break 'running;
        }

        // Handle keyboard
        if window.is_key_pressed(Key::Escape, minifb::KeyRepeat::No) {
            break 'running;
        }

        if window.is_key_pressed(Key::Up, minifb::KeyRepeat::No) {
            input_handler.request_vehicle_from_direction(Direction::South);
        }
        if window.is_key_pressed(Key::Down, minifb::KeyRepeat::No) {
            input_handler.request_vehicle_from_direction(Direction::North);
        }
        if window.is_key_pressed(Key::Right, minifb::KeyRepeat::No) {
            input_handler.request_vehicle_from_direction(Direction::West);
        }
        if window.is_key_pressed(Key::Left, minifb::KeyRepeat::No) {
            input_handler.request_vehicle_from_direction(Direction::East);
        }
        if window.is_key_pressed(Key::R, minifb::KeyRepeat::No) {
            input_handler.toggle_random_generation();
        }

        let dt = 1.0 / FPS as f32;
        total_time += dt;

        // Generate vehicles
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

        // Update and remove vehicles
        let mut vehicles_to_remove = Vec::new();

        for (i, vehicle) in vehicles.iter_mut().enumerate() {
            vehicle.update(dt);

            if !vehicle.is_in_intersection_tracked() && intersection.contains_point(vehicle.get_position()) {
                vehicle.set_entered_intersection(total_time);
            }

            if vehicle.has_left_intersection(&intersection) {
                statistics.record_vehicle(vehicle);
                vehicles_to_remove.push(i);
            }
        }

        for i in vehicles_to_remove.iter().rev() {
            vehicles.remove(*i);
        }

        // Smart intersection management
        check_collisions_and_apply_strategy(&mut vehicles, &intersection, &mut statistics);

        // Render
        renderer.clear();
        renderer.draw_intersection(&intersection)?;

        for vehicle in &vehicles {
            renderer.draw_vehicle(vehicle)?;
        }

        renderer.present(&window)?;

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
    const SAFETY_DISTANCE: f32 = 15.0;

    let mut collisions = Vec::new();
    for i in 0..vehicles.len() {
        for j in (i + 1)..vehicles.len() {
            let v1_pos = vehicles[i].get_position();
            let v2_pos = vehicles[j].get_position();

            let distance = ((v1_pos.0 - v2_pos.0).powi(2) + (v1_pos.1 - v2_pos.1).powi(2)).sqrt();

            if intersection.contains_point(v1_pos) && intersection.contains_point(v2_pos) {
                if distance < SAFETY_DISTANCE * 2.0 {
                    collisions.push((i, j));
                }
            }
        }
    }

    for (i, j) in collisions {
        statistics.record_close_call();
        let relative_velocity = vehicles[i].get_velocity() - vehicles[j].get_velocity();
        if relative_velocity > 0.0 {
            vehicles[i].reduce_velocity(0.5);
        } else if relative_velocity < 0.0 {
            vehicles[j].reduce_velocity(0.5);
        }
    }

    // Smart velocity management
    for i in 0..vehicles.len() {
        let pos = vehicles[i].get_position();

        if intersection.contains_point(pos) {
            let mut vehicles_ahead = 0;
            for j in 0..vehicles.len() {
                if i != j {
                    let other_pos = vehicles[j].get_position();
                    if intersection.contains_point(other_pos) {
                        if is_vehicle_ahead(&vehicles[i], &vehicles[j]) {
                            vehicles_ahead += 1;
                        }
                    }
                }
            }

            match vehicles_ahead {
                0 => vehicles[i].set_velocity(150.0),
                1 => vehicles[i].set_velocity(100.0),
                _ => vehicles[i].set_velocity(50.0),
            }
        } else if is_approaching_intersection(&vehicles[i], intersection) {
            vehicles[i].set_velocity(100.0);
        } else {
            vehicles[i].set_velocity(150.0);
        }
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

    'running: loop {
        let frame_start = Instant::now();

        // Handle events
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
                            input_handler.request_vehicle_from_direction(Direction::South);
                        }
                        Keycode::Down => {
                            input_handler.request_vehicle_from_direction(Direction::North);
                        }
                        Keycode::Right => {
                            input_handler.request_vehicle_from_direction(Direction::West);
                        }
                        Keycode::Left => {
                            input_handler.request_vehicle_from_direction(Direction::East);
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

        // Generate vehicles from input
        if input_handler.should_generate_random() && frame_count % 30 == 0 {
            let direction = Direction::random();
            let route = vehicle::Route::random();
            let vehicle = Vehicle::new(vehicle_counter, direction, route);
            vehicles.push(vehicle);
            vehicle_counter += 1;
        }

        // Generate vehicles from keyboard input
        while let Some(direction) = input_handler.next_vehicle_request() {
            let route = vehicle::Route::random();
            let vehicle = Vehicle::new(vehicle_counter, direction, route);
            vehicles.push(vehicle);
            vehicle_counter += 1;
        }

        // Update vehicles
        let mut vehicles_to_remove = Vec::new();

        for (i, vehicle) in vehicles.iter_mut().enumerate() {
            vehicle.update(1.0 / FPS as f32);

            // Check if vehicle has left the intersection
            if vehicle.has_left_intersection(&intersection) {
                statistics.record_vehicle(vehicle);
                vehicles_to_remove.push(i);
            }
        }

        // Remove exited vehicles (in reverse order to maintain indices)
        for i in vehicles_to_remove.iter().rev() {
            vehicles.remove(*i);
        }

        // Check for collisions and apply smart intersection management
        check_collisions_and_apply_strategy(&mut vehicles, &intersection, &mut statistics);

        // Render
        renderer.clear();
        renderer.draw_intersection(&intersection)?;

        for vehicle in &vehicles {
            renderer.draw_vehicle(vehicle)?;
        }

        renderer.present();

        // Frame rate limiting
        let elapsed = frame_start.elapsed();
        if elapsed < frame_time {
            std::thread::sleep(frame_time - elapsed);
        }

        frame_count += 1;
    }

    // Display statistics
    display_statistics(&statistics)?;

    Ok(())
}

fn check_collisions_and_apply_strategy(
    vehicles: &mut Vec<Vehicle>,
    intersection: &Intersection,
    statistics: &mut Statistics,
) {
    const SAFETY_DISTANCE: f32 = 15.0;

    // Check for collisions between all vehicle pairs
    let mut collisions = Vec::new();
    for i in 0..vehicles.len() {
        for j in (i + 1)..vehicles.len() {
            let v1_pos = vehicles[i].get_position();
            let v2_pos = vehicles[j].get_position();

            let distance = ((v1_pos.0 - v2_pos.0).powi(2) + (v1_pos.1 - v2_pos.1).powi(2)).sqrt();

            // Check if this is in the intersection area
            if intersection.contains_point(v1_pos) && intersection.contains_point(v2_pos) {
                if distance < SAFETY_DISTANCE * 2.0 {
                    collisions.push((i, j));
                }
            }
        }
    }

    // Apply collision responses
    for (i, j) in collisions {
        statistics.record_close_call();
        let relative_velocity = vehicles[i].get_velocity() - vehicles[j].get_velocity();
        if relative_velocity > 0.0 {
            vehicles[i].reduce_velocity(0.5);
        } else if relative_velocity < 0.0 {
            vehicles[j].reduce_velocity(0.5);
        }
    }

    // Smart intersection algorithm: manage velocity based on traffic
    for i in 0..vehicles.len() {
        let pos = vehicles[i].get_position();
        let vehicle_id = vehicles[i].get_id();

        if intersection.contains_point(pos) {
            // Count vehicles in intersection from different directions
            let mut vehicles_ahead = 0;
            for j in 0..vehicles.len() {
                if i != j && vehicles[j].get_id() != vehicle_id {
                    let other_pos = vehicles[j].get_position();
                    if intersection.contains_point(other_pos) {
                        if is_vehicle_ahead(&vehicles[i], &vehicles[j], intersection) {
                            vehicles_ahead += 1;
                        }
                    }
                }
            }

            // Adjust velocity based on traffic
            match vehicles_ahead {
                0 => vehicles[i].set_velocity(150.0), // High speed
                1 => vehicles[i].set_velocity(100.0), // Medium speed
                _ => vehicles[i].set_velocity(50.0),  // Low speed
            }
        } else if is_approaching_intersection(&vehicles[i], intersection) {
            // Coming up to intersection - use medium speed
            vehicles[i].set_velocity(100.0);
        } else {
            // Far from intersection - use high speed
            vehicles[i].set_velocity(150.0);
        }
    }
}

fn is_vehicle_ahead(vehicle: &Vehicle, other: &Vehicle, _intersection: &Intersection) -> bool {
    let my_pos = vehicle.get_position();
    let other_pos = other.get_position();
    let my_dir = vehicle.get_direction();
    let other_dir = other.get_direction();

    // Simplified: check if other vehicle is ahead in the same direction
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

fn display_statistics(statistics: &Statistics) -> Result<(), Box<dyn std::error::Error>> {
    let sdl_context = sdl2::init()?;
    let video_subsystem = sdl_context.video()?;

    let window = video_subsystem
        .window("Traffic Simulation Statistics", 600, 500)
        .position_centered()
        .build()?;

    let mut canvas = window.into_canvas().build()?;

    // Simple text-based display
    canvas.set_draw_color(sdl2::pixels::Color::WHITE);
    canvas.clear();

    // For now, we'll just print to console
    println!("\n========== TRAFFIC SIMULATION STATISTICS ==========");
    println!("Total Vehicles Passed: {}", statistics.get_vehicle_count());
    println!("Max Velocity: {:.2} pixels/frame", statistics.get_max_velocity());
    println!("Min Velocity: {:.2} pixels/frame", statistics.get_min_velocity());
    println!("Max Time in Intersection: {:.2} seconds", statistics.get_max_time());
    println!("Min Time in Intersection: {:.2} seconds", statistics.get_min_time());
    println!("Close Calls: {}", statistics.get_close_calls());
    println!("===================================================\n");

    canvas.present();

    // Keep window open for 5 seconds
    std::thread::sleep(Duration::from_secs(5));

    Ok(())
}
