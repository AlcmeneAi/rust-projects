mod vehicle;
mod intersection;
mod renderer;
mod input;
mod statistics;
mod physics;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use std::time::{Duration, Instant};
use vehicle::Vehicle;
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

    let frame_time = Duration::from_millis(1000 / FPS as u64);

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
            vehicle.update(1.0 / FPS as f32);
            if vehicle.has_left_intersection(&intersection) {
                statistics.record_vehicle(vehicle);
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
