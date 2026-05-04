mod vehicle;
mod intersection;
mod renderer;
mod input;
mod statistics;
mod animation;
mod collision;
mod summary;
mod glyphs;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::mouse::MouseButton;
use sdl2::image::{self, InitFlag, LoadTexture};
use std::time::{Duration, Instant};
use vehicle::Vehicle;
use vehicle::Route;
use intersection::{Intersection, Direction};
use renderer::{Renderer, CarTextures};
use input::InputHandler;
use statistics::Statistics;

#[derive(PartialEq, Eq, Clone, Copy)]
enum AppState {
    Running,
    Summary,
}

const WINDOW_WIDTH: u32 = 1400;
const WINDOW_HEIGHT: u32 = 900;
const FPS: u32 = 60;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let sdl_context = sdl2::init()?;
    let _image_context = image::init(InitFlag::PNG)?;
    let video_subsystem = sdl_context.video()?;

    let window = video_subsystem
        .window("Smart Road - Traffic Simulation", WINDOW_WIDTH, WINDOW_HEIGHT)
        .position_centered()
        .build()?;

    let canvas = window.into_canvas().build()?;
    let texture_creator = canvas.texture_creator();

    let mut renderer = Renderer::new(canvas, &texture_creator, WINDOW_WIDTH, WINDOW_HEIGHT)?;
    let car_textures = CarTextures {
        base: texture_creator.load_texture("assets/car_up.png")?,
    };
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
    let mut debug_mode = false;   // D key toggles hitbox/safety-radius overlay
    let mut state = AppState::Running;

    let frame_time = Duration::from_millis(1000 / FPS as u64);
    let dt = 1.0 / FPS as f32;  // Delta time per frame

    'running: loop {
        let frame_start = Instant::now();

        for event in event_pump.poll_iter() {
            match state {
                AppState::Running => match event {
                    Event::Quit { .. } => {
                        enter_summary(&mut state, &mut vehicles, &mut statistics, total_time);
                    }
                    Event::KeyDown { keycode: Some(code), .. } => match code {
                        Keycode::Escape => {
                            enter_summary(&mut state, &mut vehicles, &mut statistics, total_time);
                        }
                        Keycode::Up    => input_handler.request_vehicle_from_direction(Direction::North),
                        Keycode::Down  => input_handler.request_vehicle_from_direction(Direction::South),
                        Keycode::Right => input_handler.request_vehicle_from_direction(Direction::East),
                        Keycode::Left  => input_handler.request_vehicle_from_direction(Direction::West),
                        Keycode::R     => input_handler.toggle_random_generation(),
                        Keycode::D     => {
                            debug_mode = !debug_mode;
                            eprintln!("[DEBUG] hitbox overlay = {}", debug_mode);
                        }
                        _ => {}
                    },
                    _ => {}
                },
                AppState::Summary => match event {
                    Event::Quit { .. } => break 'running,
                    Event::KeyDown {
                        keycode: Some(Keycode::Escape) | Some(Keycode::Return) | Some(Keycode::KpEnter),
                        ..
                    } => break 'running,
                    Event::MouseButtonDown { mouse_btn: MouseButton::Left, x, y, .. } => {
                        let canvas_size = nalgebra::Vector2::new(WINDOW_WIDTH as i32, WINDOW_HEIGHT as i32);
                        if summary::point_in_ok_button(canvas_size, x, y) {
                            break 'running;
                        }
                        if summary::point_in_restart_button(canvas_size, x, y) {
                            // Reset everything and go back to Running
                            vehicles.clear();
                            statistics.reset();
                            frame_count = 0;
                            vehicle_counter = 0;
                            total_time = 0.0;
                            input_handler = InputHandler::new();
                            state = AppState::Running;
                        }
                    }
                    _ => {}
                },
            }
        }

        if state == AppState::Running {
            if input_handler.should_generate_random() {
                let direction = Direction::random();
                // Per-direction cooldown: skip if a vehicle was recently spawned
                // from this direction to avoid vehicles spawning on top of each other.
                if input_handler.poll_random_spawn(direction) {
                    let route = vehicle::Route::random();
                    let vehicle = Vehicle::new(vehicle_counter, direction, route);
                    vehicles.push(vehicle);
                    vehicle_counter += 1;
                    statistics.increment_generated();
                }
            }

            while let Some(direction) = input_handler.next_vehicle_request() {
                let route = vehicle::Route::random();
                let vehicle = Vehicle::new(vehicle_counter, direction, route);
                vehicles.push(vehicle);
                vehicle_counter += 1;
                statistics.increment_generated();
            }

            let mut vehicles_to_remove = Vec::new();

            for (i, vehicle) in vehicles.iter_mut().enumerate() {
                // ── Turn signal: activate within 200 px of entry edge for Left/Right ──
                const TURN_SIGNAL_DISTANCE: f32 = 200.0;
                if !vehicle.is_route_applied() && vehicle.get_route() != Route::Straight {
                    let pos = vehicle.get_position();
                    let half = intersection.size;
                    let dist_to_entry = match vehicle.get_direction() {
                        Direction::North => pos.1 - (intersection.center.1 + half),
                        Direction::South => (intersection.center.1 - half) - pos.1,
                        Direction::East  => (intersection.center.0 - half) - pos.0,
                        Direction::West  => pos.0 - (intersection.center.0 + half),
                    };
                    if dist_to_entry > 0.0 && dist_to_entry <= TURN_SIGNAL_DISTANCE {
                        vehicle.activate_turn_signal();
                    } else {
                        vehicle.deactivate_turn_signal();
                    }
                } else {
                    vehicle.deactivate_turn_signal();
                }

                vehicle.update(dt);

                // Entry: fire only when the vehicle has crossed into the intersection box.
                if intersection.contains_point(vehicle.get_position())
                    && !vehicle.has_entered_intersection()
                {
                    vehicle.mark_intersection_entry(total_time);
                    // Start rotating the sprite toward the final heading immediately
                    // upon entry so the visual turn is tied to position, not to the
                    // discrete snap event that fires mid-intersection.
                    vehicle.begin_turn_animation();
                }

                if vehicle.should_apply_route_turn(intersection.center, intersection.size) {
                    vehicle.apply_route_turn();
                }

                // Exit: fire as soon as the vehicle has turned *and* physically left the box.
                // Recording here (not at off-screen removal) gives an accurate distance/time
                // window limited to the actual intersection traversal.
                if vehicle.has_entered_intersection()
                    && !vehicle.has_exited_intersection()
                    && vehicle.is_route_applied()
                    && !intersection.contains_point(vehicle.get_position())
                {
                    vehicle.mark_intersection_exit(total_time);
                    statistics.record_vehicle(vehicle);
                }

                let pos = vehicle.get_position();
                let off_screen = pos.0 < -300.0 || pos.0 > (WINDOW_WIDTH as f32 + 300.0) ||
                                pos.1 < -300.0 || pos.1 > (WINDOW_HEIGHT as f32 + 300.0);
                if off_screen {
                    vehicles_to_remove.push(i);
                }
            }

            for i in vehicles_to_remove.iter().rev() {
                if vehicles[*i].has_entered_intersection() && !vehicles[*i].has_exited_intersection() {
                    vehicles[*i].mark_intersection_exit(total_time);
                    statistics.record_vehicle(&vehicles[*i]);
                }
                vehicles.remove(*i);
            }

            collision::check_collisions_and_apply_strategy(&mut vehicles, &intersection, &mut statistics);
        }

        match state {
            AppState::Running => {
                renderer.clear();
                renderer.draw_intersection(&intersection)?;
                if debug_mode {
                    renderer.draw_intersection_zones_debug(&intersection)?;
                }
                for vehicle in &vehicles {
                    renderer.draw_vehicle(vehicle, &car_textures)?;
                    if debug_mode {
                        renderer.draw_vehicle_debug(vehicle, &vehicles, &intersection)?;
                    }
                }
                renderer.draw_hud(
                    input_handler.is_random_generation_enabled(),
                    vehicles.len(),
                    debug_mode,
                )?;
                renderer.present();
            }
            AppState::Summary => {
                let mouse = event_pump.mouse_state();
                renderer.draw_summary(&statistics, total_time, (mouse.x(), mouse.y()))?;
                renderer.present();
            }
        }

        let elapsed = frame_start.elapsed();
        if elapsed < frame_time {
            std::thread::sleep(frame_time - elapsed);
        }

        if state == AppState::Running {
            total_time += dt;
            frame_count += 1;
        }
    }

    print_console_summary(&statistics, total_time);
    Ok(())
}

/// One-shot transition from Running to Summary. Finalizes any vehicles still
/// on-screen so the summary reflects the whole session, then prints the formatted
/// console summary so it lands in scrollback before the user finally exits.
fn enter_summary(
    state: &mut AppState,
    vehicles: &mut Vec<Vehicle>,
    statistics: &mut Statistics,
    total_time: f32,
) {
    if *state == AppState::Summary { return; }
    for v in vehicles.iter_mut() {
        if v.has_entered_intersection() && !v.has_exited_intersection() {
            v.mark_intersection_exit(total_time);
            statistics.record_vehicle(v);
        }
    }
    print_console_summary(statistics, total_time);
    *state = AppState::Summary;
}

#[allow(dead_code)]
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

#[allow(dead_code)]
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

fn print_console_summary(stats: &Statistics, total_time: f32) {
    if stats.get_vehicle_count() == 0 {
        println!();
        println!("╔══════════════════════════════════════════════════════════════╗");
        println!("║              SMART ROAD — SESSION SUMMARY                    ║");
        println!("╠══════════════════════════════════════════════════════════════╣");
        println!("║  Session duration ......................  {:>7.2} s          ║", total_time);
        println!("║  No vehicles completed the intersection.                     ║");
        println!("╚══════════════════════════════════════════════════════════════╝");
        println!();
        return;
    }

    println!();
    println!("╔═════════════════════════════════════════════════════════════════════╗");
    println!("║              SMART ROAD — SESSION SUMMARY                           ║");
    println!("╠═════════════════════════════════════════════════════════════════════╣");
    println!("║  Session duration ......................  {:>7.2} s                 ║", total_time);
    println!("║  Vehicles passed .......................  {:>7}                   ║", stats.get_vehicle_count());
    println!("║  Close calls ...........................  {:>7}                   ║", stats.get_close_calls());
    println!("║  Collisions ............................  {:>7}                   ║", stats.get_collisions());
    println!("╠── Velocity (px/s) ──────────────────────────────────────────────────╣");
    println!("║    min / mean / max ....................  {:>6.2} / {:>6.2} / {:>6.2}  ║",
        stats.get_min_velocity(), stats.get_mean_velocity(), stats.get_max_velocity());
    println!("║    std deviation .......................  {:>6.2}                    ║", stats.get_stddev_velocity());
    println!("╠── Time in intersection (s) ─────────────────────────────────────────╣");
    println!("║    min / mean / max ....................  {:>6.2} / {:>6.2} / {:>6.2}  ║",
        stats.get_min_time(), stats.get_mean_time(), stats.get_max_time());
    println!("║    std deviation .......................  {:>6.2}                    ║", stats.get_stddev_time());
    println!("╚═════════════════════════════════════════════════════════════════════╝");
    println!();
}
