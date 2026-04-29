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
use std::time::{Duration, Instant};
use vehicle::Vehicle;
use intersection::{Intersection, Direction};
use renderer::Renderer;
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
                    }
                    _ => {}
                },
            }
        }

        if state == AppState::Running {
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

                if intersection.is_near_or_in(vehicle.get_position(), 50.0)
                    && !vehicle.has_entered_intersection()
                {
                    vehicle.mark_intersection_entry(total_time);
                }

                if vehicle.should_apply_route_turn(intersection.center, 50.0) {
                    vehicle.apply_route_turn();
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
                for vehicle in &vehicles {
                    renderer.draw_vehicle(vehicle)?;
                    if debug_mode {
                        renderer.draw_vehicle_debug(vehicle)?;
                    }
                }
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
