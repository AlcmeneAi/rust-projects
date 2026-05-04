use crate::intersection::Direction;
use std::collections::VecDeque;

pub struct InputHandler {
    vehicle_requests: VecDeque<Direction>,
    random_generation_enabled: bool,
    last_key_time: std::time::Instant,
    key_cooldown: std::time::Duration,
    /// Last time a vehicle was spawned for each direction (North, South, East, West).
    /// Prevents two vehicles from entering the same lane within `SPAWN_COOLDOWN`.
    last_spawn_time: [std::time::Instant; 4],
    spawn_cooldown: std::time::Duration,
}

/// Maps Direction to an index into the per-direction cooldown array.
fn dir_index(d: Direction) -> usize {
    match d {
        Direction::North => 0,
        Direction::South => 1,
        Direction::East  => 2,
        Direction::West  => 3,
    }
}

impl InputHandler {
    pub fn new() -> Self {
        let epoch = std::time::Instant::now()
            .checked_sub(std::time::Duration::from_secs(10))
            .unwrap_or_else(std::time::Instant::now);
        InputHandler {
            vehicle_requests: VecDeque::new(),
            random_generation_enabled: false,
            last_key_time: std::time::Instant::now(),
            key_cooldown: std::time::Duration::from_millis(200),
            last_spawn_time: [epoch; 4],
            // 3 s between spawns per direction — matches intersection throughput
            // (~0.8 vehicles/s capacity) so lanes don't back up over time.
            spawn_cooldown: std::time::Duration::from_millis(3000),
        }
    }

    pub fn request_vehicle_from_direction(&mut self, direction: Direction) {
        let now = std::time::Instant::now();
        // Only the global key-repeat cooldown applies to manual spawning.
        // The per-direction spawn_cooldown is reserved for random generation
        // so the user can deliberately queue 3 cars in the same lane.
        if now.duration_since(self.last_key_time) >= self.key_cooldown {
            self.vehicle_requests.push_back(direction);
            self.last_key_time = now;
        }
    }

    pub fn toggle_random_generation(&mut self) {
        self.random_generation_enabled = !self.random_generation_enabled;
    }

    pub fn should_generate_random(&self) -> bool {
        self.random_generation_enabled
    }

    pub fn is_random_generation_enabled(&self) -> bool {
        self.random_generation_enabled
    }

    /// Returns `Some(direction)` if a vehicle may be spawned from `direction`
    /// under random generation, and records the spawn time so subsequent calls
    /// within the cooldown window return `None` for that direction.
    pub fn poll_random_spawn(&mut self, direction: Direction) -> bool {
        let now = std::time::Instant::now();
        let idx = dir_index(direction);
        if now.duration_since(self.last_spawn_time[idx]) >= self.spawn_cooldown {
            self.last_spawn_time[idx] = now;
            true
        } else {
            false
        }
    }

    pub fn next_vehicle_request(&mut self) -> Option<Direction> {
        self.vehicle_requests.pop_front()
    }
}
