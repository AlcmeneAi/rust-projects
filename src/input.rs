use crate::intersection::Direction;
use std::collections::VecDeque;

pub struct InputHandler {
    vehicle_requests: VecDeque<Direction>,
    random_generation_enabled: bool,
    last_key_time: std::time::Instant,
    key_cooldown: std::time::Duration,
}

impl InputHandler {
    pub fn new() -> Self {
        InputHandler {
            vehicle_requests: VecDeque::new(),
            random_generation_enabled: false,
            last_key_time: std::time::Instant::now(),
            key_cooldown: std::time::Duration::from_millis(500), // 500ms cooldown between key presses
        }
    }

    pub fn request_vehicle_from_direction(&mut self, direction: Direction) {
        let now = std::time::Instant::now();
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

    pub fn next_vehicle_request(&mut self) -> Option<Direction> {
        self.vehicle_requests.pop_front()
    }
}
