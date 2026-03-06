use crate::intersection::Direction;
use rand::Rng;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Route {
    Right,  // r
    Straight, // s
    Left,   // l
}

impl Route {
    pub fn random() -> Self {
        let mut rng = rand::thread_rng();
        match rng.gen_range(0..3) {
            0 => Route::Right,
            1 => Route::Straight,
            _ => Route::Left,
        }
    }
}

pub struct Vehicle {
    id: u32,
    position: (f32, f32),
    direction: Direction, // incoming direction
    route: Route,         // turning direction (r, s, l)
    velocity: f32,        // pixels per frame
    distance_traveled: f32,
    time_in_intersection: f32,
    entered_intersection: bool,
    in_intersection_tracked: bool,
    entry_time: f32,
}

impl Vehicle {
    pub fn new(id: u32, direction: Direction, route: Route) -> Self {
        // Start position depends on direction
        let position = match direction {
            Direction::North => (700.0, 1200.0),     // From bottom, moving up
            Direction::South => (700.0, -200.0),     // From top, moving down
            Direction::East => (-200.0, 450.0),      // From left, moving right
            Direction::West => (1600.0, 450.0),      // From right, moving left
        };

        Vehicle {
            id,
            position,
            direction,
            route,
            velocity: 100.0,
            distance_traveled: 0.0,
            time_in_intersection: 0.0,
            entered_intersection: false,
            in_intersection_tracked: false,
            entry_time: 0.0,
        }
    }

    pub fn update(&mut self, dt: f32) {
        // Move vehicle based on direction and velocity
        let distance_this_frame = self.velocity * dt;
        self.distance_traveled += distance_this_frame;

        match self.direction {
            Direction::North => {
                self.position.1 -= self.velocity * dt; // Move up
            }
            Direction::South => {
                self.position.1 += self.velocity * dt; // Move down
            }
            Direction::East => {
                self.position.0 += self.velocity * dt; // Move right
            }
            Direction::West => {
                self.position.0 -= self.velocity * dt; // Move left
            }
        }

        if self.entered_intersection {
            self.time_in_intersection += dt;
        }
    }

    pub fn set_entered_intersection(&mut self, time: f32) {
        if !self.entered_intersection {
            self.entered_intersection = true;
            self.entry_time = time;
        }
    }

    pub fn is_in_intersection_tracked(&self) -> bool {
        self.in_intersection_tracked
    }

    pub fn set_in_intersection_tracked(&mut self, tracked: bool) {
        self.in_intersection_tracked = tracked;
    }

    pub fn has_left_intersection(&self, intersection: &crate::intersection::Intersection) -> bool {
        !intersection.contains_point(self.position)
            && self.entered_intersection
            && self.time_in_intersection > 0.1
    }

    pub fn get_position(&self) -> (f32, f32) {
        self.position
    }

    pub fn get_direction(&self) -> Direction {
        self.direction
    }

    pub fn get_route(&self) -> Route {
        self.route
    }

    pub fn get_velocity(&self) -> f32 {
        self.velocity
    }

    pub fn set_velocity(&mut self, vel: f32) {
        self.velocity = vel.max(10.0).min(200.0); // Clamp between min and max
    }

    pub fn reduce_velocity(&mut self, factor: f32) {
        self.velocity *= factor;
        self.velocity = self.velocity.max(10.0);
    }

    pub fn get_distance_traveled(&self) -> f32 {
        self.distance_traveled
    }

    pub fn get_time_in_intersection(&self) -> f32 {
        self.time_in_intersection
    }

    pub fn get_id(&self) -> u32 {
        self.id
    }

    pub fn get_angle(&self) -> f32 {
        match self.direction {
            Direction::North => 270.0,  // Up
            Direction::South => 90.0,   // Down
            Direction::East => 0.0,     // Right
            Direction::West => 180.0,   // Left
        }
    }
}
