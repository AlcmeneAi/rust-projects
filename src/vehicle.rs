use crate::intersection::Direction;
use crate::animation::AnimationState;
use rand::Rng;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Route {
    Right,
    Straight,
    Left,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum VelocityLevel {
    Slow,      // 40 pixels/frame - used for congestion/safety
    Normal,    // 100 pixels/frame - standard operating speed
    Fast,      // 160 pixels/frame - free flow speed
}

impl VelocityLevel {
    pub fn to_pixels_per_frame(&self) -> f32 {
        match self {
            VelocityLevel::Slow => 40.0,
            VelocityLevel::Normal => 100.0,
            VelocityLevel::Fast => 160.0,
        }
    }

    pub fn description(&self) -> &'static str {
        match self {
            VelocityLevel::Slow => "Slow (40 px/f)",
            VelocityLevel::Normal => "Normal (100 px/f)",
            VelocityLevel::Fast => "Fast (160 px/f)",
        }
    }
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
    direction: Direction,
    route: Route,
    velocity: f32,
    velocity_level: VelocityLevel,  // Control level set by intersection
    distance_traveled: f32,
    time_in_intersection: f32,
    entered_intersection: bool,
    exited_intersection: bool,  // Track if we've already recorded the exit
    animation: AnimationState,
    last_direction: Direction,
    route_applied: bool,
    assigned_lane: u32,  // 0 = left, 1 = straight, 2 = right
    
    // Physics tracking for intersection traversal
    intersection_entry_time: f32,  // Time when vehicle entered intersection (seconds)
    intersection_entry_position: (f32, f32),  // Position when vehicle entered
    intersection_exit_time: f32,   // Time when vehicle left intersection (seconds)
    intersection_distance: f32,    // Distance traveled in intersection (pixels)
    intersection_physics_velocity: f32,  // Calculated velocity = distance / time
}

impl Vehicle {
    pub fn new(id: u32, direction: Direction, route: Route) -> Self {
        // Determine which lane to assign based on route
        let assigned_lane = match route {
            Route::Left => 0,
            Route::Straight => 1,
            Route::Right => 2,
        };

        // Calculate initial position based on direction and assigned lane
        let (x, y) = Self::calculate_initial_position(direction, assigned_lane);

        // Start with normal velocity level
        let velocity_level = VelocityLevel::Normal;
        let velocity = velocity_level.to_pixels_per_frame();

        Vehicle {
            id,
            position: (x, y),
            direction,
            route,
            velocity,
            velocity_level,
            distance_traveled: 0.0,
            time_in_intersection: 0.0,
            entered_intersection: false,
            exited_intersection: false,
            animation: AnimationState::new(direction),
            last_direction: direction,
            route_applied: false,
            assigned_lane,
            intersection_entry_time: 0.0,
            intersection_entry_position: (x, y),
            intersection_exit_time: 0.0,
            intersection_distance: 0.0,
            intersection_physics_velocity: 0.0,
        }
    }

    fn calculate_initial_position(direction: Direction, lane: u32) -> (f32, f32) {
        // Lane width is 50 pixels, and center of each lane starts at 25 pixels from edge
        let lane_offset = 25.0 + (lane as f32 * 50.0);
        
        match direction {
            Direction::North => {
                // Coming from North, lanes are horizontal
                // Left lane: x = 650, Middle: 700, Right: 750
                let x = 650.0 + (lane as f32 * 50.0);
                (x, 1200.0)
            }
            Direction::South => {
                // Coming from South, lanes are horizontal
                let x = 650.0 + (lane as f32 * 50.0);
                (x, -200.0)
            }
            Direction::East => {
                // Coming from East, lanes are vertical
                // Left lane: y = 400, Middle: 450, Right: 500
                let y = 400.0 + (lane as f32 * 50.0);
                (-200.0, y)
            }
            Direction::West => {
                // Coming from West, lanes are vertical
                let y = 400.0 + (lane as f32 * 50.0);
                (1600.0, y)
            }
        }
    }

    pub fn update(&mut self, dt: f32) {
        let distance_this_frame = self.velocity * dt;
        self.distance_traveled += distance_this_frame;

        match self.direction {
            Direction::North => {
                self.position.1 -= self.velocity * dt;
                // Constrain to assigned lane (horizontal lanes)
                self.enforce_lane_constraint_north();
            }
            Direction::South => {
                self.position.1 += self.velocity * dt;
                // Constrain to assigned lane (horizontal lanes)
                self.enforce_lane_constraint_south();
            }
            Direction::East => {
                self.position.0 += self.velocity * dt;
                // Constrain to assigned lane (vertical lanes)
                self.enforce_lane_constraint_east();
            }
            Direction::West => {
                self.position.0 -= self.velocity * dt;
                // Constrain to assigned lane (vertical lanes)
                self.enforce_lane_constraint_west();
            }
        }

        // Update animation if direction changed
        if self.direction != self.last_direction {
            self.animation.set_direction(self.direction);
            self.last_direction = self.direction;
        }

        // Update animation state
        self.animation.update(dt);

        if self.entered_intersection {
            self.time_in_intersection += dt;
        }
    }

    fn enforce_lane_constraint_north(&mut self) {
        // North bound: lanes are at x = 650, 700, 750 (centered)
        let target_x = 650.0 + (self.assigned_lane as f32 * 50.0);
        let lane_width = 50.0;
        let tolerance = lane_width / 2.0 - 5.0;
        
        if (self.position.0 - target_x).abs() > tolerance {
            self.position.0 = target_x;
        }
    }

    fn enforce_lane_constraint_south(&mut self) {
        // South bound: lanes are at x = 650, 700, 750 (centered)
        let target_x = 650.0 + (self.assigned_lane as f32 * 50.0);
        let lane_width = 50.0;
        let tolerance = lane_width / 2.0 - 5.0;
        
        if (self.position.0 - target_x).abs() > tolerance {
            self.position.0 = target_x;
        }
    }

    fn enforce_lane_constraint_east(&mut self) {
        // East bound: lanes are at y = 400, 450, 500 (centered)
        let target_y = 400.0 + (self.assigned_lane as f32 * 50.0);
        let lane_width = 50.0;
        let tolerance = lane_width / 2.0 - 5.0;
        
        if (self.position.1 - target_y).abs() > tolerance {
            self.position.1 = target_y;
        }
    }

    fn enforce_lane_constraint_west(&mut self) {
        // West bound: lanes are at y = 400, 450, 500 (centered)
        let target_y = 400.0 + (self.assigned_lane as f32 * 50.0);
        let lane_width = 50.0;
        let tolerance = lane_width / 2.0 - 5.0;
        
        if (self.position.1 - target_y).abs() > tolerance {
            self.position.1 = target_y;
        }
    }

    pub fn set_entered_intersection(&mut self) {
        if !self.entered_intersection {
            self.entered_intersection = true;
        }
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

    pub fn get_velocity_level(&self) -> VelocityLevel {
        self.velocity_level
    }

    /// Set velocity by controlling the velocity level (Slow, Normal, Fast)
    /// This is the primary way the smart intersection controls vehicle velocity
    pub fn set_velocity_level(&mut self, level: VelocityLevel) {
        self.velocity_level = level;
        self.velocity = level.to_pixels_per_frame();
    }

    /// Legacy method: Set velocity directly (will be converted to nearest velocity level)
    pub fn set_velocity(&mut self, vel: f32) {
        // Convert absolute velocity to nearest velocity level
        let slow_speed = VelocityLevel::Slow.to_pixels_per_frame();
        let normal_speed = VelocityLevel::Normal.to_pixels_per_frame();
        let fast_speed = VelocityLevel::Fast.to_pixels_per_frame();

        let clamped_vel = vel.max(10.0).min(200.0);
        
        // Determine closest velocity level
        let level = if (clamped_vel - slow_speed).abs() < (clamped_vel - normal_speed).abs() {
            if (clamped_vel - slow_speed).abs() < (clamped_vel - fast_speed).abs() {
                VelocityLevel::Slow
            } else {
                VelocityLevel::Fast
            }
        } else {
            if (clamped_vel - normal_speed).abs() < (clamped_vel - fast_speed).abs() {
                VelocityLevel::Normal
            } else {
                VelocityLevel::Fast
            }
        };
        
        self.set_velocity_level(level);
    }

    /// Legacy method: Reduce velocity by a factor (deprecated in favor of set_velocity_level)
    pub fn reduce_velocity(&mut self, factor: f32) {
        // Apply factor to current velocity level
        let new_velocity = self.velocity * factor;
        self.set_velocity(new_velocity);
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

    pub fn get_animation_angle(&self) -> f32 {
        self.animation.get_angle()
    }

    pub fn is_turning(&self) -> bool {
        self.animation.is_turning
    }

    pub fn get_assigned_lane(&self) -> u32 {
        self.assigned_lane
    }

    pub fn can_change_route(&self) -> bool {
        // Vehicles cannot change their route once assigned
        false
    }

    pub fn should_apply_route_turn(&self, intersection_center: (f32, f32), threshold: f32) -> bool {
        if self.route_applied {
            return false;
        }

        let dx = self.position.0 - intersection_center.0;
        let dy = self.position.1 - intersection_center.1;
        let distance = (dx * dx + dy * dy).sqrt();

        distance < threshold
    }

    pub fn apply_route_turn(&mut self) {
        if self.route_applied {
            return;
        }

        self.route_applied = true;

        self.direction = match (self.direction, self.route) {
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
        };
    }

    // ==================== Physics Tracking Methods ====================

    /// Mark the moment a vehicle enters the intersection
    pub fn mark_intersection_entry(&mut self, current_time: f32) {
        if !self.entered_intersection {
            self.entered_intersection = true;
            self.intersection_entry_time = current_time;
            self.intersection_entry_position = self.position;
        }
    }

    /// Mark the moment a vehicle exits the intersection
    pub fn mark_intersection_exit(&mut self, current_time: f32) {
        if self.entered_intersection && !self.exited_intersection {
            self.intersection_exit_time = current_time;
            self.exited_intersection = true;
            self.update_intersection_physics();
        }
    }

    /// Update physics calculations for the intersection traversal
    fn update_intersection_physics(&mut self) {
        // Calculate distance traveled in intersection
        let dx = self.position.0 - self.intersection_entry_position.0;
        let dy = self.position.1 - self.intersection_entry_position.1;
        self.intersection_distance = (dx * dx + dy * dy).sqrt();

        // Calculate time spent in intersection
        let time_in_intersection = self.intersection_exit_time - self.intersection_entry_time;

        // Calculate velocity using physics: velocity = distance / time
        if time_in_intersection > 0.0 {
            self.intersection_physics_velocity = self.intersection_distance / time_in_intersection;
        } else {
            self.intersection_physics_velocity = 0.0;
        }
    }

    /// Get the time spent in the intersection (seconds)
    pub fn get_intersection_time(&self) -> f32 {
        if self.entered_intersection && self.intersection_exit_time > 0.0 {
            self.intersection_exit_time - self.intersection_entry_time
        } else if self.entered_intersection {
            self.time_in_intersection
        } else {
            0.0
        }
    }

    /// Get the distance traveled in the intersection (pixels)
    pub fn get_intersection_distance(&self) -> f32 {
        self.intersection_distance
    }

    /// Get the calculated physics velocity (distance / time) in pixels per second
    pub fn get_physics_velocity(&self) -> f32 {
        self.intersection_physics_velocity
    }

    /// Check if vehicle has entered intersection
    pub fn has_entered_intersection(&self) -> bool {
        self.entered_intersection
    }

    /// Check if vehicle has exited intersection (has complete physics data)
    pub fn has_exited_intersection(&self) -> bool {
        self.entered_intersection && self.intersection_exit_time > 0.0
    }

    /// Get a summary of physics data for the vehicle
    pub fn get_physics_summary(&self) -> PhysicsSummary {
        PhysicsSummary {
            vehicle_id: self.id,
            time_in_intersection: self.get_intersection_time(),
            distance_in_intersection: self.intersection_distance,
            velocity_pixels_per_second: self.intersection_physics_velocity,
            velocity_pixels_per_frame: self.velocity,
            average_velocity_level: match self.velocity_level {
                VelocityLevel::Slow => "Slow (40 px/f)",
                VelocityLevel::Normal => "Normal (100 px/f)",
                VelocityLevel::Fast => "Fast (160 px/f)",
            },
            distance_traveled_total: self.distance_traveled,
        }
    }

    #[cfg(test)]
    pub fn set_position(&mut self, x: f32, y: f32) {
        self.position = (x, y);
    }
}

/// Summary of vehicle physics data through intersection
#[derive(Clone, Debug)]
pub struct PhysicsSummary {
    pub vehicle_id: u32,
    pub time_in_intersection: f32,         // seconds
    pub distance_in_intersection: f32,     // pixels
    pub velocity_pixels_per_second: f32,   // calculated: distance / time
    pub velocity_pixels_per_frame: f32,    // current velocity
    pub average_velocity_level: &'static str,
    pub distance_traveled_total: f32,      // total distance from start
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::intersection::Direction;

    #[test]
    fn set_position_changes_vehicle_position() {
        let mut v = Vehicle::new(0, Direction::North, Route::Straight);
        v.set_position(100.0, 200.0);
        assert_eq!(v.get_position(), (100.0, 200.0));
    }
}
