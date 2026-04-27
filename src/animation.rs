use crate::intersection::Direction;

/// Represents the animation state of a vehicle
#[derive(Clone, Debug)]
pub struct AnimationState {
    /// Current rotation angle in degrees (0-360)
    pub current_angle: f32,
    /// Target rotation angle in degrees
    pub target_angle: f32,
    /// Speed of rotation animation in degrees per second
    pub rotation_speed: f32,
    /// Whether the vehicle is currently turning
    pub is_turning: bool,
}

impl AnimationState {
    /// Create a new animation state for a vehicle starting in a given direction
    pub fn new(direction: Direction) -> Self {
        let angle = direction_to_angle(direction);
        AnimationState {
            current_angle: angle,
            target_angle: angle,
            rotation_speed: 360.0, // Full rotation in 1 second
            is_turning: false,
        }
    }

    /// Update the animation state based on delta time
    pub fn update(&mut self, dt: f32) {
        if (self.current_angle - self.target_angle).abs() > 0.1 {
            // Calculate the shortest rotation path
            let mut angle_diff = self.target_angle - self.current_angle;
            
            // Normalize angle difference to [-180, 180]
            while angle_diff > 180.0 {
                angle_diff -= 360.0;
            }
            while angle_diff < -180.0 {
                angle_diff += 360.0;
            }

            let max_rotation = self.rotation_speed * dt;
            if angle_diff.abs() <= max_rotation {
                self.current_angle = self.target_angle;
                self.is_turning = false;
            } else {
                self.current_angle += angle_diff.signum() * max_rotation;
            }

            // Keep angle in [0, 360) range
            self.current_angle = self.current_angle.rem_euclid(360.0);
        }
    }

    /// Set a new target direction for the vehicle
    pub fn set_direction(&mut self, direction: Direction) {
        let new_angle = direction_to_angle(direction);
        if (self.target_angle - new_angle).abs() > 0.1 {
            self.target_angle = new_angle;
            self.is_turning = true;
        }
    }

    /// Get the current rotation angle
    pub fn get_angle(&self) -> f32 {
        self.current_angle
    }
}

/// Convert a Direction to a rotation angle in degrees
/// North = 0°, East = 90°, South = 180°, West = 270°
fn direction_to_angle(direction: Direction) -> f32 {
    match direction {
        Direction::North => 0.0,
        Direction::East => 90.0,
        Direction::South => 180.0,
        Direction::West => 270.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::Route;

    fn angle_to_direction(angle: f32) -> Direction {
        let normalized = angle.rem_euclid(360.0);
        match normalized as i32 {
            0..=45 | 315..=360 => Direction::North,
            46..=135 => Direction::East,
            136..=225 => Direction::South,
            226..=314 => Direction::West,
            _ => Direction::North,
        }
    }

    fn apply_route(direction: Direction, route: Route) -> Direction {
        match route {
            Route::Straight => direction,
            Route::Right => match direction {
                Direction::North => Direction::East,
                Direction::East => Direction::South,
                Direction::South => Direction::West,
                Direction::West => Direction::North,
            },
            Route::Left => match direction {
                Direction::North => Direction::West,
                Direction::West => Direction::South,
                Direction::South => Direction::East,
                Direction::East => Direction::North,
            },
        }
    }

    #[test]
    fn test_direction_to_angle() {
        assert_eq!(direction_to_angle(Direction::North), 0.0);
        assert_eq!(direction_to_angle(Direction::East), 90.0);
        assert_eq!(direction_to_angle(Direction::South), 180.0);
        assert_eq!(direction_to_angle(Direction::West), 270.0);
    }

    #[test]
    fn test_angle_to_direction() {
        assert_eq!(angle_to_direction(0.0), Direction::North);
        assert_eq!(angle_to_direction(90.0), Direction::East);
        assert_eq!(angle_to_direction(180.0), Direction::South);
        assert_eq!(angle_to_direction(270.0), Direction::West);
    }

    #[test]
    fn test_apply_route() {
        assert_eq!(apply_route(Direction::North, Route::Straight), Direction::North);
        assert_eq!(apply_route(Direction::North, Route::Right), Direction::East);
        assert_eq!(apply_route(Direction::North, Route::Left), Direction::West);
    }
}
