/// Physics module for vehicle calculations
/// Implements: velocity = distance / time

pub struct Physics;

impl Physics {
    /// Calculate velocity given distance and time
    pub fn calculate_velocity(distance: f32, time: f32) -> f32 {
        if time > 0.0 {
            distance / time
        } else {
            0.0
        }
    }

    /// Calculate distance given velocity and time
    pub fn calculate_distance(velocity: f32, time: f32) -> f32 {
        velocity * time
    }

    /// Calculate time given distance and velocity
    pub fn calculate_time(distance: f32, velocity: f32) -> f32 {
        if velocity > 0.0 {
            distance / velocity
        } else {
            f32::INFINITY
        }
    }

    /// Calculate stopping distance given initial velocity and deceleration
    pub fn calculate_stopping_distance(velocity: f32, deceleration: f32) -> f32 {
        if deceleration > 0.0 {
            (velocity * velocity) / (2.0 * deceleration)
        } else {
            f32::INFINITY
        }
    }

    /// Check if a vehicle needs to brake to avoid collision
    pub fn needs_braking(
        current_velocity: f32,
        distance_to_obstacle: f32,
        deceleration: f32,
    ) -> bool {
        let stopping_distance = Self::calculate_stopping_distance(current_velocity, deceleration);
        stopping_distance > distance_to_obstacle
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_velocity_calculation() {
        let velocity = Physics::calculate_velocity(100.0, 2.0);
        assert_eq!(velocity, 50.0);
    }

    #[test]
    fn test_distance_calculation() {
        let distance = Physics::calculate_distance(50.0, 2.0);
        assert_eq!(distance, 100.0);
    }

    #[test]
    fn test_time_calculation() {
        let time = Physics::calculate_time(100.0, 50.0);
        assert_eq!(time, 2.0);
    }
}
