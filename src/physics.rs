pub struct Physics;

impl Physics {
    pub fn calculate_velocity(distance: f32, time: f32) -> f32 {
        if time > 0.0 {
            distance / time
        } else {
            0.0
        }
    }

    pub fn calculate_distance(velocity: f32, time: f32) -> f32 {
        velocity * time
    }

    pub fn calculate_time(distance: f32, velocity: f32) -> f32 {
        if velocity > 0.0 {
            distance / velocity
        } else {
            f32::INFINITY
        }
    }

    pub fn calculate_stopping_distance(velocity: f32, deceleration: f32) -> f32 {
        if deceleration > 0.0 {
            (velocity * velocity) / (2.0 * deceleration)
        } else {
            f32::INFINITY
        }
    }

    pub fn needs_braking(
        current_velocity: f32,
        distance_to_obstacle: f32,
        deceleration: f32,
    ) -> bool {
        let stopping_distance = Self::calculate_stopping_distance(current_velocity, deceleration);
        stopping_distance > distance_to_obstacle
    }
}
