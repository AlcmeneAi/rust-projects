use crate::vehicle::Vehicle;
use nalgebra::DVector;
use std::collections::HashSet;

pub struct Statistics {
    velocities: Vec<f32>,
    times: Vec<f32>,
    close_calls: u32,
    active_close_call_pairs: HashSet<(u32, u32)>,
    collisions: u32,
    active_collision_pairs: HashSet<(u32, u32)>,
}

impl Statistics {
    pub fn new() -> Self {
        Statistics {
            velocities: Vec::new(),
            times: Vec::new(),
            close_calls: 0,
            active_close_call_pairs: HashSet::new(),
            collisions: 0,
            active_collision_pairs: HashSet::new(),
        }
    }

    /// Records a single observation. Used by record_vehicle and by tests.
    pub fn record_sample(&mut self, velocity: f32, time_in_intersection: f32) {
        self.velocities.push(velocity);
        self.times.push(time_in_intersection);
    }

    pub fn record_vehicle(&mut self, vehicle: &Vehicle) {
        self.record_sample(
            vehicle.get_physics_velocity(),
            vehicle.get_time_in_intersection(),
        );
    }

    pub fn is_active_close_call(&self, pair: &(u32, u32)) -> bool {
        self.active_close_call_pairs.contains(pair)
    }

    pub fn update_close_calls(&mut self, current: &HashSet<(u32, u32)>) {
        for &pair in current {
            if !self.active_close_call_pairs.contains(&pair) {
                self.close_calls += 1;
            }
        }
        self.active_close_call_pairs = current.clone();
    }

    fn velocities_vec(&self) -> DVector<f32> {
        DVector::from_row_slice(&self.velocities)
    }

    fn times_vec(&self) -> DVector<f32> {
        DVector::from_row_slice(&self.times)
    }

    pub fn get_vehicle_count(&self) -> u32 {
        self.velocities.len() as u32
    }

    pub fn get_max_velocity(&self) -> f32 {
        if self.velocities.is_empty() { 0.0 } else { self.velocities_vec().max() }
    }

    pub fn get_min_velocity(&self) -> f32 {
        if self.velocities.is_empty() { 0.0 } else { self.velocities_vec().min() }
    }

    pub fn get_mean_velocity(&self) -> f32 {
        if self.velocities.is_empty() { 0.0 } else { self.velocities_vec().mean() }
    }

    pub fn get_stddev_velocity(&self) -> f32 {
        if self.velocities.is_empty() { 0.0 } else { self.velocities_vec().variance().sqrt() }
    }

    pub fn get_max_time(&self) -> f32 {
        if self.times.is_empty() { 0.0 } else { self.times_vec().max() }
    }

    pub fn get_min_time(&self) -> f32 {
        if self.times.is_empty() { 0.0 } else { self.times_vec().min() }
    }

    pub fn get_mean_time(&self) -> f32 {
        if self.times.is_empty() { 0.0 } else { self.times_vec().mean() }
    }

    pub fn get_stddev_time(&self) -> f32 {
        if self.times.is_empty() { 0.0 } else { self.times_vec().variance().sqrt() }
    }

    pub fn get_close_calls(&self) -> u32 {
        self.close_calls
    }

    pub fn is_active_collision(&self, pair: &(u32, u32)) -> bool {
        self.active_collision_pairs.contains(pair)
    }

    pub fn update_collisions(&mut self, current: &HashSet<(u32, u32)>) {
        for &pair in current {
            if !self.active_collision_pairs.contains(&pair) {
                self.collisions += 1;
            }
        }
        self.active_collision_pairs = current.clone();
    }

    pub fn get_collisions(&self) -> u32 {
        self.collisions
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32) -> bool { (a - b).abs() < 1e-4 }

    #[test]
    fn empty_statistics_returns_zeros() {
        let s = Statistics::new();
        assert_eq!(s.get_vehicle_count(), 0);
        assert_eq!(s.get_max_velocity(), 0.0);
        assert_eq!(s.get_min_velocity(), 0.0);
        assert_eq!(s.get_mean_velocity(), 0.0);
        assert_eq!(s.get_stddev_velocity(), 0.0);
        assert_eq!(s.get_max_time(), 0.0);
        assert_eq!(s.get_min_time(), 0.0);
        assert_eq!(s.get_close_calls(), 0);
    }

    #[test]
    fn single_sample_min_equals_max_equals_mean() {
        let mut s = Statistics::new();
        s.record_sample(42.0, 1.5);
        assert_eq!(s.get_vehicle_count(), 1);
        assert!(approx(s.get_max_velocity(), 42.0));
        assert!(approx(s.get_min_velocity(), 42.0));
        assert!(approx(s.get_mean_velocity(), 42.0));
        assert!(approx(s.get_max_time(), 1.5));
        assert!(approx(s.get_min_time(), 1.5));
        assert!(approx(s.get_mean_time(), 1.5));
        assert!(approx(s.get_stddev_velocity(), 0.0));
    }

    #[test]
    fn multiple_samples_aggregate_correctly() {
        let mut s = Statistics::new();
        s.record_sample(10.0, 1.0);
        s.record_sample(20.0, 2.0);
        s.record_sample(30.0, 3.0);
        assert_eq!(s.get_vehicle_count(), 3);
        assert!(approx(s.get_max_velocity(), 30.0));
        assert!(approx(s.get_min_velocity(), 10.0));
        assert!(approx(s.get_mean_velocity(), 20.0));
        assert!(approx(s.get_max_time(), 3.0));
        assert!(approx(s.get_min_time(), 1.0));
        assert!(approx(s.get_mean_time(), 2.0));
        // population stddev of (10,20,30): sqrt(((-10)^2 + 0 + 10^2)/3) = sqrt(200/3)
        assert!(approx(s.get_stddev_velocity(), (200.0_f32 / 3.0).sqrt()));
    }

    #[test]
    fn close_calls_dedup_per_pair() {
        let mut s = Statistics::new();
        let mut frame_a = HashSet::new();
        frame_a.insert((1u32, 2u32));
        s.update_close_calls(&frame_a);
        // Same pair on the next frame must not increment.
        s.update_close_calls(&frame_a);
        assert_eq!(s.get_close_calls(), 1);

        // New pair appearing -> increment by one more.
        let mut frame_b = HashSet::new();
        frame_b.insert((1u32, 2u32));
        frame_b.insert((3u32, 4u32));
        s.update_close_calls(&frame_b);
        assert_eq!(s.get_close_calls(), 2);
    }
}
