use crate::vehicle::Vehicle;

pub struct Statistics {
    vehicle_count: u32,
    max_velocity: f32,
    min_velocity: f32,
    max_time: f32,
    min_time: f32,
    close_calls: u32,
    first_vehicle: bool,
}

impl Statistics {
    pub fn new() -> Self {
        Statistics {
            vehicle_count: 0,
            max_velocity: 0.0,
            min_velocity: f32::MAX,
            max_time: 0.0,
            min_time: f32::MAX,
            close_calls: 0,
            first_vehicle: true,
        }
    }

    pub fn record_vehicle(&mut self, vehicle: &Vehicle) {
        self.vehicle_count += 1;
        let velocity = vehicle.get_velocity();
        if velocity > self.max_velocity {
            self.max_velocity = velocity;
        }
        if velocity < self.min_velocity {
            self.min_velocity = velocity;
        }
        let time = vehicle.get_time_in_intersection();
        if time > self.max_time {
            self.max_time = time;
        }
        if time < self.min_time || self.first_vehicle {
            self.min_time = time;
        }
        self.first_vehicle = false;
    }

    pub fn record_close_call(&mut self) {
        self.close_calls += 1;
    }

    pub fn get_vehicle_count(&self) -> u32 {
        self.vehicle_count
    }

    pub fn get_max_velocity(&self) -> f32 {
        self.max_velocity
    }

    pub fn get_min_velocity(&self) -> f32 {
        if self.vehicle_count == 0 {
            0.0
        } else {
            self.min_velocity
        }
    }

    pub fn get_max_time(&self) -> f32 {
        self.max_time
    }

    pub fn get_min_time(&self) -> f32 {
        if self.vehicle_count == 0 {
            0.0
        } else {
            self.min_time
        }
    }

    pub fn get_close_calls(&self) -> u32 {
        self.close_calls
    }
}
