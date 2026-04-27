use rand::Rng;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Direction {
    North,
    South,
    East,
    West,
}

impl Direction {
    pub fn random() -> Self {
        let mut rng = rand::thread_rng();
        match rng.gen_range(0..4) {
            0 => Direction::North,
            1 => Direction::South,
            2 => Direction::East,
            _ => Direction::West,
        }
    }
}

pub struct Intersection {
    pub center: (f32, f32),
    pub size: f32,
}

impl Intersection {
    pub fn new(center_x: f32, center_y: f32, size: f32) -> Self {
        Intersection {
            center: (center_x, center_y),
            size,
        }
    }

    pub fn contains_point(&self, point: (f32, f32)) -> bool {
        let dx = (point.0 - self.center.0).abs();
        let dy = (point.1 - self.center.1).abs();
        dx < self.size && dy < self.size
    }

    /// True when the point is inside or within `padding` pixels of the intersection boundary.
    /// Used to detect when the smart intersection algorithm first notices a vehicle.
    pub fn is_near_or_in(&self, point: (f32, f32), padding: f32) -> bool {
        let dx = (point.0 - self.center.0).abs();
        let dy = (point.1 - self.center.1).abs();
        dx < self.size + padding && dy < self.size + padding
    }

}
