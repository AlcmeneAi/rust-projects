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

pub struct Lane {
    pub id: usize,
    pub direction: Direction,
}

pub struct Intersection {
    pub center: (f32, f32),
    pub size: f32,
    pub lanes: Vec<Lane>,
}

impl Intersection {
    pub fn new(center_x: f32, center_y: f32, size: f32) -> Self {
        let lanes = vec![
            Lane { id: 0, direction: Direction::North },
            Lane { id: 1, direction: Direction::South },
            Lane { id: 2, direction: Direction::East },
            Lane { id: 3, direction: Direction::West },
        ];

        Intersection {
            center: (center_x, center_y),
            size,
            lanes,
        }
    }

    pub fn contains_point(&self, point: (f32, f32)) -> bool {
        let dx = (point.0 - self.center.0).abs();
        let dy = (point.1 - self.center.1).abs();
        dx < self.size && dy < self.size
    }

    pub fn get_distance_to_center(&self, point: (f32, f32)) -> f32 {
        let dx = point.0 - self.center.0;
        let dy = point.1 - self.center.1;
        (dx * dx + dy * dy).sqrt()
    }
}
