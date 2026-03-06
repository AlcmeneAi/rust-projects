use minifb::Window;
use crate::vehicle::Vehicle;
use crate::intersection::Intersection;

pub struct Renderer {
    buffer: Vec<u32>,
    width: u32,
    height: u32,
}

impl Renderer {
    pub fn new(width: u32, height: u32) -> Result<Self, Box<dyn std::error::Error>> {
        let buffer = vec![0; (width * height) as usize];
        Ok(Renderer {
            buffer,
            width,
            height,
        })
    }

    pub fn clear(&mut self) {
        // Light gray background
        let bg_color = Self::rgb(240, 240, 240);
        for pixel in self.buffer.iter_mut() {
            *pixel = bg_color;
        }
    }

    pub fn present(&self, window: &Window) -> Result<(), Box<dyn std::error::Error>> {
        window.update_with_buffer(&self.buffer, self.width as usize, self.height as usize)?;
        Ok(())
    }

    pub fn draw_intersection(&mut self, intersection: &Intersection) -> Result<(), Box<dyn std::error::Error>> {
        // Draw intersection as gray square
        let size = (intersection.size * 2.0) as u32;
        let x = (intersection.center.0 - intersection.size) as i32;
        let y = (intersection.center.1 - intersection.size) as i32;

        let gray = Self::rgb(200, 200, 200);
        self.fill_rect(x, y, size, size, gray);

        // Draw lane markings
        let white = Self::rgb(255, 255, 255);
        let lane_width = 40;

        // Horizontal lanes
        for i in 0..3 {
            let lane_y = (y + (i as f32 * lane_width as f32) as i32) as u32;
            self.draw_rect(x as u32, lane_y, size, lane_width, white);
        }

        // Vertical lanes
        for i in 0..3 {
            let lane_x = (x + (i as f32 * lane_width as f32) as i32) as u32;
            self.draw_rect(lane_x, y as u32, lane_width, size, white);
        }

        // Draw roads
        let dark = Self::rgb(50, 50, 50);

        // North road
        let north_width = (intersection.size) as u32;
        let north_height = (intersection.center.1 - intersection.size) as u32;
        self.fill_rect((intersection.center.0 - intersection.size / 2.0) as i32, 0, north_width, north_height, dark);

        // South road
        let south_y = (intersection.center.1 + intersection.size) as i32;
        let south_height = (self.height as f32 - intersection.center.1 - intersection.size) as u32;
        self.fill_rect((intersection.center.0 - intersection.size / 2.0) as i32, south_y, north_width, south_height, dark);

        // East road
        let east_x = (intersection.center.0 + intersection.size) as i32;
        let east_width = (self.width as f32 - intersection.center.0 - intersection.size) as u32;
        self.fill_rect(east_x, (intersection.center.1 - intersection.size / 2.0) as i32, east_width, (intersection.size) as u32, dark);

        // West road
        let west_width = (intersection.center.0 - intersection.size) as u32;
        self.fill_rect(0, (intersection.center.1 - intersection.size / 2.0) as i32, west_width, (intersection.size) as u32, dark);

        Ok(())
    }

    pub fn draw_vehicle(&mut self, vehicle: &Vehicle) -> Result<(), Box<dyn std::error::Error>> {
        let pos = vehicle.get_position();
        let x = pos.0 as i32;
        let y = pos.1 as i32;
        let size = 20u32;

        let color = match vehicle.get_id() % 5 {
            0 => Self::rgb(255, 0, 0),    // Red
            1 => Self::rgb(0, 0, 255),    // Blue
            2 => Self::rgb(0, 255, 0),    // Green
            3 => Self::rgb(255, 255, 0),  // Yellow
            _ => Self::rgb(255, 0, 255),  // Magenta
        };

        self.fill_rect(x - (size / 2) as i32, y - (size / 2) as i32, size, size, color);

        Ok(())
    }

    // Helper functions
    fn rgb(r: u32, g: u32, b: u32) -> u32 {
        (r << 16) | (g << 8) | b
    }

    fn fill_rect(&mut self, x: i32, y: i32, width: u32, height: u32, color: u32) {
        for dy in 0..height {
            for dx in 0..width {
                let px = x + dx as i32;
                let py = y + dy as i32;
                if px >= 0 && px < self.width as i32 && py >= 0 && py < self.height as i32 {
                    let idx = ((py as u32) * self.width + (px as u32)) as usize;
                    if idx < self.buffer.len() {
                        self.buffer[idx] = color;
                    }
                }
            }
        }
    }

    fn draw_rect(&mut self, x: u32, y: u32, width: u32, height: u32, color: u32) {
        // Draw outline
        for i in 0..width {
            let idx_top = (y * self.width + x + i) as usize;
            let idx_bottom = ((y + height - 1) * self.width + x + i) as usize;
            if idx_top < self.buffer.len() {
                self.buffer[idx_top] = color;
            }
            if idx_bottom < self.buffer.len() {
                self.buffer[idx_bottom] = color;
            }
        }
        for i in 0..height {
            let idx_left = ((y + i) * self.width + x) as usize;
            let idx_right = ((y + i) * self.width + x + width - 1) as usize;
            if idx_left < self.buffer.len() {
                self.buffer[idx_left] = color;
            }
            if idx_right < self.buffer.len() {
                self.buffer[idx_right] = color;
            }
        }
    }
}
