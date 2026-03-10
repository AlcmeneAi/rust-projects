use sdl2::render::{Canvas, TextureCreator};
use sdl2::video::Window;
use sdl2::rect::Rect;
use sdl2::pixels::Color;
use crate::vehicle::Vehicle;
use crate::intersection::Intersection;
use crate::intersection::Direction;
use std::collections::HashMap;

pub struct Renderer {
    canvas: Canvas<Window>,
    width: u32,
    height: u32,
    // Vehicle sprite cache (color_id -> texture)
    vehicle_sprites: HashMap<u32, (u32, u32)>, // Store sprite dimensions
}

const LANE_WIDTH: i32 = 50;
const LANE_COUNT: usize = 3; // 3 lanes per direction (left, straight, right)
const DASHED_LINE_LENGTH: i32 = 10;
const DASHED_LINE_GAP: i32 = 10;

impl Renderer {
    pub fn new(
        canvas: Canvas<Window>,
        _texture_creator: &TextureCreator<sdl2::video::WindowContext>,
        width: u32,
        height: u32,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Renderer {
            canvas,
            width,
            height,
            vehicle_sprites: HashMap::new(),
        })
    }

    pub fn clear(&mut self) {
        self.canvas.set_draw_color(Color::RGB(34, 139, 34)); // Dark green grass
        self.canvas.clear();
    }

    pub fn present(&mut self) {
        self.canvas.present();
    }

    pub fn draw_intersection(&mut self, intersection: &Intersection) -> Result<(), Box<dyn std::error::Error>> {
        let center_x = intersection.center.0 as i32;
        let center_y = intersection.center.1 as i32;
        let size = intersection.size as i32;

        // Draw roads approaching from all directions
        self.draw_road_north(center_x, center_y, size)?;
        self.draw_road_south(center_x, center_y, size)?;
        self.draw_road_east(center_x, center_y, size)?;
        self.draw_road_west(center_x, center_y, size)?;

        // Draw intersection center
        let intersection_rect = Rect::new(
            center_x - size,
            center_y - size,
            (size * 2) as u32,
            (size * 2) as u32,
        );
        self.canvas.set_draw_color(Color::RGB(100, 100, 100));
        self.canvas.fill_rect(intersection_rect)?;

        // Draw lane markings in intersection
        self.draw_intersection_lane_markings(center_x, center_y, size)?;

        Ok(())
    }

    fn draw_road_north(&mut self, center_x: i32, center_y: i32, size: i32) -> Result<(), Box<dyn std::error::Error>> {
        let road_width = (LANE_WIDTH * LANE_COUNT as i32) as u32;
        let road_start_x = center_x - (road_width as i32 / 2);
        let road_end_y = center_y - size;

        // Draw road surface
        self.canvas.set_draw_color(Color::RGB(50, 50, 50));
        let road_rect = Rect::new(road_start_x, 0, road_width, (center_y - size) as u32);
        self.canvas.fill_rect(road_rect)?;

        // Draw lanes with markings
        self.draw_lanes_vertical(road_start_x, 0, road_width as i32, center_y - size, true)?;

        // Draw lane labels (Left, Straight, Right)
        let label_y = center_y - size - 40;
        self.draw_text_simple(road_start_x + LANE_WIDTH / 2 - 5, label_y, "l")?;
        self.draw_text_simple(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH - 5, label_y, "s")?;
        self.draw_text_simple(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH * 2 - 5, label_y, "r")?;

        // Draw direction arrows
        self.draw_arrow_down(road_start_x + LANE_WIDTH / 2, label_y + 15)?;
        self.draw_arrow_down(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH, label_y + 15)?;
        self.draw_arrow_down(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH * 2, label_y + 15)?;

        Ok(())
    }

    fn draw_road_south(&mut self, center_x: i32, center_y: i32, size: i32) -> Result<(), Box<dyn std::error::Error>> {
        let road_width = (LANE_WIDTH * LANE_COUNT as i32) as u32;
        let road_start_x = center_x - (road_width as i32 / 2);
        let road_start_y = center_y + size;

        // Draw road surface
        self.canvas.set_draw_color(Color::RGB(50, 50, 50));
        let road_rect = Rect::new(road_start_x, road_start_y, road_width, (self.height as i32 - road_start_y) as u32);
        self.canvas.fill_rect(road_rect)?;

        // Draw lanes with markings
        self.draw_lanes_vertical(road_start_x, road_start_y, road_width as i32, self.height as i32 - road_start_y, true)?;

        // Draw lane labels (but reversed for southbound: r, s, l from left to right visually)
        let label_y = road_start_y + 30;
        self.draw_text_simple(road_start_x + LANE_WIDTH / 2 - 5, label_y, "r")?;
        self.draw_text_simple(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH - 5, label_y, "s")?;
        self.draw_text_simple(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH * 2 - 5, label_y, "l")?;

        // Draw direction arrows
        self.draw_arrow_up(road_start_x + LANE_WIDTH / 2, label_y - 15)?;
        self.draw_arrow_up(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH, label_y - 15)?;
        self.draw_arrow_up(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH * 2, label_y - 15)?;

        Ok(())
    }

    fn draw_road_east(&mut self, center_x: i32, center_y: i32, size: i32) -> Result<(), Box<dyn std::error::Error>> {
        let road_height = (LANE_WIDTH * LANE_COUNT as i32) as u32;
        let road_start_y = center_y - (road_height as i32 / 2);
        let road_start_x = center_x + size;

        // Draw road surface
        self.canvas.set_draw_color(Color::RGB(50, 50, 50));
        let road_rect = Rect::new(road_start_x, road_start_y, (self.width as i32 - road_start_x) as u32, road_height);
        self.canvas.fill_rect(road_rect)?;

        // Draw lanes with markings
        self.draw_lanes_horizontal(road_start_x, road_start_y, self.width as i32 - road_start_x, road_height as i32, true)?;

        // Draw lane labels (l, s, r from top to bottom)
        let label_x = road_start_x + 30;
        self.draw_text_simple(label_x, road_start_y + LANE_WIDTH / 2 - 5, "l")?;
        self.draw_text_simple(label_x, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH - 5, "s")?;
        self.draw_text_simple(label_x, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH * 2 - 5, "r")?;

        // Draw direction arrows
        self.draw_arrow_left(label_x - 15, road_start_y + LANE_WIDTH / 2)?;
        self.draw_arrow_left(label_x - 15, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH)?;
        self.draw_arrow_left(label_x - 15, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH * 2)?;

        Ok(())
    }

    fn draw_road_west(&mut self, center_x: i32, center_y: i32, size: i32) -> Result<(), Box<dyn std::error::Error>> {
        let road_height = (LANE_WIDTH * LANE_COUNT as i32) as u32;
        let road_start_y = center_y - (road_height as i32 / 2);
        let road_end_x = center_x - size;

        // Draw road surface
        self.canvas.set_draw_color(Color::RGB(50, 50, 50));
        let road_rect = Rect::new(0, road_start_y, road_end_x as u32, road_height);
        self.canvas.fill_rect(road_rect)?;

        // Draw lanes with markings
        self.draw_lanes_horizontal(0, road_start_y, road_end_x, road_height as i32, true)?;

        // Draw lane labels (r, s, l from top to bottom for westbound)
        let label_x = road_end_x - 30;
        self.draw_text_simple(label_x, road_start_y + LANE_WIDTH / 2 - 5, "r")?;
        self.draw_text_simple(label_x, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH - 5, "s")?;
        self.draw_text_simple(label_x, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH * 2 - 5, "l")?;

        // Draw direction arrows
        self.draw_arrow_right(label_x + 15, road_start_y + LANE_WIDTH / 2)?;
        self.draw_arrow_right(label_x + 15, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH)?;
        self.draw_arrow_right(label_x + 15, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH * 2)?;

        Ok(())
    }

    fn draw_lanes_vertical(&mut self, start_x: i32, start_y: i32, width: i32, height: i32, _with_labels: bool) -> Result<(), Box<dyn std::error::Error>> {
        // Draw vertical lane dividers (dashed white lines)
        self.canvas.set_draw_color(Color::RGB(255, 255, 255));

        for i in 1..LANE_COUNT {
            let line_x = start_x + (i as i32) * LANE_WIDTH;
            self.draw_dashed_line_vertical(line_x, start_y, start_y + height)?;
        }

        Ok(())
    }

    fn draw_lanes_horizontal(&mut self, start_x: i32, start_y: i32, width: i32, height: i32, _with_labels: bool) -> Result<(), Box<dyn std::error::Error>> {
        // Draw horizontal lane dividers (dashed white lines)
        self.canvas.set_draw_color(Color::RGB(255, 255, 255));

        for i in 1..LANE_COUNT {
            let line_y = start_y + (i as i32) * LANE_WIDTH;
            self.draw_dashed_line_horizontal(start_x, line_y, start_x + width)?;
        }

        Ok(())
    }

    fn draw_dashed_line_vertical(&mut self, x: i32, y_start: i32, y_end: i32) -> Result<(), Box<dyn std::error::Error>> {
        let mut y = y_start;
        while y < y_end {
            let rect = Rect::new(x, y, 2, DASHED_LINE_LENGTH as u32);
            self.canvas.fill_rect(rect)?;
            y += DASHED_LINE_LENGTH + DASHED_LINE_GAP;
        }
        Ok(())
    }

    fn draw_dashed_line_horizontal(&mut self, x_start: i32, y: i32, x_end: i32) -> Result<(), Box<dyn std::error::Error>> {
        let mut x = x_start;
        while x < x_end {
            let rect = Rect::new(x, y, DASHED_LINE_LENGTH as u32, 2);
            self.canvas.fill_rect(rect)?;
            x += DASHED_LINE_LENGTH + DASHED_LINE_GAP;
        }
        Ok(())
    }

    fn draw_intersection_lane_markings(&mut self, center_x: i32, center_y: i32, size: i32) -> Result<(), Box<dyn std::error::Error>> {
        // Draw lane markings within the intersection
        self.canvas.set_draw_color(Color::RGB(200, 200, 200));

        // Vertical lane dividers in intersection
        for i in 1..LANE_COUNT {
            let line_x = center_x - (LANE_WIDTH * LANE_COUNT as i32 / 2) + (i as i32) * LANE_WIDTH;
            let rect = Rect::new(line_x, center_y - size, 2, (size * 2) as u32);
            self.canvas.fill_rect(rect)?;
        }

        // Horizontal lane dividers in intersection
        for i in 1..LANE_COUNT {
            let line_y = center_y - (LANE_WIDTH * LANE_COUNT as i32 / 2) + (i as i32) * LANE_WIDTH;
            let rect = Rect::new(center_x - size, line_y, (size * 2) as u32, 2);
            self.canvas.fill_rect(rect)?;
        }

        Ok(())
    }

    fn draw_arrow_down(&mut self, x: i32, y: i32) -> Result<(), Box<dyn std::error::Error>> {
        self.canvas.set_draw_color(Color::RGB(255, 255, 255));
        // Vertical line
        self.draw_line_simple(x, y, x, y + 10)?;
        // Arrow head
        self.draw_line_simple(x, y + 10, x - 3, y + 6)?;
        self.draw_line_simple(x, y + 10, x + 3, y + 6)?;
        Ok(())
    }

    fn draw_arrow_up(&mut self, x: i32, y: i32) -> Result<(), Box<dyn std::error::Error>> {
        self.canvas.set_draw_color(Color::RGB(255, 255, 255));
        // Vertical line
        self.draw_line_simple(x, y, x, y - 10)?;
        // Arrow head
        self.draw_line_simple(x, y - 10, x - 3, y - 6)?;
        self.draw_line_simple(x, y - 10, x + 3, y - 6)?;
        Ok(())
    }

    fn draw_arrow_left(&mut self, x: i32, y: i32) -> Result<(), Box<dyn std::error::Error>> {
        self.canvas.set_draw_color(Color::RGB(255, 255, 255));
        // Horizontal line
        self.draw_line_simple(x, y, x - 10, y)?;
        // Arrow head
        self.draw_line_simple(x - 10, y, x - 6, y - 3)?;
        self.draw_line_simple(x - 10, y, x - 6, y + 3)?;
        Ok(())
    }

    fn draw_arrow_right(&mut self, x: i32, y: i32) -> Result<(), Box<dyn std::error::Error>> {
        self.canvas.set_draw_color(Color::RGB(255, 255, 255));
        // Horizontal line
        self.draw_line_simple(x, y, x + 10, y)?;
        // Arrow head
        self.draw_line_simple(x + 10, y, x + 6, y - 3)?;
        self.draw_line_simple(x + 10, y, x + 6, y + 3)?;
        Ok(())
    }

    fn draw_line_simple(&mut self, x0: i32, y0: i32, x1: i32, y1: i32) -> Result<(), Box<dyn std::error::Error>> {
        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = (dx as i32) - (dy as i32);

        let mut x = x0;
        let mut y = y0;

        loop {
            let rect = Rect::new(x, y, 1, 1);
            let _ = self.canvas.fill_rect(rect);

            if x == x1 && y == y1 {
                break;
            }

            let e2 = 2 * err;
            if e2 > -(dy as i32) {
                err -= dy as i32;
                x += sx;
            }
            if e2 < (dx as i32) {
                err += dx as i32;
                y += sy;
            }
        }

        Ok(())
    }

    fn draw_text_simple(&mut self, x: i32, y: i32, text: &str) -> Result<(), Box<dyn std::error::Error>> {
        // Simple character drawing - just draw a white rect as placeholder
        // In a real implementation, you'd use a font library
        self.canvas.set_draw_color(Color::RGB(255, 255, 0));
        
        match text {
            "l" => {
                self.draw_line_simple(x, y, x, y + 8)?;
                self.draw_line_simple(x, y + 8, x + 4, y + 8)?;
            }
            "s" => {
                self.draw_line_simple(x, y, x + 4, y)?;
                self.draw_line_simple(x + 4, y, x + 4, y + 4)?;
                self.draw_line_simple(x + 4, y + 4, x, y + 4)?;
                self.draw_line_simple(x, y + 4, x, y + 8)?;
                self.draw_line_simple(x, y + 8, x + 4, y + 8)?;
            }
            "r" => {
                self.draw_line_simple(x, y, x, y + 8)?;
                self.draw_line_simple(x, y, x + 4, y)?;
                self.draw_line_simple(x + 4, y, x + 4, y + 4)?;
                self.draw_line_simple(x + 4, y + 4, x, y + 4)?;
                self.draw_line_simple(x + 4, y + 4, x + 4, y + 8)?;
            }
            _ => {}
        }

        Ok(())
    }

    pub fn draw_vehicle(&mut self, vehicle: &Vehicle) -> Result<(), Box<dyn std::error::Error>> {
        let pos = vehicle.get_position();
        let x = pos.0;
        let y = pos.1;
        let width = 30.0;
        let height = 15.0;
        let angle = vehicle.get_animation_angle();

        let color = match vehicle.get_id() % 5 {
            0 => Color::RGB(255, 0, 0),
            1 => Color::RGB(0, 0, 255),
            2 => Color::RGB(0, 255, 0),
            3 => Color::RGB(255, 255, 0),
            _ => Color::RGB(255, 0, 255),
        };

        // Draw a rotated rectangle representing the vehicle
        self.draw_rotated_vehicle(x, y, width, height, angle, color)?;

        Ok(())
    }

    /// Draw a vehicle as a rotated rectangle
    fn draw_rotated_vehicle(
        &mut self,
        center_x: f32,
        center_y: f32,
        width: f32,
        height: f32,
        angle_degrees: f32,
        color: Color,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Convert angle to radians
        let angle_rad = angle_degrees.to_radians();
        let cos_a = angle_rad.cos();
        let sin_a = angle_rad.sin();

        // Create the four corners of the rectangle (before rotation)
        let hw = width / 2.0;
        let hh = height / 2.0;

        let corners = [
            (-hw, -hh),
            (hw, -hh),
            (hw, hh),
            (-hw, hh),
        ];

        // Rotate corners and translate to center position
        let rotated_corners: Vec<(i32, i32)> = corners
            .iter()
            .map(|(x, y)| {
                let rot_x = x * cos_a - y * sin_a;
                let rot_y = x * sin_a + y * cos_a;
                ((center_x + rot_x) as i32, (center_y + rot_y) as i32)
            })
            .collect();

        self.canvas.set_draw_color(color);

        // Draw the outline of the rotated rectangle
        for i in 0..4 {
            let p1 = rotated_corners[i];
            let p2 = rotated_corners[(i + 1) % 4];
            self.draw_line_simple(p1.0, p1.1, p2.0, p2.1)?;
        }

        // Fill the rectangle using multiple horizontal lines
        self.fill_rotated_rect_scanline(&rotated_corners, color)?;

        // Draw a directional indicator (white triangle at the front)
        let front_x = center_x + (width / 2.0 + 5.0) * cos_a;
        let front_y = center_y + (width / 2.0 + 5.0) * sin_a;
        
        let indicator_size = 5.0;
        let ind1_x = (front_x + indicator_size * cos_a) as i32;
        let ind1_y = (front_y + indicator_size * sin_a) as i32;
        let ind2_x = (front_x - indicator_size * 0.7 * sin_a) as i32;
        let ind2_y = (front_y + indicator_size * 0.7 * cos_a) as i32;
        let ind3_x = (front_x + indicator_size * 0.7 * sin_a) as i32;
        let ind3_y = (front_y - indicator_size * 0.7 * cos_a) as i32;

        let indicator_color = Color::RGB(255, 255, 255);
        self.draw_line_simple(ind1_x, ind1_y, ind2_x, ind2_y)?;
        self.draw_line_simple(ind1_x, ind1_y, ind3_x, ind3_y)?;

        Ok(())
    }

    /// Fill a rotated rectangle using scanline algorithm
    fn fill_rotated_rect_scanline(
        &mut self,
        corners: &[(i32, i32)],
        color: Color,
    ) -> Result<(), Box<dyn std::error::Error>> {
        if corners.len() < 4 {
            return Ok(());
        }

        let min_y = *corners.iter().map(|p| &p.1).min().unwrap_or(&0);
        let max_y = *corners.iter().map(|p| &p.1).max().unwrap_or(&0);

        self.canvas.set_draw_color(color);

        for y in min_y..=max_y {
            let mut x_intersections = Vec::new();

            // Find intersections with all edges
            for i in 0..4 {
                let p1 = corners[i];
                let p2 = corners[(i + 1) % 4];

                if (p1.1 <= y && y <= p2.1) || (p2.1 <= y && y <= p1.1) {
                    if p1.1 != p2.1 {
                        let t = (y - p1.1) as f32 / (p2.1 - p1.1) as f32;
                        let x = (p1.0 as f32 + t * (p2.0 - p1.0) as f32) as i32;
                        x_intersections.push(x);
                    }
                }
            }

            // Sort intersections and fill between pairs
            x_intersections.sort();
            for i in (0..x_intersections.len()).step_by(2) {
                if i + 1 < x_intersections.len() {
                    let x_start = x_intersections[i];
                    let x_end = x_intersections[i + 1];
                    for x in x_start..=x_end {
                        let rect = Rect::new(x, y, 1, 1);
                        let _ = self.canvas.fill_rect(rect);
                    }
                }
            }
        }

        Ok(())
    }
}

