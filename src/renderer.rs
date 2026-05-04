use sdl2::render::{BlendMode, Canvas, Texture, TextureCreator};
use sdl2::video::Window;
use sdl2::rect::Rect;
use sdl2::pixels::Color;
use nalgebra::Vector2;
use crate::vehicle::{Vehicle, VelocityLevel, Route};
use crate::intersection::{Intersection, Direction};
use crate::statistics::Statistics;
use crate::glyphs::{glyph_for, text_width, Glyph, GLYPH_W, GLYPH_H, GLYPH_SPACING};
use crate::summary::{window_origin, ok_button_rect, restart_button_rect,
                     point_in_ok_button, point_in_restart_button, WIN_W, WIN_H};
use crate::collision::{vehicle_time_window, predict_atw_conflict};

/// Holds a single north-facing (up) base sprite for car rendering.
/// The sprite is rotated by the vehicle's current animation angle to produce
/// smooth turning visuals in all four cardinal directions and between them.
pub struct CarTextures<'a> {
    /// `car_up.png` — north-facing sprite used as the rotation base (0° = up).
    pub base: Texture<'a>,
}

pub struct Renderer {
    canvas: Canvas<Window>,
    width: u32,
    height: u32,
}

const LANE_WIDTH: i32 = 50;
const LANE_COUNT: usize = 6; // 6 lanes total: 3 outgoing + 3 incoming per road
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
        })
    }

    pub fn clear(&mut self) {
        self.canvas.set_draw_color(Color::RGB(34, 139, 34)); // Dark green grass
        self.canvas.clear();
    }

    pub fn present(&mut self) {
        self.canvas.present();
    }

    /// Draw a small always-on HUD in the top-left corner.
    ///
    /// Shows:
    ///   - "RAND: ON" (green) or "RAND: OFF" (grey) — random vehicle generation state
    ///   - The current live vehicle count
    ///   - A controls reminder (R = toggle random, D = debug, arrows = manual)
    pub fn draw_hud(
        &mut self,
        random_enabled: bool,
        vehicle_count: usize,
        debug_mode: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let scale = 1;
        let lh = (GLYPH_H + 3) * scale; // line height
        let x0 = 10;
        let mut y = 10;

        // Background pill so the text stays readable over any road colour
        let bg_w = 160u32;
        let bg_h = (lh * 4 + 6) as u32;
        self.canvas.set_draw_color(Color::RGBA(0, 0, 0, 160));
        self.canvas.set_blend_mode(sdl2::render::BlendMode::Blend);
        self.canvas.fill_rect(Rect::new(x0 - 4, y - 3, bg_w, bg_h)).ok();
        self.canvas.set_blend_mode(sdl2::render::BlendMode::None);

        // Line 1: random generation state
        let rand_label = if random_enabled { "RAND: ON " } else { "RAND: OFF" };
        let rand_col   = if random_enabled { Color::RGB(80, 220, 80) } else { Color::RGB(150, 150, 150) };
        self.draw_text(Vector2::new(x0, y), rand_label, scale, rand_col)?;
        y += lh;

        // Line 2: vehicle count
        let vc_str = format!("VEHICLES: {}", vehicle_count);
        self.draw_text(Vector2::new(x0, y), &vc_str, scale, Color::RGB(200, 200, 200))?;
        y += lh;

        // Line 3: debug mode
        let dbg_label = if debug_mode { "DEBUG:  ON" } else { "DEBUG: OFF" };
        let dbg_col   = if debug_mode { Color::RGB(255, 200, 60) } else { Color::RGB(120, 120, 120) };
        self.draw_text(Vector2::new(x0, y), dbg_label, scale, dbg_col)?;
        y += lh;

        // Line 4: key hints
        self.draw_text(Vector2::new(x0, y), "R:RAND D:DBG", scale, Color::RGB(100, 140, 180))?;

        Ok(())
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

        // ── Always-visible stop lines at each entry face (2 px white) ────────
        // These mark the exact boundary where ATW tracking begins and vehicles
        // enter the managed intersection zone.
        self.canvas.set_draw_color(Color::RGB(255, 255, 255));
        let road_half = (LANE_COUNT as i32 / 2) * LANE_WIDTH; // 150 px
        // South face (northbound entry)
        self.canvas.fill_rect(Rect::new(center_x - road_half, center_y + size,     road_half as u32, 2)).ok();
        // North face (southbound entry)
        self.canvas.fill_rect(Rect::new(center_x - road_half, center_y - size - 1, road_half as u32, 2)).ok();
        // West face (eastbound entry)
        self.canvas.fill_rect(Rect::new(center_x - size - 1,  center_y - road_half, 2, road_half as u32)).ok();
        // East face (westbound entry)
        self.canvas.fill_rect(Rect::new(center_x + size,       center_y - road_half, 2, road_half as u32)).ok();

        Ok(())
    }

    fn draw_road_north(&mut self, center_x: i32, center_y: i32, size: i32) -> Result<(), Box<dyn std::error::Error>> {
        let road_width = (LANE_WIDTH * LANE_COUNT as i32) as u32;
        let road_start_x = center_x - (road_width as i32 / 2);
        // Draw road surface
        self.canvas.set_draw_color(Color::RGB(50, 50, 50));
        let road_rect = Rect::new(road_start_x, 0, road_width, (center_y - size) as u32);
        self.canvas.fill_rect(road_rect)?;

        // Draw lanes with markings
        self.draw_lanes_vertical(road_start_x, 0, road_width as i32, center_y - size, true)?;

        // Labels and arrows for southbound incoming lanes (left 3)
        let label_y = center_y - size - 40;
        let half = (LANE_COUNT as i32 / 2) * LANE_WIDTH; // 150
        self.draw_text_simple(road_start_x + LANE_WIDTH / 2 - 5, label_y, "r")?;
        self.draw_text_simple(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH - 5, label_y, "s")?;
        self.draw_text_simple(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH * 2 - 5, label_y, "l")?;
        self.draw_arrow_down(road_start_x + LANE_WIDTH / 2, label_y + 15)?;
        self.draw_arrow_down(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH, label_y + 15)?;
        self.draw_arrow_down(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH * 2, label_y + 15)?;

        // Arrows for northbound outgoing lanes (right 3)
        self.draw_arrow_up(road_start_x + half + LANE_WIDTH / 2, label_y + 15)?;
        self.draw_arrow_up(road_start_x + half + LANE_WIDTH / 2 + LANE_WIDTH, label_y + 15)?;
        self.draw_arrow_up(road_start_x + half + LANE_WIDTH / 2 + LANE_WIDTH * 2, label_y + 15)?;

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

        // Labels and arrows for northbound incoming lanes (right 3)
        let label_y = road_start_y + 30;
        let half = (LANE_COUNT as i32 / 2) * LANE_WIDTH; // 150
        self.draw_text_simple(road_start_x + half + LANE_WIDTH / 2 - 5, label_y, "l")?;
        self.draw_text_simple(road_start_x + half + LANE_WIDTH / 2 + LANE_WIDTH - 5, label_y, "s")?;
        self.draw_text_simple(road_start_x + half + LANE_WIDTH / 2 + LANE_WIDTH * 2 - 5, label_y, "r")?;
        self.draw_arrow_up(road_start_x + half + LANE_WIDTH / 2, label_y - 15)?;
        self.draw_arrow_up(road_start_x + half + LANE_WIDTH / 2 + LANE_WIDTH, label_y - 15)?;
        self.draw_arrow_up(road_start_x + half + LANE_WIDTH / 2 + LANE_WIDTH * 2, label_y - 15)?;

        // Arrows for southbound outgoing lanes (left 3)
        self.draw_arrow_down(road_start_x + LANE_WIDTH / 2, label_y - 15)?;
        self.draw_arrow_down(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH, label_y - 15)?;
        self.draw_arrow_down(road_start_x + LANE_WIDTH / 2 + LANE_WIDTH * 2, label_y - 15)?;

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

        // Labels and arrows for westbound incoming lanes (top 3)
        let label_x = road_start_x + 30;
        let half = (LANE_COUNT as i32 / 2) * LANE_WIDTH; // 150
        self.draw_text_simple(label_x, road_start_y + LANE_WIDTH / 2 - 5, "r")?;
        self.draw_text_simple(label_x, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH - 5, "s")?;
        self.draw_text_simple(label_x, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH * 2 - 5, "l")?;
        self.draw_arrow_left(label_x - 15, road_start_y + LANE_WIDTH / 2)?;
        self.draw_arrow_left(label_x - 15, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH)?;
        self.draw_arrow_left(label_x - 15, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH * 2)?;

        // Arrows for eastbound outgoing lanes (bottom 3)
        self.draw_arrow_right(label_x - 15, road_start_y + half + LANE_WIDTH / 2)?;
        self.draw_arrow_right(label_x - 15, road_start_y + half + LANE_WIDTH / 2 + LANE_WIDTH)?;
        self.draw_arrow_right(label_x - 15, road_start_y + half + LANE_WIDTH / 2 + LANE_WIDTH * 2)?;

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

        // Labels and arrows for eastbound incoming lanes (bottom 3)
        let label_x = road_end_x - 30;
        let half = (LANE_COUNT as i32 / 2) * LANE_WIDTH; // 150
        self.draw_text_simple(label_x, road_start_y + half + LANE_WIDTH / 2 - 5, "l")?;
        self.draw_text_simple(label_x, road_start_y + half + LANE_WIDTH / 2 + LANE_WIDTH - 5, "s")?;
        self.draw_text_simple(label_x, road_start_y + half + LANE_WIDTH / 2 + LANE_WIDTH * 2 - 5, "r")?;
        self.draw_arrow_right(label_x + 15, road_start_y + half + LANE_WIDTH / 2)?;
        self.draw_arrow_right(label_x + 15, road_start_y + half + LANE_WIDTH / 2 + LANE_WIDTH)?;
        self.draw_arrow_right(label_x + 15, road_start_y + half + LANE_WIDTH / 2 + LANE_WIDTH * 2)?;

        // Arrows for westbound outgoing lanes (top 3)
        self.draw_arrow_left(label_x + 15, road_start_y + LANE_WIDTH / 2)?;
        self.draw_arrow_left(label_x + 15, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH)?;
        self.draw_arrow_left(label_x + 15, road_start_y + LANE_WIDTH / 2 + LANE_WIDTH * 2)?;

        Ok(())
    }

    fn draw_lanes_vertical(&mut self, start_x: i32, start_y: i32, _width: i32, height: i32, _with_labels: bool) -> Result<(), Box<dyn std::error::Error>> {
        for i in 1..LANE_COUNT {
            let line_x = start_x + (i as i32) * LANE_WIDTH;
            if i == LANE_COUNT / 2 {
                // Solid yellow center divider separating outgoing from incoming traffic
                self.canvas.set_draw_color(Color::RGB(255, 200, 0));
                let rect = Rect::new(line_x - 1, start_y, 3, height as u32);
                self.canvas.fill_rect(rect)?;
            } else {
                self.canvas.set_draw_color(Color::RGB(255, 255, 255));
                self.draw_dashed_line_vertical(line_x, start_y, start_y + height)?;
            }
        }
        Ok(())
    }

    fn draw_lanes_horizontal(&mut self, start_x: i32, start_y: i32, width: i32, _height: i32, _with_labels: bool) -> Result<(), Box<dyn std::error::Error>> {
        for i in 1..LANE_COUNT {
            let line_y = start_y + (i as i32) * LANE_WIDTH;
            if i == LANE_COUNT / 2 {
                // Solid yellow center divider separating outgoing from incoming traffic
                self.canvas.set_draw_color(Color::RGB(255, 200, 0));
                let rect = Rect::new(start_x, line_y - 1, width as u32, 3);
                self.canvas.fill_rect(rect)?;
            } else {
                self.canvas.set_draw_color(Color::RGB(255, 255, 255));
                self.draw_dashed_line_horizontal(start_x, line_y, start_x + width)?;
            }
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
        let road_half = (LANE_WIDTH * LANE_COUNT as i32) / 2; // 150

        // Vertical lane dividers in intersection
        for i in 1..LANE_COUNT {
            let line_x = center_x - road_half + (i as i32) * LANE_WIDTH;
            if i == LANE_COUNT / 2 {
                // Solid yellow center divider
                self.canvas.set_draw_color(Color::RGB(255, 200, 0));
                let rect = Rect::new(line_x - 1, center_y - size, 3, (size * 2) as u32);
                self.canvas.fill_rect(rect)?;
            } else {
                self.canvas.set_draw_color(Color::RGB(200, 200, 200));
                let rect = Rect::new(line_x, center_y - size, 2, (size * 2) as u32);
                self.canvas.fill_rect(rect)?;
            }
        }

        // Horizontal lane dividers in intersection
        for i in 1..LANE_COUNT {
            let line_y = center_y - road_half + (i as i32) * LANE_WIDTH;
            if i == LANE_COUNT / 2 {
                // Solid yellow center divider
                self.canvas.set_draw_color(Color::RGB(255, 200, 0));
                let rect = Rect::new(center_x - size, line_y - 1, (size * 2) as u32, 3);
                self.canvas.fill_rect(rect)?;
            } else {
                self.canvas.set_draw_color(Color::RGB(200, 200, 200));
                let rect = Rect::new(center_x - size, line_y, (size * 2) as u32, 2);
                self.canvas.fill_rect(rect)?;
            }
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

    pub fn draw_vehicle(&mut self, vehicle: &Vehicle, textures: &CarTextures<'_>) -> Result<(), Box<dyn std::error::Error>> {
        let pos = vehicle.get_position();
        let x = pos.0 as i32;
        let y = pos.1 as i32;
        let w = 40u32;
        let h = 40u32;
        let dst = Rect::new(x - w as i32 / 2, y - h as i32 / 2, w, h);

        // Rotate the north-facing base sprite by the vehicle's interpolated animation
        // angle (0° = North/up, 90° = East/right, etc.).  During a turn the angle
        // smoothly interpolates between the old and new heading, giving the appearance
        // of the car physically steering through the intersection.
        let angle = vehicle.get_animation_angle() as f64;
        let center = sdl2::rect::Point::new(w as i32 / 2, h as i32 / 2);
        self.canvas.copy_ex(&textures.base, None, dst, angle, center, false, false)
            .map_err(|e| -> Box<dyn std::error::Error> { e.into() })?;

        // ── Turn signal indicator ───────────────────────────────────────────────
        // Draw a small amber rectangle at the front-left (Left turn) or front-right
        // (Right turn) corner of the sprite, rotated with the car.
        //
        // Coordinate convention: the sprite is 40×40, north-facing means "up" is
        // the front of the car.  In screen space (y-down) the SDL2 clockwise angle θ
        // corresponds to a mathematical CCW rotation, so the offset transform is:
        //   x' = ox·cos(θ) − oy·sin(θ)
        //   y' = ox·sin(θ) + oy·cos(θ)
        if vehicle.get_turn_signal_on() {
            let (ox, oy): (f32, f32) = match vehicle.get_route() {
                Route::Left     => (-14.0, -14.0), // front-left corner
                Route::Right    => ( 14.0, -14.0), // front-right corner
                Route::Straight => (  0.0, -16.0), // centre-front (safety fallback)
            };
            let theta = (angle as f32).to_radians();
            let sx = ox * theta.cos() - oy * theta.sin();
            let sy = ox * theta.sin() + oy * theta.cos();
            let sig_x = (pos.0 + sx) as i32;
            let sig_y = (pos.1 + sy) as i32;
            let sig_rect = sdl2::rect::Rect::new(sig_x - 3, sig_y - 3, 6, 6);
            self.canvas.set_draw_color(Color::RGB(255, 165, 0)); // amber
            self.canvas.fill_rect(sig_rect).ok();
        }

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

    /// Extended debug overlay for a single vehicle. Shows:
    ///   - Projected path: dashed approach line + bezier arc through intersection + exit tail
    ///   - Conflict highlight: red hitbox + "CF" badge + red safety circle
    ///   - Safety-distance circle: colour driven by velocity level, overridden red when conflicted
    ///   - Floating text: ID, velocity (px/s), ETA to entry edge, ATW time window [t_in, t_out]
    ///
    /// Expensive path arcs and circles are culled for vehicles beyond
    /// `DEBUG_CULL_RANGE` pixels from the intersection centre.
    pub fn draw_vehicle_debug(
        &mut self,
        vehicle: &Vehicle,
        all_vehicles: &[Vehicle],
        intersection: &Intersection,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let pos = vehicle.get_position();
        let cx  = pos.0 as i32;
        let cy  = pos.1 as i32;

        // ── Distance cull ────────────────────────────────────────────────────
        const DEBUG_CULL_RANGE: f32 = 600.0;
        let dx_c = pos.0 - intersection.center.0;
        let dy_c = pos.1 - intersection.center.1;
        let in_cull_range = dx_c * dx_c + dy_c * dy_c <= DEBUG_CULL_RANGE * DEBUG_CULL_RANGE;

        // ── Conflict detection (skipped when culled) ──────────────────────────
        let in_conflict = in_cull_range && !vehicle.is_route_applied()
            && all_vehicles.iter().any(|other| {
                other.get_id() != vehicle.get_id()
                    && predict_atw_conflict(vehicle, other, intersection)
            });

        // ── Projected path (expensive bezier — culled when far) ───────────────
        if in_cull_range && !vehicle.is_route_applied() {
            self.draw_vehicle_path(vehicle, intersection, in_conflict)?;
        }

        // ── Hitbox (30×16) — always drawn ────────────────────────────────────
        let hw = 15i32;
        let hh = 8i32;
        let hitbox_color = if in_conflict { Color::RGB(255, 50, 50) } else { Color::RGB(255, 165, 0) };
        let hitbox = sdl2::rect::Rect::new(cx - hw, cy - hh, (hw * 2) as u32, (hh * 2) as u32);
        self.canvas.set_draw_color(hitbox_color);
        self.canvas.draw_rect(hitbox).ok();

        if !in_cull_range {
            return Ok(());
        }

        // ── Safety-distance circle (dynamic colour + conflict override) ───────
        let radius = match vehicle.get_velocity_level() {
            VelocityLevel::Fast      => 120i32,
            VelocityLevel::Normal    => 80i32,
            VelocityLevel::Slow      => 65i32,
            VelocityLevel::Custom(_) => 55i32,
            VelocityLevel::Stopped   => 50i32,
        };
        let circle_color = if in_conflict {
            Color::RGB(255, 40, 40)
        } else {
            match vehicle.get_velocity_level() {
                VelocityLevel::Fast      => Color::RGB(0, 200, 0),
                VelocityLevel::Normal    => Color::RGB(200, 200, 0),
                VelocityLevel::Slow      => Color::RGB(255, 140, 0),
                VelocityLevel::Custom(_) => Color::RGB(220, 60, 0),
                VelocityLevel::Stopped   => Color::RGB(255, 0, 0),
            }
        };
        self.draw_circle(cx, cy, radius, circle_color)?;

        // ── Floating text block ──────────────────────────────────────────────
        let vel   = vehicle.get_velocity();
        let is_in = intersection.contains_point(pos);
        let half  = intersection.size;

        // Distance from current position to entry face along direction of travel
        let dist_to_entry = match vehicle.get_direction() {
            Direction::North => (pos.1 - (intersection.center.1 + half)).max(0.0),
            Direction::South => ((intersection.center.1 - half) - pos.1).max(0.0),
            Direction::East  => ((intersection.center.0 - half) - pos.0).max(0.0),
            Direction::West  => (pos.0 - (intersection.center.0 + half)).max(0.0),
        };
        let tta = if vel > 0.0 { dist_to_entry / vel } else { f32::MAX };

        // ATW time window
        let (t_in, t_out) = vehicle_time_window(vehicle, intersection);

        // Build label strings (glyph font: A-Z uppercase, 0-9, :  .  -  /)
        let id_str  = format!("ID:{}", vehicle.get_id());
        let vel_str = format!("V:{:.0}", vel);
        let eta_str = if is_in {
            "ETA:IN BOX".to_string()
        } else {
            format!("ETA:{:.1}S", tta.min(99.9))
        };
        let tw_str = if vehicle.is_route_applied() {
            "TW:DONE".to_string()
        } else if t_in >= f32::MAX / 2.0 {
            "TW:NONE".to_string()
        } else if is_in {
            format!("TX:{:.1}S", t_out.clamp(0.0, 99.9))
        } else {
            format!("TW:{:.1}-{:.1}", t_in.max(0.0).min(99.9), t_out.min(99.9))
        };

        let scale = 1;
        let row_h  = (GLYPH_H + 2) * scale; // 9 px per row at scale=1
        // Top of label block: 6 px above the hitbox top.
        let mut ly = cy - hh - 6 - row_h * 4;

        if in_conflict {
            let w = text_width("CF", scale);
            self.draw_text(Vector2::new(cx - w / 2, ly), "CF", scale, Color::RGB(255, 60, 60))?;
            ly += row_h;
        }

        let w = text_width(&id_str, scale);
        self.draw_text(Vector2::new(cx - w / 2, ly), &id_str, scale, Color::RGB(255, 255, 255))?;
        ly += row_h;

        let w = text_width(&vel_str, scale);
        self.draw_text(Vector2::new(cx - w / 2, ly), &vel_str, scale, Color::RGB(255, 255, 255))?;
        ly += row_h;

        let eta_col = if is_in { Color::RGB(100, 255, 160) } else { Color::RGB(100, 220, 255) };
        let w = text_width(&eta_str, scale);
        self.draw_text(Vector2::new(cx - w / 2, ly), &eta_str, scale, eta_col)?;
        ly += row_h;

        let tw_col = if in_conflict { Color::RGB(255, 120, 40) } else { Color::RGB(255, 200, 80) };
        let w = text_width(&tw_str, scale);
        self.draw_text(Vector2::new(cx - w / 2, ly), &tw_str, scale, tw_col)?;

        Ok(())
    }

    /// Draw semi-transparent approach zone overlays and entry-edge stop lines on the
    /// intersection to provide visual ATW context in debug mode.
    ///
    /// Zones drawn:
    ///  - 200 px green band before each entry face (matches TURN_SIGNAL_DISTANCE in main.rs)
    ///  - Amber-tinted intersection box
    ///  - 3-px yellow stop lines at each entry face
    ///  - Small directional labels ("NB IN", "SB IN", "EB IN", "WB IN")
    pub fn draw_intersection_zones_debug(
        &mut self,
        intersection: &Intersection,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let cx = intersection.center.0 as i32;
        let cy = intersection.center.1 as i32;
        let hs = intersection.size as i32;
        const APPROACH: i32 = 200;

        // ── Semi-transparent overlays ────────────────────────────────────────
        self.canvas.set_blend_mode(BlendMode::Blend);

        // Approach bands (dim green) – one per entry face
        self.canvas.set_draw_color(Color::RGBA(0, 220, 80, 22));
        // South face → northbound traffic enters here
        self.canvas.fill_rect(Rect::new(cx - hs, cy + hs, (hs * 2) as u32, APPROACH as u32)).ok();
        // North face → southbound traffic enters here
        self.canvas.fill_rect(Rect::new(cx - hs, cy - hs - APPROACH, (hs * 2) as u32, APPROACH as u32)).ok();
        // West face → eastbound traffic enters here
        self.canvas.fill_rect(Rect::new(cx - hs - APPROACH, cy - hs, APPROACH as u32, (hs * 2) as u32)).ok();
        // East face → westbound traffic enters here
        self.canvas.fill_rect(Rect::new(cx + hs, cy - hs, APPROACH as u32, (hs * 2) as u32)).ok();

        // Intersection box (dim amber)
        self.canvas.set_draw_color(Color::RGBA(255, 200, 0, 35));
        self.canvas.fill_rect(Rect::new(cx - hs, cy - hs, (hs * 2) as u32, (hs * 2) as u32)).ok();

        self.canvas.set_blend_mode(BlendMode::None);

        // ── Entry face stop lines (3 px bright yellow) ───────────────────────
        for d in 0..3i32 {
            self.canvas.set_draw_color(Color::RGB(255, 220, 0));
            // South face (northbound entry)
            self.canvas.fill_rect(Rect::new(cx - hs, cy + hs + d, (hs * 2) as u32, 1)).ok();
            // North face (southbound entry)
            self.canvas.fill_rect(Rect::new(cx - hs, cy - hs - d, (hs * 2) as u32, 1)).ok();
            // West face (eastbound entry)
            self.canvas.fill_rect(Rect::new(cx - hs - d, cy - hs, 1, (hs * 2) as u32)).ok();
            // East face (westbound entry)
            self.canvas.fill_rect(Rect::new(cx + hs + d, cy - hs, 1, (hs * 2) as u32)).ok();
        }

        // ── Entry face labels ─────────────────────────────────────────────────
        let lc = Color::RGB(255, 230, 80);
        // NB = northbound; these vehicles enter at the south (bottom) face
        self.draw_text(Vector2::new(cx - hs + 4,              cy + hs + 5),          "NB IN", 1, lc)?;
        // SB = southbound; enter at north (top) face
        self.draw_text(Vector2::new(cx - hs + 4,              cy - hs - 14),         "SB IN", 1, lc)?;
        // EB = eastbound; enter at west (left) face
        self.draw_text(Vector2::new(cx - hs - APPROACH + 4,   cy - hs + 4),          "EB IN", 1, lc)?;
        // WB = westbound; enter at east (right) face
        self.draw_text(Vector2::new(cx + hs + 4,              cy - hs + 4),          "WB IN", 1, lc)?;

        Ok(())
    }

    /// Draw a vehicle's projected path through the intersection:
    ///  1. Dashed approach line from current position to the entry face
    ///  2. Dashed quadratic Bézier arc through the intersection box
    ///  3. Short dashed exit continuation beyond the exit face
    ///
    /// Colour: cyan when no conflict, red-tinted when conflicted.
    fn draw_vehicle_path(
        &mut self,
        vehicle: &Vehicle,
        intersection: &Intersection,
        in_conflict: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let pos   = vehicle.get_position();
        let dir   = vehicle.get_direction();
        let route = vehicle.get_route();
        let cx    = intersection.center.0;
        let cy    = intersection.center.1;
        let hs    = intersection.size;

        let path_col = if in_conflict { Color::RGB(255, 100, 100) } else { Color::RGB(0, 200, 255) };

        // Entry point: where the vehicle's lane crosses the entry face of the box
        let entry: (f32, f32) = match dir {
            Direction::North => (pos.0, cy + hs),
            Direction::South => (pos.0, cy - hs),
            Direction::East  => (cx - hs, pos.1),
            Direction::West  => (cx + hs, pos.1),
        };

        // Is the vehicle still before the entry face?
        let before_entry = match dir {
            Direction::North => pos.1 > entry.1 + 1.0,
            Direction::South => pos.1 < entry.1 - 1.0,
            Direction::East  => pos.0 < entry.0 - 1.0,
            Direction::West  => pos.0 > entry.0 + 1.0,
        };

        // Approach line
        if before_entry {
            self.draw_dashed_segment(
                (pos.0 as i32, pos.1 as i32),
                (entry.0 as i32, entry.1 as i32),
                path_col, 12,
            )?;
        }

        // Bézier control + exit points for each (direction, route) combination.
        // Right-turn control point = inside corner of the turn (corner of the box).
        // Left-turn control point  = intersection centre (wider cross-box sweep).
        // Straight control point   = midpoint along the same axis (degenerates to a line).
        // Exit snap positions match those used in Vehicle::apply_route_turn().
        let (ctrl, exit): ((f32, f32), (f32, f32)) = match (dir, route) {
            (Direction::North, Route::Straight) => ((pos.0,   cy       ), (pos.0,   cy - hs)),
            (Direction::North, Route::Right)    => ((cx + hs, cy + hs  ), (cx + hs, 575.0  )),
            (Direction::North, Route::Left)     => ((cx,      cy       ), (cx - hs, 425.0  )),
            (Direction::South, Route::Straight) => ((pos.0,   cy       ), (pos.0,   cy + hs)),
            (Direction::South, Route::Right)    => ((cx - hs, cy - hs  ), (cx - hs, 325.0  )),
            (Direction::South, Route::Left)     => ((cx,      cy       ), (cx + hs, 475.0  )),
            (Direction::East,  Route::Straight) => ((cx,      pos.1    ), (cx + hs, pos.1  )),
            (Direction::East,  Route::Right)    => ((cx - hs, cy + hs  ), (575.0,   cy + hs)),
            (Direction::East,  Route::Left)     => ((cx,      cy       ), (725.0,   cy - hs)),
            (Direction::West,  Route::Straight) => ((cx,      pos.1    ), (cx - hs, pos.1  )),
            (Direction::West,  Route::Right)    => ((cx + hs, cy - hs  ), (825.0,   cy - hs)),
            (Direction::West,  Route::Left)     => ((cx,      cy       ), (675.0,   cy + hs)),
        };

        // If already inside the box, start the bezier from current position
        let bez_start = if before_entry { entry } else { pos };
        self.draw_bezier_dashed(bez_start, ctrl, exit, path_col)?;

        // Exit continuation (80 px beyond the exit point in the post-turn direction)
        let exit_dir = match (dir, route) {
            (d, Route::Straight)             => d,
            (Direction::North, Route::Right) => Direction::East,
            (Direction::North, Route::Left)  => Direction::West,
            (Direction::South, Route::Right) => Direction::West,
            (Direction::South, Route::Left)  => Direction::East,
            (Direction::East,  Route::Right) => Direction::South,
            (Direction::East,  Route::Left)  => Direction::North,
            (Direction::West,  Route::Right) => Direction::North,
            (Direction::West,  Route::Left)  => Direction::South,
        };
        let (ddx, ddy): (f32, f32) = match exit_dir {
            Direction::North => (0.0,  -80.0),
            Direction::South => (0.0,   80.0),
            Direction::East  => (80.0,   0.0),
            Direction::West  => (-80.0,  0.0),
        };
        self.draw_dashed_segment(
            (exit.0 as i32, exit.1 as i32),
            ((exit.0 + ddx) as i32, (exit.1 + ddy) as i32),
            path_col, 12,
        )?;

        Ok(())
    }

    /// Draw a dashed straight line between two points.
    /// `dash` controls the length of each drawn segment; the gap equals the same length.
    fn draw_dashed_segment(
        &mut self,
        from: (i32, i32),
        to:   (i32, i32),
        color: Color,
        dash: i32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let dx  = (to.0 - from.0) as f32;
        let dy  = (to.1 - from.1) as f32;
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1.0 { return Ok(()); }
        let nx     = dx / len;
        let ny     = dy / len;
        let period = (dash * 2) as f32;
        let mut d  = 0.0f32;
        self.canvas.set_draw_color(color);
        while d < len {
            let d2 = (d + dash as f32).min(len);
            let x0 = (from.0 as f32 + nx * d)  as i32;
            let y0 = (from.1 as f32 + ny * d)  as i32;
            let x1 = (from.0 as f32 + nx * d2) as i32;
            let y1 = (from.1 as f32 + ny * d2) as i32;
            self.draw_line_simple(x0, y0, x1, y1)?;
            d += period;
        }
        Ok(())
    }

    /// Draw a dashed quadratic Bézier curve from `p0` via control point `ctrl` to `p2`.
    /// Sampled into 24 equal-parameter segments; every other segment is drawn (12 dashes).
    fn draw_bezier_dashed(
        &mut self,
        p0:   (f32, f32),
        ctrl: (f32, f32),
        p2:   (f32, f32),
        color: Color,
    ) -> Result<(), Box<dyn std::error::Error>> {
        const STEPS: usize = 24;
        let pts: Vec<(i32, i32)> = (0..=STEPS)
            .map(|i| {
                let t = i as f32 / STEPS as f32;
                let u = 1.0 - t;
                let x = u * u * p0.0 + 2.0 * t * u * ctrl.0 + t * t * p2.0;
                let y = u * u * p0.1 + 2.0 * t * u * ctrl.1 + t * t * p2.1;
                (x as i32, y as i32)
            })
            .collect();
        self.canvas.set_draw_color(color);
        for i in (0..STEPS).step_by(2) {
            self.draw_line_simple(pts[i].0, pts[i].1, pts[i + 1].0, pts[i + 1].1)?;
        }
        Ok(())
    }

    /// Draw a circle outline using the midpoint circle algorithm.
    /// Draw a circle outline using the midpoint circle algorithm.
    /// All ~8r points are batched into a single `draw_points` call to
    /// minimise SDL2 per-call overhead at high vehicle counts.
    fn draw_circle(&mut self, cx: i32, cy: i32, r: i32, color: Color) -> Result<(), Box<dyn std::error::Error>> {
        self.canvas.set_draw_color(color);
        let capacity = ((r as usize) * 8).max(8);
        let mut pts: Vec<sdl2::rect::Point> = Vec::with_capacity(capacity);
        let mut x   = r;
        let mut y   = 0i32;
        let mut err = 0i32;
        while x >= y {
            pts.push(sdl2::rect::Point::new(cx + x, cy + y));
            pts.push(sdl2::rect::Point::new(cx - x, cy + y));
            pts.push(sdl2::rect::Point::new(cx + x, cy - y));
            pts.push(sdl2::rect::Point::new(cx - x, cy - y));
            pts.push(sdl2::rect::Point::new(cx + y, cy + x));
            pts.push(sdl2::rect::Point::new(cx - y, cy + x));
            pts.push(sdl2::rect::Point::new(cx + y, cy - x));
            pts.push(sdl2::rect::Point::new(cx - y, cy - x));
            y += 1;
            err += 2 * y - 1;
            if err > x { x -= 1; err -= 2 * x + 1; }
        }
        self.canvas.draw_points(pts.as_slice()).ok();
        Ok(())
    }

    /// Draw a tiny "id=N vel=L" label above the vehicle using small dots.
    fn draw_debug_label(&mut self, cx: i32, cy: i32, id: u32, vel: &str, color: Color) -> Result<(), Box<dyn std::error::Error>> {
        self.canvas.set_draw_color(color);
        // Just draw a small 4×4 pixel dot so it's visible without a font
        let text = format!("{}{}", id, vel);
        let mut ox = cx - (text.len() as i32 * 4) / 2;
        for ch in text.chars() {
            // Draw a simple 3×5 pixel digit/letter using a minimal bitmap
            let bitmap: &[u8] = match ch {
                '0' => &[0b111,0b101,0b101,0b101,0b111],
                '1' => &[0b010,0b110,0b010,0b010,0b111],
                '2' => &[0b111,0b001,0b111,0b100,0b111],
                '3' => &[0b111,0b001,0b011,0b001,0b111],
                '4' => &[0b101,0b101,0b111,0b001,0b001],
                '5' => &[0b111,0b100,0b111,0b001,0b111],
                '6' => &[0b111,0b100,0b111,0b101,0b111],
                '7' => &[0b111,0b001,0b001,0b001,0b001],
                '8' => &[0b111,0b101,0b111,0b101,0b111],
                '9' => &[0b111,0b101,0b111,0b001,0b111],
                'F' => &[0b111,0b100,0b111,0b100,0b100],
                'N' => &[0b101,0b111,0b111,0b101,0b101],
                'S' => &[0b111,0b100,0b111,0b001,0b111],
                'X' => &[0b101,0b101,0b010,0b101,0b101],
                _   => &[0b000,0b000,0b010,0b000,0b000],
            };
            for (row, &bits) in bitmap.iter().enumerate() {
                for col in 0..3i32 {
                    if bits & (1 << (2 - col)) != 0 {
                        self.canvas.draw_point(sdl2::rect::Point::new(ox + col, cy + row as i32)).ok();
                    }
                }
            }
            ox += 4;
        }
        Ok(())
    }

    // ===== Pixel-text rendering (5x7 bitmap font, no SDL2_ttf required) =====

    fn draw_glyph(
        &mut self,
        origin: Vector2<i32>,
        g: &Glyph,
        scale: i32,
        color: Color,
    ) -> Result<(), Box<dyn std::error::Error>> {
        self.canvas.set_draw_color(color);
        for (row_idx, row) in g.iter().enumerate() {
            for col in 0..GLYPH_W {
                // Bit GLYPH_W-1 = leftmost column.
                let bit = (row >> (GLYPH_W - 1 - col)) & 1;
                if bit == 1 {
                    let x = origin.x + col * scale;
                    let y = origin.y + (row_idx as i32) * scale;
                    self.canvas.fill_rect(Rect::new(x, y, scale as u32, scale as u32))?;
                }
            }
        }
        Ok(())
    }

    pub fn draw_text(
        &mut self,
        origin: Vector2<i32>,
        text: &str,
        scale: i32,
        color: Color,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let advance = (GLYPH_W + GLYPH_SPACING) * scale;
        for (i, c) in text.chars().enumerate() {
            let pos = origin + Vector2::new(advance * i as i32, 0);
            self.draw_glyph(pos, glyph_for(c), scale, color)?;
        }
        Ok(())
    }

    fn draw_text_centered_x(
        &mut self,
        center_x: i32,
        y: i32,
        text: &str,
        scale: i32,
        color: Color,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let w = text_width(text, scale);
        self.draw_text(Vector2::new(center_x - w / 2, y), text, scale, color)
    }

    // ===== Final-stats dialog window =====

    pub fn draw_summary(
        &mut self,
        stats: &Statistics,
        total_time: f32,
        mouse: (i32, i32),
    ) -> Result<(), Box<dyn std::error::Error>> {
        let canvas_size = Vector2::new(self.width as i32, self.height as i32);
        let origin = window_origin(canvas_size);

        // ── Backdrop ─────────────────────────────────────────────────────────
        self.canvas.set_draw_color(Color::RGB(10, 10, 15));
        self.canvas.fill_rect(Rect::new(0, 0, self.width, self.height))?;

        // ── Window chrome ─────────────────────────────────────────────────────
        self.canvas.set_draw_color(Color::RGB(180, 180, 180));
        self.canvas.fill_rect(Rect::new(origin.x, origin.y, WIN_W as u32, WIN_H as u32))?;
        self.canvas.set_draw_color(Color::RGB(22, 22, 28));
        self.canvas.fill_rect(Rect::new(origin.x + 2, origin.y + 2,
            (WIN_W - 4) as u32, (WIN_H - 4) as u32))?;

        // ── Title strip ───────────────────────────────────────────────────────
        let title_h = 30i32;
        self.canvas.set_draw_color(Color::RGB(50, 55, 65));
        self.canvas.fill_rect(Rect::new(origin.x + 2, origin.y + 2,
            (WIN_W - 4) as u32, title_h as u32))?;
        self.draw_text_centered_x(origin.x + WIN_W / 2, origin.y + 9,
            "SESSION STATISTICS", 2, Color::RGB(220, 225, 235))?;

        // ── Layout constants ──────────────────────────────────────────────────
        let scale    = 1; // glyph scale for data rows
        let row_h    = (GLYPH_H + 3) * scale; // ~10 px per row
        let label_x  = origin.x + 20;
        let val_x    = origin.x + WIN_W - 20; // right-align values here
        let sec_col  = Color::RGB(100, 140, 180); // section heading colour
        let lab_col  = Color::RGB(160, 165, 175);
        let val_col  = Color::RGB(100, 220, 140);
        let warn_col = Color::RGB(255, 120, 60);

        let mut cy = origin.y + title_h + 14;

        macro_rules! section {
            ($title:expr) => {{
                self.canvas.set_draw_color(Color::RGB(38, 42, 50));
                self.canvas.fill_rect(Rect::new(origin.x + 2, cy - 2,
                    (WIN_W - 4) as u32, (row_h + 4) as u32)).ok();
                self.draw_text(Vector2::new(label_x, cy), $title, scale, sec_col)?;
                cy += row_h + 6;
            }};
        }

        macro_rules! row {
            ($label:expr, $val:expr, $col:expr) => {{
                self.draw_text(Vector2::new(label_x + 8, cy), $label, scale, lab_col)?;
                let vw = text_width($val, scale);
                self.draw_text(Vector2::new(val_x - vw, cy), $val, scale, $col)?;
                cy += row_h + 2;
            }};
        }

        // ── Section 1: Traffic throughput ─────────────────────────────────────
        section!("TRAFFIC");

        let generated = stats.get_vehicles_generated();
        let passed    = stats.get_vehicle_count();
        let throughput = if total_time > 0.0 {
            passed as f32 / (total_time / 60.0)
        } else { 0.0 };

        row!("GENERATED",   &format!("{}", generated),           val_col);
        row!("PASSED",      &format!("{}", passed),              val_col);
        row!("THROUGHPUT",  &format!("{:.1} VEH/MIN", throughput), val_col);
        row!("SESSION",     &format!("{:.1}S", total_time),      val_col);

        cy += 4;

        // ── Section 2: Velocity statistics ───────────────────────────────────
        section!("VELOCITY PX/S");

        let mean   = stats.get_mean_velocity();
        let median = stats.get_median_velocity();
        let stddev = stats.get_stddev_velocity();
        let vmax   = stats.get_max_velocity();
        let vmin   = stats.get_min_velocity();

        row!("MIN",    &format!("{:.1}", vmin),   val_col);
        row!("MEAN",   &format!("{:.1}", mean),   val_col);
        row!("MEDIAN", &format!("{:.1}", median), val_col);
        row!("MAX",    &format!("{:.1}", vmax),   val_col);
        row!("STDDEV", &format!("{:.1}", stddev), val_col);

        cy += 4;

        // ── Velocity histogram ────────────────────────────────────────────────
        const BINS: usize = 16;
        const HIST_W: i32 = WIN_W - 44; // total bar-area width
        const HIST_H: i32 = 36;
        let hist_x    = origin.x + 22;
        let hist_top  = cy;

        let hist_max_vel = 320.0f32; // upper boundary covers Fast=260 + some headroom
        let counts = stats.get_velocity_histogram::<BINS>(hist_max_vel);
        let max_count = *counts.iter().max().unwrap_or(&1).max(&1);

        let bar_w  = HIST_W / BINS as i32;
        let bar_gap = 1;

        // draw a faint background for the histogram area
        self.canvas.set_draw_color(Color::RGB(28, 32, 40));
        self.canvas.fill_rect(Rect::new(hist_x - 2, hist_top - 2,
            (HIST_W + 4) as u32, (HIST_H + 4) as u32)).ok();

        for (i, &count) in counts.iter().enumerate() {
            let bar_h = if max_count > 0 {
                (count as f32 / max_count as f32 * HIST_H as f32) as i32
            } else { 0 };

            // Colour: gradient from green (slow) to red (fast)
            let t = i as f32 / (BINS as f32 - 1.0);
            let r = (t * 220.0) as u8;
            let g = ((1.0 - t) * 200.0) as u8;
            let bar_col = Color::RGB(r, g, 40);

            let bx = hist_x + i as i32 * bar_w;
            let by = hist_top + HIST_H - bar_h;
            if bar_h > 0 {
                self.canvas.set_draw_color(bar_col);
                self.canvas.fill_rect(Rect::new(bx, by,
                    (bar_w - bar_gap).max(1) as u32, bar_h as u32)).ok();
            }
        }

        // axis labels: 0 on left, hist_max_vel on right
        cy = hist_top + HIST_H + 4;
        self.draw_text(Vector2::new(hist_x, cy), "0", scale, lab_col)?;
        let rlab = format!("{:.0}", hist_max_vel);
        let rw   = text_width(&rlab, scale);
        self.draw_text(Vector2::new(hist_x + HIST_W - rw, cy), &rlab, scale, lab_col)?;

        cy += row_h + 6;

        // ── Section 3: Intersection timing ───────────────────────────────────
        section!("TIME IN INTERSECTION S");

        row!("MIN",    &format!("{:.2}", stats.get_min_time()),    val_col);
        row!("MEAN",   &format!("{:.2}", stats.get_mean_time()),   val_col);
        row!("MAX",    &format!("{:.2}", stats.get_max_time()),    val_col);
        row!("STDDEV", &format!("{:.2}", stats.get_stddev_time()), val_col);

        cy += 4;

        // ── Section 4: Safety ────────────────────────────────────────────────
        section!("SAFETY");

        let close_col = if stats.get_close_calls() > 0 { warn_col } else { val_col };
        let coll_col  = if stats.get_collisions() > 0 { Color::RGB(255, 50, 50) } else { val_col };
        row!("CLOSE CALLS", &format!("{}", stats.get_close_calls()), close_col);
        row!("COLLISIONS",  &format!("{}", stats.get_collisions()),  coll_col);

        cy += 4;

        // ── Footer row: Restart and OK buttons ───────────────────────────────
        let draw_btn = |canvas: &mut sdl2::render::Canvas<sdl2::video::Window>,
                        btn: sdl2::rect::Rect,
                        hovered: bool,
                        col_normal: Color,
                        col_hover: Color| {
            let fill = if hovered { col_hover } else { col_normal };
            canvas.set_draw_color(Color::RGB(180, 180, 180));
            canvas.fill_rect(btn).ok();
            canvas.set_draw_color(fill);
            canvas.fill_rect(Rect::new(btn.x() + 1, btn.y() + 1,
                btn.width() - 2, btn.height() - 2)).ok();
        };

        let restart_btn = restart_button_rect(canvas_size);
        let ok_btn      = ok_button_rect(canvas_size);
        let restart_hov = point_in_restart_button(canvas_size, mouse.0, mouse.1);
        let ok_hov      = point_in_ok_button(canvas_size, mouse.0, mouse.1);

        // Restart (left button, orange accent)
        draw_btn(&mut self.canvas, restart_btn,
            restart_hov, Color::RGB(80, 45, 12), Color::RGB(140, 80, 20));
        // OK / Quit (right button, neutral grey)
        draw_btn(&mut self.canvas, ok_btn,
            ok_hov, Color::RGB(50, 55, 65), Color::RGB(80, 85, 95));

        // Button labels via the glyph font
        let btn_scale = 2;
        let label_h   = GLYPH_H * btn_scale;

        let restart_label = "RESTART";
        let rw = text_width(restart_label, btn_scale);
        self.draw_text(
            Vector2::new(
                restart_btn.x() + (restart_btn.width() as i32 - rw) / 2,
                restart_btn.y() + (restart_btn.height() as i32 - label_h) / 2,
            ),
            restart_label, btn_scale, Color::RGB(255, 180, 80))?;

        let ok_label = "QUIT";
        let ow = text_width(ok_label, btn_scale);
        self.draw_text(
            Vector2::new(
                ok_btn.x() + (ok_btn.width() as i32 - ow) / 2,
                ok_btn.y() + (ok_btn.height() as i32 - label_h) / 2,
            ),
            ok_label, btn_scale, Color::RGB(200, 205, 215))?;

        Ok(())
    }
}

