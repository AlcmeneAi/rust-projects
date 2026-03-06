use sdl2::render::{Canvas, TextureCreator};
use sdl2::video::Window;
use sdl2::rect::Rect;
use sdl2::pixels::Color;
use crate::vehicle::Vehicle;
use crate::intersection::Intersection;

pub struct Renderer {
    canvas: Canvas<Window>,
    width: u32,
    height: u32,
}

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
        self.canvas.set_draw_color(Color::RGB(240, 240, 240));
        self.canvas.clear();
    }

    pub fn present(&mut self) {
        self.canvas.present();
    }

    pub fn draw_intersection(&mut self, intersection: &Intersection) -> Result<(), Box<dyn std::error::Error>> {
        let size = (intersection.size * 2.0) as u32;
        let x = (intersection.center.0 - intersection.size) as i32;
        let y = (intersection.center.1 - intersection.size) as i32;

        self.canvas.set_draw_color(Color::RGB(200, 200, 200));
        let rect = Rect::new(x, y, size, size);
        self.canvas.fill_rect(rect)?;

        self.canvas.set_draw_color(Color::RGB(255, 255, 255));
        let lane_width = 40;
        for i in 0..3 {
            let lane_y = (intersection.center.1 - intersection.size + (i as f32) * lane_width as f32) as i32;
            let rect = Rect::new(x, lane_y, size, lane_width as u32);
            self.canvas.draw_rect(rect)?;
        }

        for i in 0..3 {
            let lane_x = (intersection.center.0 - intersection.size + (i as f32) * lane_width as f32) as i32;
            let rect = Rect::new(lane_x, y, lane_width as u32, size);
            self.canvas.draw_rect(rect)?;
        }

        self.canvas.set_draw_color(Color::RGB(50, 50, 50));
        let north_rect = Rect::new(
            (intersection.center.0 - intersection.size / 2.0) as i32,
            0,
            (intersection.size) as u32,
            (intersection.center.1 - intersection.size) as u32,
        );
        self.canvas.fill_rect(north_rect)?;

        let south_rect = Rect::new(
            (intersection.center.0 - intersection.size / 2.0) as i32,
            (intersection.center.1 + intersection.size) as i32,
            (intersection.size) as u32,
            (self.height as f32 - intersection.center.1 - intersection.size) as u32,
        );
        self.canvas.fill_rect(south_rect)?;

        let east_rect = Rect::new(
            (intersection.center.0 + intersection.size) as i32,
            (intersection.center.1 - intersection.size / 2.0) as i32,
            (self.width as f32 - intersection.center.0 - intersection.size) as u32,
            (intersection.size) as u32,
        );
        self.canvas.fill_rect(east_rect)?;

        let west_rect = Rect::new(
            0,
            (intersection.center.1 - intersection.size / 2.0) as i32,
            (intersection.center.0 - intersection.size) as u32,
            (intersection.size) as u32,
        );
        self.canvas.fill_rect(west_rect)?;

        Ok(())
    }

    pub fn draw_vehicle(&mut self, vehicle: &Vehicle) -> Result<(), Box<dyn std::error::Error>> {
        let pos = vehicle.get_position();
        let x = pos.0 as i32;
        let y = pos.1 as i32;
        let size = 20;

        let color = match vehicle.get_id() % 5 {
            0 => Color::RGB(255, 0, 0),
            1 => Color::RGB(0, 0, 255),
            2 => Color::RGB(0, 255, 0),
            3 => Color::RGB(255, 255, 0),
            _ => Color::RGB(255, 0, 255),
        };

        self.canvas.set_draw_color(color);
        let rect = Rect::new(x - size / 2, y - size / 2, size as u32, size as u32);
        self.canvas.fill_rect(rect)?;

        Ok(())
    }
}
