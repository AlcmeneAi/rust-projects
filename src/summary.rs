use nalgebra::Vector2;
use sdl2::rect::Rect;

pub const WIN_W: i32 = 560;
pub const WIN_H: i32 = 380;

pub const BTN_W: i32 = 80;
pub const BTN_H: i32 = 28;
pub const BTN_BOTTOM_PAD: i32 = 24;

/// Top-left corner of the centered Statistics window.
pub fn window_origin(canvas: Vector2<i32>) -> Vector2<i32> {
    let win = Vector2::new(WIN_W, WIN_H);
    (canvas - win) / 2
}

/// Rect describing the OK button hit zone, derived from the same layout math
/// as the drawn button so the two cannot drift apart.
pub fn ok_button_rect(canvas: Vector2<i32>) -> Rect {
    let origin = window_origin(canvas);
    let x = origin.x + (WIN_W - BTN_W) / 2;
    let y = origin.y + WIN_H - BTN_H - BTN_BOTTOM_PAD;
    Rect::new(x, y, BTN_W as u32, BTN_H as u32)
}

/// True when (x, y) falls inside the OK button.
pub fn point_in_ok_button(canvas: Vector2<i32>, x: i32, y: i32) -> bool {
    ok_button_rect(canvas).contains_point((x, y))
}

#[cfg(test)]
mod tests {
    use super::*;

    const CANVAS: Vector2<i32> = Vector2::new(1400, 900);

    #[test]
    fn window_is_centered_on_canvas() {
        let origin = window_origin(CANVAS);
        assert_eq!(origin.x, (1400 - WIN_W) / 2);
        assert_eq!(origin.y, (900 - WIN_H) / 2);
    }

    #[test]
    fn ok_button_horizontally_centered_on_window() {
        let r = ok_button_rect(CANVAS);
        let win_origin = window_origin(CANVAS);
        let expected_x = win_origin.x + (WIN_W - BTN_W) / 2;
        let expected_y = win_origin.y + WIN_H - BTN_H - BTN_BOTTOM_PAD;
        assert_eq!(r.x(), expected_x);
        assert_eq!(r.y(), expected_y);
        assert_eq!(r.width(), BTN_W as u32);
        assert_eq!(r.height(), BTN_H as u32);
    }

    #[test]
    fn point_inside_ok_button_returns_true() {
        let r = ok_button_rect(CANVAS);
        let mid_x = r.x() + (r.width() / 2) as i32;
        let mid_y = r.y() + (r.height() / 2) as i32;
        assert!(point_in_ok_button(CANVAS, mid_x, mid_y));
    }

    #[test]
    fn point_outside_ok_button_returns_false() {
        assert!(!point_in_ok_button(CANVAS, 0, 0));
        assert!(!point_in_ok_button(CANVAS, 1399, 899));
    }

    #[test]
    fn point_just_outside_ok_button_returns_false() {
        let r = ok_button_rect(CANVAS);
        // One pixel left of the left edge.
        assert!(!point_in_ok_button(CANVAS, r.x() - 1, r.y() + 1));
        // One pixel below the bottom edge.
        assert!(!point_in_ok_button(CANVAS, r.x() + 1, r.y() + r.height() as i32));
    }
}
