use nalgebra::Vector2;
use sdl2::rect::Rect;

pub const WIN_W: i32 = 640;
pub const WIN_H: i32 = 560;

pub const BTN_W: i32 = 100;
pub const BTN_H: i32 = 28;
pub const BTN_BOTTOM_PAD: i32 = 20;
// Gap between the two buttons (OK and Restart) in the footer row.
const BTN_GAP: i32 = 16;

/// Top-left corner of the centered Statistics window.
pub fn window_origin(canvas: Vector2<i32>) -> Vector2<i32> {
    let win = Vector2::new(WIN_W, WIN_H);
    (canvas - win) / 2
}

/// Total width of both footer buttons side-by-side with BTN_GAP between them.
fn footer_buttons_total_width() -> i32 {
    BTN_W * 2 + BTN_GAP
}

/// Rect of the OK / Quit button (right button in the footer row).
pub fn ok_button_rect(canvas: Vector2<i32>) -> Rect {
    let origin = window_origin(canvas);
    let row_start_x = origin.x + (WIN_W - footer_buttons_total_width()) / 2;
    let x = row_start_x + BTN_W + BTN_GAP;
    let y = origin.y + WIN_H - BTN_H - BTN_BOTTOM_PAD;
    Rect::new(x, y, BTN_W as u32, BTN_H as u32)
}

/// Rect of the Restart button (left button in the footer row).
pub fn restart_button_rect(canvas: Vector2<i32>) -> Rect {
    let origin = window_origin(canvas);
    let x = origin.x + (WIN_W - footer_buttons_total_width()) / 2;
    let y = origin.y + WIN_H - BTN_H - BTN_BOTTOM_PAD;
    Rect::new(x, y, BTN_W as u32, BTN_H as u32)
}

/// True when (x, y) falls inside the OK button.
pub fn point_in_ok_button(canvas: Vector2<i32>, x: i32, y: i32) -> bool {
    ok_button_rect(canvas).contains_point((x, y))
}

/// True when (x, y) falls inside the Restart button.
pub fn point_in_restart_button(canvas: Vector2<i32>, x: i32, y: i32) -> bool {
    restart_button_rect(canvas).contains_point((x, y))
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
        let origin = window_origin(CANVAS);
        let expected_y = origin.y + WIN_H - BTN_H - BTN_BOTTOM_PAD;
        assert_eq!(r.y(), expected_y);
        assert_eq!(r.width(), BTN_W as u32);
        assert_eq!(r.height(), BTN_H as u32);
    }

    #[test]
    fn restart_button_is_left_of_ok_button() {
        let rb = restart_button_rect(CANVAS);
        let ok = ok_button_rect(CANVAS);
        assert!(rb.x() < ok.x(), "restart button must be left of ok");
        assert_eq!(rb.y(), ok.y());
        assert_eq!(rb.width(), ok.width());
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

    #[test]
    fn point_inside_restart_button_returns_true() {
        let r = restart_button_rect(CANVAS);
        let mid_x = r.x() + (r.width() / 2) as i32;
        let mid_y = r.y() + (r.height() / 2) as i32;
        assert!(point_in_restart_button(CANVAS, mid_x, mid_y));
    }
}
