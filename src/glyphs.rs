//! 5x7 bitmap font for retro pixel-text rendering.
//!
//! Each glyph is `[u8; 7]`. Each row uses bits 4..0 (bit 4 = leftmost column).
//! Bit set = lit pixel.

pub const GLYPH_W: i32 = 5;
pub const GLYPH_H: i32 = 7;
pub const GLYPH_SPACING: i32 = 1;

pub type Glyph = [u8; GLYPH_H as usize];

const SPACE: Glyph = [0; 7];

#[rustfmt::skip]
const DIGITS: [Glyph; 10] = [
    // 0
    [0b01110, 0b10001, 0b10011, 0b10101, 0b11001, 0b10001, 0b01110],
    // 1
    [0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
    // 2
    [0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111],
    // 3
    [0b11110, 0b00001, 0b00001, 0b01110, 0b00001, 0b00001, 0b11110],
    // 4
    [0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010],
    // 5
    [0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110],
    // 6
    [0b00110, 0b01000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110],
    // 7
    [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000],
    // 8
    [0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110],
    // 9
    [0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00010, 0b01100],
];

#[rustfmt::skip]
const LETTERS: [Glyph; 26] = [
    // A
    [0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
    // B
    [0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110],
    // C
    [0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110],
    // D
    [0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110],
    // E
    [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111],
    // F
    [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000],
    // G
    [0b01110, 0b10001, 0b10000, 0b10111, 0b10001, 0b10001, 0b01111],
    // H
    [0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
    // I
    [0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
    // J
    [0b00111, 0b00010, 0b00010, 0b00010, 0b00010, 0b10010, 0b01100],
    // K
    [0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001],
    // L
    [0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111],
    // M
    [0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001],
    // N
    [0b10001, 0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001],
    // O
    [0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
    // P
    [0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000],
    // Q
    [0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101],
    // R
    [0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001],
    // S
    [0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110],
    // T
    [0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100],
    // U
    [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
    // V
    [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100],
    // W
    [0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b10101, 0b01010],
    // X
    [0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001],
    // Y
    [0b10001, 0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b00100],
    // Z
    [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111],
];

const SLASH: Glyph    = [0b00001, 0b00010, 0b00010, 0b00100, 0b01000, 0b01000, 0b10000];
const COLON: Glyph    = [0b00000, 0b01100, 0b01100, 0b00000, 0b01100, 0b01100, 0b00000];
const PERIOD: Glyph   = [0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b01100, 0b01100];
const MINUS: Glyph    = [0b00000, 0b00000, 0b00000, 0b11111, 0b00000, 0b00000, 0b00000];

/// Returns the glyph for a character. Lowercase is folded to uppercase.
/// Unknown characters return SPACE.
pub fn glyph_for(c: char) -> &'static Glyph {
    let upper = c.to_ascii_uppercase();
    match upper {
        '0'..='9' => &DIGITS[(upper as u8 - b'0') as usize],
        'A'..='Z' => &LETTERS[(upper as u8 - b'A') as usize],
        '/' => &SLASH,
        ':' => &COLON,
        '.' => &PERIOD,
        '-' => &MINUS,
        _ => &SPACE,
    }
}

/// Total pixel width of a string at the given scale, including inter-glyph spacing.
pub fn text_width(s: &str, scale: i32) -> i32 {
    let n = s.chars().count() as i32;
    if n == 0 { return 0; }
    (n * GLYPH_W + (n - 1) * GLYPH_SPACING) * scale
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn space_returns_blank_glyph() {
        let g = glyph_for(' ');
        assert!(g.iter().all(|row| *row == 0));
    }

    #[test]
    fn unknown_char_returns_blank_glyph() {
        let g = glyph_for('@');
        assert!(g.iter().all(|row| *row == 0));
    }

    #[test]
    fn digit_zero_has_lit_pixels() {
        let g = glyph_for('0');
        assert!(g.iter().any(|row| *row != 0), "'0' must have at least one lit pixel");
    }

    #[test]
    fn letter_a_uppercase_and_lowercase_match() {
        let upper = glyph_for('A');
        let lower = glyph_for('a');
        assert_eq!(upper, lower);
    }

    #[test]
    fn each_letter_is_distinct_from_blank() {
        for c in 'A'..='Z' {
            let g = glyph_for(c);
            assert!(g.iter().any(|row| *row != 0), "letter {c} must not be blank");
        }
    }

    #[test]
    fn each_digit_is_distinct_from_blank() {
        for c in '0'..='9' {
            let g = glyph_for(c);
            assert!(g.iter().any(|row| *row != 0), "digit {c} must not be blank");
        }
    }

    #[test]
    fn text_width_empty_is_zero() {
        assert_eq!(text_width("", 1), 0);
        assert_eq!(text_width("", 3), 0);
    }

    #[test]
    fn text_width_single_glyph_is_glyph_width() {
        assert_eq!(text_width("A", 1), GLYPH_W);
        assert_eq!(text_width("A", 3), GLYPH_W * 3);
    }

    #[test]
    fn text_width_two_glyphs_includes_spacing() {
        // 2 glyphs + 1 inter-glyph gap
        assert_eq!(text_width("AB", 1), GLYPH_W * 2 + GLYPH_SPACING);
        assert_eq!(text_width("AB", 2), (GLYPH_W * 2 + GLYPH_SPACING) * 2);
    }
}
