# Running the Smart Road Traffic Simulation

This project is a Rust-based traffic simulation using SDL2 for graphics. Follow these steps to build and run the program:

## Prerequisites

- **Rust toolchain**: Install from https://rustup.rs/
- **SDL2 library**: You must have the SDL2 development libraries installed on your system.
    - **Windows**: Download SDL2 development libraries from https://www.libsdl.org/download-2.0.php and follow the instructions for your toolchain (e.g., MSVC or MinGW).
    - **Linux**: Install via your package manager, e.g., `sudo apt-get install libsdl2-dev`
    - **macOS**: Install via Homebrew: `brew install sdl2`

## Building the Project

Open a terminal in the project root and run:

```
cargo build --release
```

## Running the Simulation

Run the following command from the project root:

```
cargo run --release
```

## Controls

- **Arrow keys**: Spawn vehicles from North, South, East, or West
- **R**: Toggle random vehicle generation
- **Esc** or close window: Exit simulation

## Output

Simulation statistics will be printed to the terminal after you exit.

---

For any issues with SDL2 setup, consult the [SDL2 crate documentation](https://docs.rs/sdl2/latest/sdl2/) or the official SDL2 website.