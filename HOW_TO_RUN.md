# Smart Road - How to Run

## Quick Start

### Build the Project
```bash
cd /home/ebourmpo/Desktop/rust
cargo build --release
```

### Run the Simulation
```bash
cargo run --release
```

## Controls During Simulation

### Vehicle Generation
- **Arrow Up** - Generate vehicles coming from South (moving North)
- **Arrow Down** - Generate vehicles coming from North (moving South)
- **Arrow Right** - Generate vehicles coming from West (moving East)
- **Arrow Left** - Generate vehicles coming from East (moving West)

### Continuous Generation
- **R** - Toggle random continuous vehicle generation
  - When enabled, vehicles spawn automatically every 0.5 seconds
  - Routes (right/straight/left) are randomized

### Exit & Statistics
- **Esc** - Exit simulation
  - Closes the window
  - Displays final statistics in the console

## Understanding the Display

### Intersection Visualization
- **Gray square** - The intersection area where vehicles interact
- **Dark gray roads** - Lanes connecting to the intersection
- **Colored rectangles** - Vehicles (different colors for different IDs)

### Vehicle Colors
- Red, Blue, Green, Yellow, Magenta (rotates based on vehicle ID)

### Statistics Output

When you press Esc, you'll see:
```
========== TRAFFIC SIMULATION STATISTICS ==========
Total Vehicles Passed: 42
Max Velocity: 150.00 pixels/frame
Min Velocity: 10.00 pixels/frame
Max Time in Intersection: 5.23 seconds
Min Time in Intersection: 0.15 seconds
Close Calls: 3
===================================================
```

## Smart Traffic Algorithm

The system automatically:
1. **Detects vehicles ahead** - Counts vehicles in the intersection
2. **Adjusts speeds adaptively**:
   - 0 vehicles ahead → 150 px/frame (high speed)
   - 1 vehicle ahead → 100 px/frame (medium speed)
   - 2+ vehicles ahead → 50 px/frame (low speed)
3. **Prevents collisions** - Enforces safety distance of 15 pixels
4. **Tracks close calls** - Records when vehicles get too close

## Example Session

1. Start the program: `cargo run --release`
2. Press **Arrow Up** multiple times to spawn vehicles from the South
3. Press **Arrow Right** to spawn vehicles from the West
4. Watch them navigate the intersection without traffic lights!
5. Press **Esc** to see statistics

## Requirements

- Rust 1.70+
- SDL2 development libraries installed:
  ```bash
  sudo apt-get install libsdl2-dev libsdl2-image-dev
  ```

## Troubleshooting

### "cannot find -lSDL2" error
Install SDL2 dev libraries:
```bash
sudo apt-get install libsdl2-dev
```

### Window doesn't open
Ensure your display is available (X11 or Wayland)

### No vehicles appear
Try pressing the arrow keys or R key to generate them

## Performance

- Runs at 60 FPS
- Resolution: 1400 x 900 pixels
- Supports unlimited vehicles
- O(n²) collision detection
