# Smart Road - Traffic Simulation

A Rust-based autonomous vehicle traffic simulation system for optimizing intersection management without traditional traffic lights.

## Overview

Smart Road is a traffic control strategy simulation that demonstrates how autonomous vehicles (AVs) can safely and efficiently pass through an intersection without collisions and with minimal traffic congestion. This project implements a smart intersection management algorithm designed specifically for self-driving cars.

## Features

### Core Components

- **Vehicle Physics Engine**: Implements realistic movement with velocity, distance, and time calculations
  - Three velocity levels (high, medium, low) for traffic control
  - Safety distance enforcement between vehicles
  - Collision detection and avoidance

- **Smart Intersection Management**: Algorithm that optimizes traffic flow
  - Velocity-based traffic control
  - Real-time vehicle count analysis
  - Adaptive speed adjustment based on intersection occupancy

- **Cross-Intersection Model**: 4-way intersection with lane-based routing
  - Multiple lanes per direction
  - Three routing options per lane: right (r), straight (s), left (l)
  - Autonomous path planning

- **Collision Detection**: Safety system with close-call tracking
  - Real-time distance monitoring between vehicles
  - Automatic velocity reduction on proximity
  - Close-call statistics recording

### Controls

- **Arrow Up** - Generate vehicles from South to North
- **Arrow Down** - Generate vehicles from North to South
- **Arrow Right** - Generate vehicles from West to East
- **Arrow Left** - Generate vehicles from East to West
- **R** - Toggle continuous random vehicle generation
- **Esc** - Exit simulation and display statistics

### Statistics

After the simulation ends, the system displays:

- **Total vehicles passed**: Maximum number of vehicles that successfully traversed the intersection
- **Max velocity**: Fastest speed achieved by any vehicle (pixels/frame)
- **Min velocity**: Slowest speed recorded (pixels/frame)
- **Max time**: Longest time any vehicle spent in the intersection (seconds)
- **Min time**: Shortest time any vehicle spent in the intersection (seconds)
- **Close calls**: Number of safety distance violations detected

## Project Structure

```
smart_road/
├── Cargo.toml              # Project dependencies and metadata
├── src/
│   ├── main.rs            # Main simulation loop and traffic control algorithm
│   ├── vehicle.rs         # Vehicle struct and physics implementation
│   ├── intersection.rs     # Intersection model and lane management
│   ├── renderer.rs        # Graphics rendering using SDL2
│   ├── input.rs           # Keyboard input handling
│   ├── physics.rs         # Physics calculations (velocity, distance, time)
│   └── statistics.rs      # Statistics tracking and recording
├── .gitignore
└── README.md
```

## Technical Details

### Vehicle Physics

The simulation uses the fundamental physics equation:

$$v = \frac{d}{t}$$

Where:
- **v** = velocity (pixels per frame)
- **d** = distance traveled
- **t** = time elapsed

Each vehicle maintains:
- Position (x, y coordinates)
- Direction (North, South, East, West)
- Route (Right turn, Straight, Left turn)
- Current velocity (10-200 pixels/frame)
- Distance traveled and time in intersection

### Smart Intersection Algorithm

The algorithm manages traffic by adjusting vehicle velocities based on intersection occupancy:

1. **Vehicle Counting**: Count vehicles ahead in the intersection
2. **Velocity Assignment**:
   - 0 vehicles ahead → 150 pixels/frame (high speed)
   - 1 vehicle ahead → 100 pixels/frame (medium speed)
   - 2+ vehicles ahead → 50 pixels/frame (low speed)

3. **Collision Avoidance**: When vehicles get closer than safety distance (15.0 pixels), apply braking (reduce velocity to 50%)

4. **Approach Phase**: Vehicles approaching the intersection use medium speed (100 pixels/frame)

## Dependencies

- **sdl2** (0.35) - Graphics rendering and window management
- **rand** (0.8) - Random vehicle generation
- **nalgebra** (0.33) - Vector and matrix mathematics

## Building and Running

### Prerequisites

- Rust 1.70+ (install from https://rustup.rs/)
- SDL2 development libraries:
  ```bash
  # Ubuntu/Debian
  sudo apt-get install libsdl2-dev libsdl2-image-dev libsdl2-ttf-dev
  
  # macOS
  brew install sdl2 sdl2_image sdl2_ttf
  
  # Fedora/RHEL
  sudo dnf install SDL2-devel SDL2_image-devel SDL2_ttf-devel
  ```

### Build

```bash
cargo build --release
```

### Run

```bash
cargo run --release
```

## How It Works

1. **Initialization**: Creates a 4-way intersection with 4 lanes (one per direction)
2. **Vehicle Generation**: Users can spawn vehicles from any direction using arrow keys
3. **Simulation Loop**: 
   - Updates all vehicle positions based on velocity
   - Checks for collisions and safety violations
   - Applies smart intersection algorithm to adjust velocities
   - Renders the current state at 60 FPS
4. **Statistics Collection**: Tracks all vehicles and their metrics as they leave the intersection
5. **Display Results**: Shows final statistics when simulation ends (Esc key)

## Smart Intersection Benefits

### Advantages Over Traditional Traffic Lights

- **No waiting**: Vehicles continuously move through the intersection
- **Adaptive flow**: Speed adjusts in real-time to traffic conditions
- **Reduced congestion**: Optimal spacing between vehicles
- **Zero emissions idle time**: No vehicles stopped at red lights
- **Scalability**: Algorithm works with any number of vehicles

### Safety Features

- Mandatory safety distance enforcement (15 pixels minimum)
- Real-time collision detection
- Automatic velocity reduction on proximity
- Close-call tracking for analysis

## Performance Metrics

The simulation runs at 60 FPS with the following typical metrics:

- **Vehicle Processing**: O(n²) collision detection for n vehicles
- **Memory**: ~1KB per vehicle
- **Rendering**: Real-time graphics at 1400x900 resolution

## Future Enhancements

- [ ] Sprite-based vehicle rendering with rotation animation
- [ ] Multiple intersection types and configurations
- [ ] Machine learning-based velocity prediction
- [ ] Traffic flow optimization algorithms
- [ ] Network simulation with multiple intersections
- [ ] Vehicle priority system (emergency vehicles)
- [ ] Lane-changing algorithms
- [ ] Real-world traffic data integration

## Testing

Run the physics module tests:

```bash
cargo test --lib physics
```

## License

This project is part of the Rust Piscine educational program.

## Author

Created as a continuation of the road_intersection project, implementing smart autonomous vehicle traffic control.

## References

- National Highway Traffic Safety Administration (NHTSA) study on intersection-related crashes
- Autonomous vehicle traffic control research
- Traffic flow optimization algorithms
- SDL2 Graphics Library Documentation
