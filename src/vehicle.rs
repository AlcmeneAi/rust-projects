use crate::intersection::Direction;
use crate::animation::AnimationState;
use rand::Rng;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Route {
    Right,
    Straight,
    Left,
}

#[derive(Clone, Copy, Debug)]
pub enum VelocityLevel {
    Stopped,        // 0 pixels/frame - waiting for intersection to clear
    Slow,           // 80 pixels/frame - velocity-modulated approach speed
    Normal,         // 160 pixels/frame - standard operating speed
    Fast,           // 260 pixels/frame - free flow speed
    Custom(f32),    // arbitrary px/frame set by ATW target velocity (>= CRAWL_SPEED)
}

impl PartialEq for VelocityLevel {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Stopped,   Self::Stopped)   => true,
            (Self::Slow,      Self::Slow)      => true,
            (Self::Normal,    Self::Normal)    => true,
            (Self::Fast,      Self::Fast)      => true,
            (Self::Custom(a), Self::Custom(b)) => a.to_bits() == b.to_bits(),
            _                                  => false,
        }
    }
}

impl Eq for VelocityLevel {}

impl std::hash::Hash for VelocityLevel {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        std::mem::discriminant(self).hash(state);
        if let Self::Custom(v) = self {
            v.to_bits().hash(state);
        }
    }
}

impl VelocityLevel {
    pub fn to_pixels_per_frame(&self) -> f32 {
        match self {
            VelocityLevel::Stopped    => 0.0,
            VelocityLevel::Slow       => 80.0,
            VelocityLevel::Normal     => 160.0,
            VelocityLevel::Fast       => 260.0,
            VelocityLevel::Custom(v)  => *v,
        }
    }

}

impl Route {
    pub fn random() -> Self {
        let mut rng = rand::thread_rng();
        match rng.gen_range(0..3) {
            0 => Route::Right,
            1 => Route::Straight,
            _ => Route::Left,
        }
    }
}

pub struct Vehicle {
    id: u32,
    position: (f32, f32),
    direction: Direction,
    route: Route,
    velocity: f32,
    velocity_level: VelocityLevel,  // Control level set by intersection
    distance_traveled: f32,
    time_in_intersection: f32,
    entered_intersection: bool,
    exited_intersection: bool,  // Track if we've already recorded the exit
    animation: AnimationState,
    last_direction: Direction,
    route_applied: bool,
    assigned_lane: u32,  // 0 = left, 1 = straight, 2 = right
    
    // Physics tracking for intersection traversal
    intersection_entry_time: f32,  // Time when vehicle entered intersection (seconds)
    intersection_entry_position: (f32, f32),  // Position when vehicle entered
    intersection_exit_time: f32,   // Time when vehicle left intersection (seconds)
    intersection_distance: f32,    // Distance traveled in intersection (pixels)
    intersection_physics_velocity: f32,  // Calculated velocity = distance / time
    distance_at_intersection_entry: f32, // Odometer snapshot at intersection entry
}

impl Vehicle {
    pub fn new(id: u32, direction: Direction, route: Route) -> Self {
        // Determine which lane to assign based on route
        let assigned_lane = match route {
            Route::Left => 0,
            Route::Straight => 1,
            Route::Right => 2,
        };

        // Calculate initial position based on direction and assigned lane
        let (x, y) = Self::calculate_initial_position(direction, assigned_lane);

        // Start with normal velocity level
        let velocity_level = VelocityLevel::Normal;
        let velocity = velocity_level.to_pixels_per_frame();

        Vehicle {
            id,
            position: (x, y),
            direction,
            route,
            velocity,
            velocity_level,
            distance_traveled: 0.0,
            time_in_intersection: 0.0,
            entered_intersection: false,
            exited_intersection: false,
            animation: AnimationState::new(direction),
            last_direction: direction,
            route_applied: false,
            assigned_lane,
            intersection_entry_time: 0.0,
            intersection_entry_position: (x, y),
            intersection_exit_time: 0.0,
            intersection_distance: 0.0,
            intersection_physics_velocity: 0.0,
            distance_at_intersection_entry: 0.0,
        }
    }

    fn calculate_initial_position(direction: Direction, lane: u32) -> (f32, f32) {
        // Intersection center: (700, 450). Road half-width = 150.
        // N/S road: x 550-850. Northbound (right half): center + 25 + lane*50.
        //                        Southbound (left half):  center - 25 - lane*50.
        // E/W road: y 300-600. Eastbound (bottom half):  center + 25 + lane*50.
        //                       Westbound (top half):     center - 25 - lane*50.
        match direction {
            Direction::North => {
                // Northbound: right (east) half of N/S road
                let x = 725.0 + (lane as f32 * 50.0);
                (x, 1200.0)
            }
            Direction::South => {
                // Southbound: left (west) half of N/S road
                let x = 675.0 - (lane as f32 * 50.0);
                (x, -200.0)
            }
            Direction::East => {
                // Eastbound: bottom half of E/W road
                let y = 475.0 + (lane as f32 * 50.0);
                (-200.0, y)
            }
            Direction::West => {
                // Westbound: top half of E/W road
                let y = 425.0 - (lane as f32 * 50.0);
                (1600.0, y)
            }
        }
    }

    pub fn update(&mut self, dt: f32) {
        let pos_before = self.position;
        let distance_this_frame = self.velocity * dt;
        self.distance_traveled += distance_this_frame;

        match self.direction {
            Direction::North => {
                self.position.1 -= self.velocity * dt;
                // Only enforce lane constraints before the route turn — after turning,
                // the vehicle is on a different road and assigned_lane no longer applies.
                if !self.route_applied {
                    self.enforce_lane_constraint_north();
                }
            }
            Direction::South => {
                self.position.1 += self.velocity * dt;
                if !self.route_applied {
                    self.enforce_lane_constraint_south();
                }
            }
            Direction::East => {
                self.position.0 += self.velocity * dt;
                if !self.route_applied {
                    self.enforce_lane_constraint_east();
                }
            }
            Direction::West => {
                self.position.0 -= self.velocity * dt;
                if !self.route_applied {
                    self.enforce_lane_constraint_west();
                }
            }
        }

        // Detect unexpected large jumps (teleports)
        let dx = (self.position.0 - pos_before.0).abs();
        let dy = (self.position.1 - pos_before.1).abs();
        let max_expected = self.velocity * dt + 1.0; // +1 for fp rounding
        if dx > max_expected || dy > max_expected {
            eprintln!(
                "[JUMP] id={} dir={:?} route_applied={} ({:.1},{:.1})->({:.1},{:.1}) delta=({:.1},{:.1}) max={:.1}",
                self.id, self.direction, self.route_applied,
                pos_before.0, pos_before.1,
                self.position.0, self.position.1,
                dx, dy, max_expected
            );
        }

        // Update animation if direction changed
        if self.direction != self.last_direction {
            self.animation.set_direction(self.direction);
            self.last_direction = self.direction;
        }

        // Update animation state
        self.animation.update(dt);

        if self.entered_intersection {
            self.time_in_intersection += dt;
        }
    }

    fn enforce_lane_constraint_north(&mut self) {
        // Northbound: right (east) half of N/S road — x = 725, 775, 825
        let target_x = 725.0 + (self.assigned_lane as f32 * 50.0);
        let tolerance = 10.0;
        if (self.position.0 - target_x).abs() > tolerance {
            eprintln!(
                "[LANE-SNAP N] id={} lane={} x: {:.1} -> {:.1} (off {:.1})",
                self.id, self.assigned_lane, self.position.0, target_x,
                self.position.0 - target_x
            );
            self.position.0 = target_x;
        }
    }

    fn enforce_lane_constraint_south(&mut self) {
        // Southbound: left (west) half of N/S road — x = 675, 625, 575
        let target_x = 675.0 - (self.assigned_lane as f32 * 50.0);
        let tolerance = 10.0;
        if (self.position.0 - target_x).abs() > tolerance {
            eprintln!(
                "[LANE-SNAP S] id={} lane={} x: {:.1} -> {:.1} (off {:.1})",
                self.id, self.assigned_lane, self.position.0, target_x,
                self.position.0 - target_x
            );
            self.position.0 = target_x;
        }
    }

    fn enforce_lane_constraint_east(&mut self) {
        // Eastbound: bottom half of E/W road — y = 475, 525, 575
        let target_y = 475.0 + (self.assigned_lane as f32 * 50.0);
        let tolerance = 10.0;
        if (self.position.1 - target_y).abs() > tolerance {
            eprintln!(
                "[LANE-SNAP E] id={} lane={} y: {:.1} -> {:.1} (off {:.1})",
                self.id, self.assigned_lane, self.position.1, target_y,
                self.position.1 - target_y
            );
            self.position.1 = target_y;
        }
    }

    fn enforce_lane_constraint_west(&mut self) {
        // Westbound: top half of E/W road — y = 425, 375, 325
        let target_y = 425.0 - (self.assigned_lane as f32 * 50.0);
        let tolerance = 10.0;
        if (self.position.1 - target_y).abs() > tolerance {
            eprintln!(
                "[LANE-SNAP W] id={} lane={} y: {:.1} -> {:.1} (off {:.1})",
                self.id, self.assigned_lane, self.position.1, target_y,
                self.position.1 - target_y
            );
            self.position.1 = target_y;
        }
    }

    pub fn get_position(&self) -> (f32, f32) {
        self.position
    }

    pub fn get_direction(&self) -> Direction {
        self.direction
    }

    pub fn get_route(&self) -> Route {
        self.route
    }

    /// Permanently change this vehicle's intended route and remap its assigned lane.
    /// The transverse axis position is snapped to the new lane centre immediately so
    /// the per-frame lane-enforcement logic sees no large discontinuity.
    pub fn set_route(&mut self, route: Route) {
        self.route = route;
        self.assigned_lane = match route {
            Route::Left     => 0,
            Route::Straight => 1,
            Route::Right    => 2,
        };
        // Snap transverse position to new lane centre to prevent [LANE-SNAP] spam.
        match self.direction {
            Direction::North => self.position.0 = 725.0 + (self.assigned_lane as f32 * 50.0),
            Direction::South => self.position.0 = 675.0 - (self.assigned_lane as f32 * 50.0),
            Direction::East  => self.position.1 = 475.0 + (self.assigned_lane as f32 * 50.0),
            Direction::West  => self.position.1 = 425.0 - (self.assigned_lane as f32 * 50.0),
        }
    }

    pub fn is_route_applied(&self) -> bool {
        self.route_applied
    }

    pub fn get_velocity(&self) -> f32 {
        self.velocity
    }

    pub fn get_velocity_level(&self) -> VelocityLevel {
        self.velocity_level
    }

    /// Set velocity by controlling the velocity level (Slow, Normal, Fast)
    /// This is the primary way the smart intersection controls vehicle velocity
    pub fn set_velocity_level(&mut self, level: VelocityLevel) {
        self.velocity_level = level;
        self.velocity = level.to_pixels_per_frame();
    }

    /// Set a continuous modulated velocity (used by ATW target-velocity strategy).
    /// Stores VelocityLevel::Custom(vel) so the exact value is observable by the
    /// renderer and collision passes; fixed-level sentinel values are preserved.
    pub fn set_modulated_velocity(&mut self, vel: f32) {
        let v = vel.max(0.0);
        self.velocity = v;
        self.velocity_level = if v == 0.0 {
            VelocityLevel::Stopped
        } else if v == VelocityLevel::Slow.to_pixels_per_frame() {
            VelocityLevel::Slow
        } else if v == VelocityLevel::Normal.to_pixels_per_frame() {
            VelocityLevel::Normal
        } else if v >= VelocityLevel::Fast.to_pixels_per_frame() {
            VelocityLevel::Fast
        } else {
            VelocityLevel::Custom(v)
        };
    }

    pub fn get_time_in_intersection(&self) -> f32 {
        self.time_in_intersection
    }

    pub fn get_id(&self) -> u32 {
        self.id
    }

    pub fn get_animation_angle(&self) -> f32 {
        self.animation.get_angle()
    }

    pub fn get_assigned_lane(&self) -> u32 {
        self.assigned_lane
    }

    /// Returns true when this vehicle should execute its route turn (or, for Straight,
    /// mark `route_applied`) this frame.
    ///
    /// * `intersection_center`    — center of the intersection box.
    /// * `intersection_half_size` — half-side-length of the intersection box (i.e. `size`
    ///   in `Intersection::new`).  Used to place the Straight trigger at the **exit
    ///   boundary** of the intersection instead of the centre, so the ATW collision
    ///   system stops managing the vehicle only after it has physically left the box.
    ///
    /// For Right / Left routes the trigger fires at the **perpendicular snap target**
    /// (the exit-lane centre-line position), which is always inside the intersection.
    /// A tiny margin `M` handles one-frame overshoot without introducing large snaps.
    ///
    /// Snap targets used by `apply_route_turn` (derived from lane geometry):
    ///   North+Right→East  y=575   North+Left→West  y=425
    ///   South+Left→East   y=475   South+Right→West y=325
    ///   East+Right→South  x=575   East+Left→North  x=725
    ///   West+Left→South   x=675   West+Right→North x=825
    pub fn should_apply_route_turn(
        &self,
        intersection_center: (f32, f32),
        intersection_half_size: f32,
    ) -> bool {
        if self.route_applied {
            return false;
        }
        // Tiny margin (≈ one frame of travel at Normal speed) handles floating-point
        // overshoot so a vehicle that crosses the trigger line by a pixel still fires.
        const M: f32 = 5.0;
        match (self.direction, self.route) {
            // ── Northbound (y decreasing) ────────────────────────────────────────
            (Direction::North, Route::Right)    => self.position.1 <= 575.0 + M,
            (Direction::North, Route::Left)     => self.position.1 <= 425.0 + M,
            // Straight: fire at the north (exit) boundary of the intersection
            (Direction::North, Route::Straight) =>
                self.position.1 <= intersection_center.1 - intersection_half_size,

            // ── Southbound (y increasing) ────────────────────────────────────────
            (Direction::South, Route::Left)     => self.position.1 >= 475.0 - M,
            (Direction::South, Route::Right)    => self.position.1 >= 325.0 - M,
            // Straight: fire at the south (exit) boundary
            (Direction::South, Route::Straight) =>
                self.position.1 >= intersection_center.1 + intersection_half_size,

            // ── Eastbound (x increasing) ─────────────────────────────────────────
            (Direction::East, Route::Left)      => self.position.0 >= 725.0 - M,
            (Direction::East, Route::Right)     => self.position.0 >= 575.0 - M,
            // Straight: fire at the east (exit) boundary
            (Direction::East, Route::Straight)  =>
                self.position.0 >= intersection_center.0 + intersection_half_size,

            // ── Westbound (x decreasing) ─────────────────────────────────────────
            (Direction::West, Route::Left)      => self.position.0 <= 675.0 + M,
            (Direction::West, Route::Right)     => self.position.0 <= 825.0 + M,
            // Straight: fire at the west (exit) boundary
            (Direction::West, Route::Straight)  =>
                self.position.0 <= intersection_center.0 - intersection_half_size,
        }
    }

    pub fn apply_route_turn(&mut self) {
        if self.route_applied {
            return;
        }

        self.route_applied = true;

        eprintln!(
            "[TURN] id={} dir={:?} route={:?} pos=({:.1},{:.1}) BEFORE TURN",
            self.id, self.direction, self.route, self.position.0, self.position.1
        );

        self.direction = match (self.direction, self.route) {
            // From North
            (Direction::North, Route::Left) => Direction::West,
            (Direction::North, Route::Straight) => Direction::North,
            (Direction::North, Route::Right) => Direction::East,
            
            // From South
            (Direction::South, Route::Left) => Direction::East,
            (Direction::South, Route::Straight) => Direction::South,
            (Direction::South, Route::Right) => Direction::West,
            
            // From East
            (Direction::East, Route::Left) => Direction::North,
            (Direction::East, Route::Straight) => Direction::East,
            (Direction::East, Route::Right) => Direction::South,
            
            // From West
            (Direction::West, Route::Left) => Direction::South,
            (Direction::West, Route::Straight) => Direction::West,
            (Direction::West, Route::Right) => Direction::North,
        };

        // Snap the perpendicular coordinate to the correct exit lane.
        // Lane positions (using intersection center 700,450, lane width 50):
        //   Northbound: x = 725 + lane*50  (lane 0=Left,1=Straight,2=Right)
        //   Southbound: x = 675 - lane*50
        //   Eastbound:  y = 475 + lane*50
        //   Westbound:  y = 425 - lane*50
        // Right turns enter lane 2 (outer), Left turns enter lane 0 (inner).
        let pos_before_snap = self.position;
        match (self.route, self.direction) {
            (Route::Right, Direction::East)  => self.position.1 = 575.0, // ex-North, eastbound outer
            (Route::Right, Direction::West)  => self.position.1 = 325.0, // ex-South, westbound outer
            (Route::Right, Direction::South) => self.position.0 = 575.0, // ex-East,  southbound outer
            (Route::Right, Direction::North) => self.position.0 = 825.0, // ex-West,  northbound outer
            (Route::Left,  Direction::West)  => self.position.1 = 425.0, // ex-North, westbound inner
            (Route::Left,  Direction::East)  => self.position.1 = 475.0, // ex-South, eastbound inner
            (Route::Left,  Direction::North) => self.position.0 = 725.0, // ex-East,  northbound inner
            (Route::Left,  Direction::South) => self.position.0 = 675.0, // ex-West,  southbound inner
            _ => {}
        }
        eprintln!(
            "[TURN] id={} new_dir={:?} snap ({:.1},{:.1})->({:.1},{:.1})",
            self.id, self.direction,
            pos_before_snap.0, pos_before_snap.1,
            self.position.0, self.position.1
        );
    }

    // ==================== Physics Tracking Methods ====================

    /// Mark the moment a vehicle enters the intersection
    pub fn mark_intersection_entry(&mut self, current_time: f32) {
        if !self.entered_intersection {
            self.entered_intersection = true;
            self.intersection_entry_time = current_time;
            self.intersection_entry_position = self.position;
            self.distance_at_intersection_entry = self.distance_traveled;
        }
    }

    /// Mark the moment a vehicle exits the intersection
    pub fn mark_intersection_exit(&mut self, current_time: f32) {
        if self.entered_intersection && !self.exited_intersection {
            self.intersection_exit_time = current_time;
            self.exited_intersection = true;
            self.update_intersection_physics();
        }
    }

    /// Update physics calculations for the intersection traversal
    fn update_intersection_physics(&mut self) {
        // Calculate distance traveled through the intersection using the odometer
        // (cumulative path distance) rather than straight-line entry→exit so that
        // curved left/right turns are measured correctly.
        self.intersection_distance =
            (self.distance_traveled - self.distance_at_intersection_entry).max(0.0);

        // Calculate time spent in intersection
        let time_in_intersection = self.intersection_exit_time - self.intersection_entry_time;

        // Calculate velocity using physics: velocity = distance / time
        if time_in_intersection > 0.0 {
            self.intersection_physics_velocity = self.intersection_distance / time_in_intersection;
        } else {
            self.intersection_physics_velocity = 0.0;
        }
    }

    /// Get the calculated physics velocity (distance / time) in pixels per second
    pub fn get_physics_velocity(&self) -> f32 {
        self.intersection_physics_velocity
    }

    /// Check if vehicle has entered intersection
    pub fn has_entered_intersection(&self) -> bool {
        self.entered_intersection
    }

    /// Check if vehicle has exited intersection (has complete physics data)
    pub fn has_exited_intersection(&self) -> bool {
        self.entered_intersection && self.intersection_exit_time > 0.0
    }

    #[cfg(test)]
    pub fn set_position(&mut self, x: f32, y: f32) {
        self.position = (x, y);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::intersection::Direction;

    #[test]
    fn set_position_changes_vehicle_position() {
        let mut v = Vehicle::new(0, Direction::North, Route::Straight);
        v.set_position(100.0, 200.0);
        assert_eq!(v.get_position(), (100.0, 200.0));
    }
}
