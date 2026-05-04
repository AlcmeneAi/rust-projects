use crate::vehicle::{Vehicle, VelocityLevel, Route};
use crate::intersection::{Intersection, Direction};
use crate::statistics::Statistics;
use std::collections::{HashSet, HashMap};
use rayon::prelude::*;

// ── Spatial grid for O(1) neighbour lookup in Passes 2 & 4 ─────────────────
struct SpatialGrid {
    cells: Vec<Vec<usize>>,
    cell_size: f32,
    cols: usize,
    rows: usize,
}

impl SpatialGrid {
    fn build(positions: &[(f32, f32)], cell_size: f32, world_w: f32, world_h: f32) -> Self {
        let cols = ((world_w / cell_size).ceil() as usize).max(1);
        let rows = ((world_h / cell_size).ceil() as usize).max(1);
        let mut grid = SpatialGrid { cells: vec![Vec::new(); cols * rows], cell_size, cols, rows };
        for (idx, &pos) in positions.iter().enumerate() {
            let col = ((pos.0 / cell_size) as usize).min(cols - 1);
            let row = ((pos.1 / cell_size) as usize).min(rows - 1);
            grid.cells[row * cols + col].push(idx);
        }
        grid
    }

    /// All vehicle indices in the same and immediately neighbouring cells (1-cell radius).
    fn candidates_near(&self, pos: (f32, f32)) -> Vec<usize> {
        let col = ((pos.0 / self.cell_size) as i32).clamp(0, self.cols as i32 - 1);
        let row = ((pos.1 / self.cell_size) as i32).clamp(0, self.rows as i32 - 1);
        let mut out = Vec::new();
        for dr in -1i32..=1 {
            for dc in -1i32..=1 {
                let c2 = col + dc;
                let r2 = row + dr;
                if c2 >= 0 && c2 < self.cols as i32 && r2 >= 0 && r2 < self.rows as i32 {
                    out.extend_from_slice(
                        &self.cells[(r2 as usize) * self.cols + c2 as usize],
                    );
                }
            }
        }
        out
    }
}

const SAFETY_DISTANCE_FAST: f32 = 120.0;  // same-lane following: beyond this → Fast
const SAFETY_DISTANCE_SLOW: f32 = 80.0;   // same-lane following: below this  → Slow
const SAFETY_DISTANCE_STOP: f32 = 50.0;   // same-lane following: below this  → Stopped (~vehicle body)
// Planning horizon for the ATW algorithm (pixels beyond intersection boundary).
const ETA_LOOKAHEAD: f32 = 1000.0;
// Safety buffer added to both ends of each vehicle's crossing time window (seconds).
const WINDOW_BUFFER_SECS: f32 = 0.3;
// Stop-line distance: set to 0.0 to disable the physical stop line — ATW target-velocity
// modulation is the sole speed-control mechanism; vehicles never hard-stop at a line.
#[allow(dead_code)]
const STOP_LINE_DISTANCE: f32 = 0.0;
// Near zone used for close-call counting in Pass 2.
const INTERSECTION_PADDING: f32 = 50.0;
// Half-width of the vehicle sprite used as the hard-collision radius (px).
// Vehicles whose centres are closer than this are considered to have physically collided.
const VEHICLE_COLLISION_RADIUS: f32 = 25.0;
// Minimum speed enforced by ATW: vehicles always keep moving at at least this rate
// so they never fully halt due to a computed target velocity (Pass 3 still stops them
// on hard physical contact via SAFETY_DISTANCE_STOP).
const CRAWL_SPEED: f32 = 20.0;

pub fn check_collisions_and_apply_strategy(
    vehicles: &mut Vec<Vehicle>,
    intersection: &Intersection,
    statistics: &mut Statistics,
) {
    let n = vehicles.len();
    if n == 0 { return; }
    let fast_vel = VelocityLevel::Fast.to_pixels_per_frame();

    // ── Snapshot positions once — avoids repeated getter calls across all passes.
    let positions: Vec<(f32, f32)> = vehicles.iter().map(|v| v.get_position()).collect();

    // Snapshot velocity levels before any pass modifies them.
    // Pass 3 hysteresis compares against this snapshot, not against whatever
    // Pass 1 just wrote, so the two passes are independent.
    let prev_levels: Vec<VelocityLevel> = vehicles.iter().map(|v| v.get_velocity_level()).collect();

    // ── Precompute time windows at current velocity ────────────────────────
    // Cached entry times used as the Pass 1 priority sort key and reused in
    // Pass 2 conflict detection — avoids calling compute_time_window twice.
    let precomp_windows: Vec<(f32, f32)> = vehicles.iter().zip(positions.iter()).map(|(v, &pos)| {
        if v.is_route_applied() { return (f32::MAX, f32::MAX); }
        if !is_in_or_near_intersection(pos, intersection, ETA_LOOKAHEAD) { return (f32::MAX, f32::MAX); }
        let vel = v.get_velocity();
        if vel <= 0.0 { return (f32::MAX, f32::MAX); }
        compute_time_window(pos, v.get_direction(), v.get_route(), vel,
                            intersection.contains_point(pos), intersection)
    }).collect();

    // ─────────────────────────────────────────────────────────────────────────
    // Pass 1 — Arrival-Time-Window (ATW) velocity modulation
    //
    // Each approaching vehicle is assigned a target velocity such that it
    // arrives at the intersection entry only *after* all conflicting higher-
    // priority vehicles have cleared the box.
    //
    // Priority metric: earliest projected t_entry (first-come-first-served).
    // Tie-breaking: lower vehicle ID wins (older vehicle has priority).
    //
    // For each vehicle processed in priority order:
    //   1. Find the latest committed exit time among already-scheduled
    //      conflicting vehicles (the "blocking exit time").
    //   2. Compute the target velocity that makes the vehicle arrive at the
    //      entry edge exactly after all blockers have cleared:
    //
    //         target = dist_to_entry / (blocking_exit_t + WINDOW_BUFFER_SECS)
    //         target = clamp(target, CRAWL_SPEED, Fast)
    //
    //   3. Apply via set_modulated_velocity — a continuous value, never a
    //      coarse step.  Vehicles never fully stop due to ATW alone.
    //   4. Record the vehicle's committed window at the assigned velocity so
    //      subsequent (lower-priority) vehicles see an accurate clearing time.
    //
    // Predictive braking is emergent: as a vehicle closes on the intersection
    // with an active blocker, dist shrinks each frame while the blocker exit
    // time stays roughly constant → target velocity decreases smoothly,
    // producing a continuous deceleration curve with no abrupt stop/go.
    // ─────────────────────────────────────────────────────────────────────────

    // 1a. Sort vehicle indices by ascending precomputed t_entry.
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&a, &b| {
        precomp_windows[a].0
            .partial_cmp(&precomp_windows[b].0)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| vehicles[a].get_id().cmp(&vehicles[b].get_id()))
    });

    // committed[i] = the intersection-occupancy window guaranteed for vehicle i
    // after its ATW velocity has been assigned in this frame.
    let mut committed: Vec<Option<(f32, f32)>> = vec![None; n];

    // 1c. Process vehicles in priority order (highest first).
    for &i in &order {
        let pos_i = positions[i];

        // Exited or outside planning horizon → free-running at Fast.
        if vehicles[i].is_route_applied()
            || !is_in_or_near_intersection(pos_i, intersection, ETA_LOOKAHEAD)
        {
            vehicles[i].set_velocity_level(VelocityLevel::Fast);
            continue;
        }

        let dir_i   = vehicles[i].get_direction();
        let route_i = vehicles[i].get_route();

        // Inside the intersection: absolute highest priority, hold at Normal.
        if intersection.contains_point(pos_i) {
            vehicles[i].set_velocity_level(VelocityLevel::Normal);
            let v = VelocityLevel::Normal.to_pixels_per_frame();
            committed[i] = Some(compute_time_window(pos_i, dir_i, route_i, v, true, intersection));
            continue;
        }

        // Approaching: find the latest committed exit time of any conflicting peer.
        let dist_i = dist_to_entry_edge(pos_i, dir_i, intersection);
        let mut blocking_exit_t = f32::NEG_INFINITY;
        for j in 0..n {
            if i == j { continue; }
            let Some(w_j) = committed[j] else { continue };
            if !will_paths_conflict(dir_i, route_i, vehicles[j].get_direction(), vehicles[j].get_route()) {
                continue;
            }
            if w_j.1 > blocking_exit_t {
                blocking_exit_t = w_j.1;
            }
        }

        // Derive target velocity from the blocking window.
        let target_vel = if blocking_exit_t <= 0.0 {
            // No active blocker, or all conflicts already cleared → full speed.
            fast_vel
        } else if dist_i <= 0.0 {
            // Right at the entry edge with a live blocker → crawl.
            CRAWL_SPEED
        } else {
            // Arrive at the entry edge only after blocking_exit_t + buffer.
            let raw = dist_i / (blocking_exit_t + WINDOW_BUFFER_SECS);
            raw.clamp(CRAWL_SPEED, fast_vel)
        };

        vehicles[i].set_modulated_velocity(target_vel);

        // Commit the window at the just-assigned velocity so lower-priority
        // vehicles downstream see this vehicle's realistic clearing time.
        committed[i] = Some(compute_time_window(pos_i, dir_i, route_i, target_vel, false, intersection));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Pass 2 — Close-call detection  (spatially pre-filtered + parallel)
    //
    // When two vehicles with crossing paths come within SAFETY_DISTANCE_STOP
    // of each other without actually colliding, record a close call.
    // No velocity modification here — all speed decisions belong to Pass 1.
    //
    // Optimisation: only vehicles that are already near the intersection box
    // can produce close calls.  We pre-filter once, then run O(k²) pair checks
    // over that small subset (k << n) in parallel via rayon.
    // ─────────────────────────────────────────────────────────────────────────
    let near_isect: Vec<usize> = (0..n)
        .filter(|&i| {
            !vehicles[i].is_route_applied()
                && is_in_or_near_intersection(positions[i], intersection, INTERSECTION_PADDING)
        })
        .collect();

    // Parallel pair detection — read-only; writes happen after the join.
    // We reborrow vehicles/positions as immutable slices so the fat-pointer
    // copies inside move closures satisfy rayon's Fn + Send + Sync bounds.
    let current_close_call_pairs: HashSet<(u32, u32)> = {
        let vslice = vehicles.as_slice();
        let pslice = positions.as_slice();
        near_isect
            .par_iter()
            .flat_map(|&i| {
                let p_i  = pslice[i];
                let di   = vslice[i].get_direction();
                let ri   = vslice[i].get_route();
                let id_i = vslice[i].get_id();
                near_isect
                    .iter()
                    .filter(move |&&j| {
                        if j <= i { return false; }
                        let p_j = pslice[j];
                        let dx = p_i.0 - p_j.0;
                        let dy = p_i.1 - p_j.1;
                        let dist_sq = dx * dx + dy * dy;
                        dist_sq < SAFETY_DISTANCE_STOP * SAFETY_DISTANCE_STOP
                            && dist_sq >= VEHICLE_COLLISION_RADIUS * VEHICLE_COLLISION_RADIUS
                            && will_paths_conflict(di, ri, vslice[j].get_direction(), vslice[j].get_route())
                    })
                    .map(move |&j| {
                        let id_j = vslice[j].get_id();
                        (id_i.min(id_j), id_i.max(id_j))
                    })
                    .collect::<Vec<_>>()
            })
            .collect()
    };

    // Log newly-activated pairs (serial, infrequent).
    for &pair in &current_close_call_pairs {
        if !statistics.is_active_close_call(&pair) {
            let p_a = positions[vehicles.iter().position(|v| v.get_id() == pair.0).unwrap_or(0)];
            let p_b = positions[vehicles.iter().position(|v| v.get_id() == pair.1).unwrap_or(0)];
            let dx = p_a.0 - p_b.0; let dy = p_a.1 - p_b.1;
            eprintln!(
                "[CLOSE CALL] id={} & id={} dist={:.1} pos_a=({:.0},{:.0}) pos_b=({:.0},{:.0})",
                pair.0, pair.1, (dx*dx+dy*dy).sqrt(), p_a.0, p_a.1, p_b.0, p_b.1
            );
        }
    }
    statistics.update_close_calls(&current_close_call_pairs);

    // ─────────────────────────────────────────────────────────────────────────
    // Pass 3 — Same-lane safety distance  (lane-grouped, O(n) + O(kg²))
    //
    // Enforce minimum following distances within same-lane queues.  ATW handles
    // intersection timing; this pass catches rear-end scenarios where a follower
    // closes on a leader before the intersection.
    //
    // Optimisation: group vehicles by (direction, lane) first.  Only pairs in
    // the same group can conflict — this reduces comparisons from O(n²) to
    // O(n + Σ kg²) where kg is the typical group size (≈ n / 12 for 4 dirs ×
    // 3 lanes).  For each follower we track the closest ahead leader so the
    // closest-leader constraint is respected exactly.
    // ─────────────────────────────────────────────────────────────────────────
    let mut lane_groups: HashMap<(Direction, u32), Vec<usize>> = HashMap::new();
    for i in 0..n {
        lane_groups
            .entry((vehicles[i].get_direction(), vehicles[i].get_assigned_lane()))
            .or_default()
            .push(i);
    }

    // Collect updates first to avoid conflicting mutable borrows while iterating.
    let mut p3_updates: Vec<(usize, VelocityLevel)> = Vec::new();
    for (_, group) in &lane_groups {
        for &i in group {
            let pos_i = positions[i];
            let dir_i = vehicles[i].get_direction();
            // Find the closest ahead vehicle in this lane group.
            let mut min_dist  = f32::MAX;
            let mut best_dist = f32::MAX;
            for &j in group {
                if i == j { continue; }
                let pos_j = positions[j];
                let is_ahead = match dir_i {
                    Direction::North => pos_j.1 < pos_i.1,
                    Direction::South => pos_j.1 > pos_i.1,
                    Direction::East  => pos_j.0 > pos_i.0,
                    Direction::West  => pos_j.0 < pos_i.0,
                };
                if !is_ahead { continue; }
                let dx = pos_i.0 - pos_j.0;
                let dy = pos_i.1 - pos_j.1;
                let d  = (dx * dx + dy * dy).sqrt();
                if d < min_dist { min_dist = d; best_dist = d; }
            }
            if best_dist > SAFETY_DISTANCE_FAST { continue; }
            let distance = best_dist;
            let cur = prev_levels[i];
            // Hysteresis bands (Fast→hysteresis extends upward by 5 px):
            //   < STOP  (50)  → Stopped
            //   < SLOW  (80)  → Slow
            //   < FAST (120)  → Normal   (with extra hysteresis when coming from Fast)
            //   otherwise     → keep current
            let fast_hyst = if cur == VelocityLevel::Fast { 5.0 } else { 0.0 };
            let new_level = if distance < SAFETY_DISTANCE_STOP {
                VelocityLevel::Stopped
            } else if distance < SAFETY_DISTANCE_SLOW {
                VelocityLevel::Slow
            } else if distance < SAFETY_DISTANCE_FAST - fast_hyst {
                VelocityLevel::Normal
            } else {
                cur
            };
            // Always write when a closer leader is in range — this overrides
            // whatever Pass 1 set (e.g. Fast → Normal for queue followers).
            if new_level != prev_levels[i] {
                eprintln!(
                    "[COL P3] id={} dir={:?} lane={} closest_ahead dist={:.1} {:?}->{:?}",
                    vehicles[i].get_id(), dir_i, vehicles[i].get_assigned_lane(),
                    distance, prev_levels[i], new_level
                );
            }
            p3_updates.push((i, new_level));
        }
    }
    for (i, level) in p3_updates {
        vehicles[i].set_velocity_level(level);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Pass 4 — Hard collision detection  (spatial grid + parallel)
    //
    // If two vehicle centres are closer than VEHICLE_COLLISION_RADIUS they have
    // physically overlapped — ATW failed to keep them apart.  Hard-stop both
    // vehicles immediately and record the event in statistics.
    //
    // Optimisation: build a uniform spatial grid (cell size = 2 × collision
    // radius).  Each vehicle only checks its 3×3 neighbourhood — O(1) per
    // vehicle, O(n) total.  Pair detection runs in parallel via rayon;
    // hard-stop writes happen serially after the join to avoid data races.
    // ─────────────────────────────────────────────────────────────────────────
    const WORLD_W: f32 = 1400.0;
    const WORLD_H: f32 = 900.0;
    let grid = SpatialGrid::build(&positions, VEHICLE_COLLISION_RADIUS * 2.0, WORLD_W, WORLD_H);

    // Parallel detection — only reads positions and vehicle IDs.
    // Reborrow as immutable slices so fat-pointer copies in move closures work.
    let collision_pairs: HashSet<(u32, u32)> = {
        let vslice = vehicles.as_slice();
        let pslice = positions.as_slice();
        (0..n)
            .into_par_iter()
            .flat_map(|i| {
                let pos_i = pslice[i];
                let id_i  = vslice[i].get_id();
                let r2    = VEHICLE_COLLISION_RADIUS * VEHICLE_COLLISION_RADIUS;
                grid.candidates_near(pos_i)
                    .into_iter()
                    .filter(move |&j| {
                        if j <= i { return false; }
                        let pos_j = pslice[j];
                        let dx = pos_i.0 - pos_j.0;
                        let dy = pos_i.1 - pos_j.1;
                        dx * dx + dy * dy < r2
                    })
                    .map(move |j| {
                        let id_j = vslice[j].get_id();
                        (id_i.min(id_j), id_i.max(id_j))
                    })
                    .collect::<Vec<_>>()
            })
            .collect()
    };

    // Serial write phase — log + hard-stop.
    for &pair in &collision_pairs {
        if !statistics.is_active_collision(&pair) {
            let p_a = positions[vehicles.iter().position(|v| v.get_id() == pair.0).unwrap_or(0)];
            let p_b = positions[vehicles.iter().position(|v| v.get_id() == pair.1).unwrap_or(0)];
            let dx = p_a.0 - p_b.0; let dy = p_a.1 - p_b.1;
            eprintln!(
                "[COLLISION] id={} & id={} dist={:.1} pos_a=({:.0},{:.0}) pos_b=({:.0},{:.0})",
                pair.0, pair.1, (dx*dx+dy*dy).sqrt(), p_a.0, p_a.1, p_b.0, p_b.1
            );
        }
    }
    for v in vehicles.iter_mut() {
        let id = v.get_id();
        if collision_pairs.iter().any(|&(a, b)| a == id || b == id) {
            v.set_velocity_level(VelocityLevel::Stopped);
        }
    }
    statistics.update_collisions(&collision_pairs);
}

pub fn will_paths_conflict(dir1: Direction, route1: Route, dir2: Direction, route2: Route) -> bool {
    // Vehicles from the same entry direction never block each other at the intersection
    if dir1 == dir2 {
        return false;
    }
    paths_physically_cross(dir1, route1, dir2, route2)
}

/// Returns true if the two vehicles' paths physically cross inside the intersection.
fn paths_physically_cross(dir1: Direction, route1: Route, dir2: Direction, route2: Route) -> bool {
    // Right turns stay on the outer edge and never cross other paths
    if route1 == Route::Right || route2 == Route::Right {
        return false;
    }
    // Opposite directions going straight use separate parallel lanes — they don't cross
    match (dir1, dir2) {
        (Direction::North, Direction::South) | (Direction::South, Direction::North) => {
            // Only conflict when at least one turns Left (into oncoming lane)
            route1 == Route::Left || route2 == Route::Left
        }
        (Direction::East, Direction::West) | (Direction::West, Direction::East) => {
            // Only conflict when at least one turns Left (into oncoming lane)
            route1 == Route::Left || route2 == Route::Left
        }
        // Perpendicular directions always cross
        _ => true,
    }
}

pub fn is_in_or_near_intersection(pos: (f32, f32), intersection: &Intersection, padding: f32) -> bool {
    let dx = (pos.0 - intersection.center.0).abs();
    let dy = (pos.1 - intersection.center.1).abs();
    dx < intersection.size + padding && dy < intersection.size + padding
}

// === Arrival-Time-Window helper functions ===

/// Distance (px) from vehicle's position to the intersection entry edge along its direction.
/// Returns 0.0 if already at or past the entry edge.
fn dist_to_entry_edge(pos: (f32, f32), dir: Direction, intersection: &Intersection) -> f32 {
    match dir {
        Direction::North => (pos.1 - (intersection.center.1 + intersection.size)).max(0.0),
        Direction::South => ((intersection.center.1 - intersection.size) - pos.1).max(0.0),
        Direction::East  => ((intersection.center.0 - intersection.size) - pos.0).max(0.0),
        Direction::West  => (pos.0 - (intersection.center.0 + intersection.size)).max(0.0),
    }
}

/// Distance (px) from vehicle's current position to the exit edge along its direction.
/// Used for vehicles already inside the intersection to estimate remaining crossing time.
fn dist_to_exit_edge(pos: (f32, f32), dir: Direction, intersection: &Intersection) -> f32 {
    match dir {
        Direction::North => (pos.1 - (intersection.center.1 - intersection.size)).max(0.0),
        Direction::South => ((intersection.center.1 + intersection.size) - pos.1).max(0.0),
        Direction::East  => ((intersection.center.0 + intersection.size) - pos.0).max(0.0),
        Direction::West  => (pos.0 - (intersection.center.0 - intersection.size)).max(0.0),
    }
}

/// Approximate path length through the intersection for each route (pixels).
fn crossing_distance(route: Route) -> f32 {
    match route {
        Route::Right    => 100.0, // short quarter-arc on outer edge
        Route::Straight => 400.0, // full diameter
        Route::Left     => 500.0, // wider arc across the intersection
    }
}

/// Compute a vehicle’s time window [entry_t, exit_t] in seconds relative to now,
/// with WINDOW_BUFFER_SECS applied to both ends.
///
/// * `in_intersection` – true when the vehicle is already inside the intersection box.
/// * `vel`            – velocity in px/s.
fn compute_time_window(
    pos: (f32, f32),
    dir: Direction,
    route: Route,
    vel: f32,
    in_intersection: bool,
    intersection: &Intersection,
) -> (f32, f32) {
    if in_intersection {
        let remaining = dist_to_exit_edge(pos, dir, intersection);
        let exit_t = if vel > 0.0 { remaining / vel } else { f32::MAX };
        (-WINDOW_BUFFER_SECS, exit_t + WINDOW_BUFFER_SECS)
    } else {
        if vel <= 0.0 {
            return (f32::MAX, f32::MAX); // stopped outside → no future window
        }
        let d_entry = dist_to_entry_edge(pos, dir, intersection);
        let cross   = crossing_distance(route);
        (
            d_entry / vel - WINDOW_BUFFER_SECS,
            (d_entry + cross) / vel + WINDOW_BUFFER_SECS,
        )
    }
}

/// True when two time windows [a.0, a.1] and [b.0, b.1] overlap.
fn windows_overlap(a: (f32, f32), b: (f32, f32)) -> bool {
    a.0 < b.1 && b.0 < a.1
}

// === Public ATW prediction API ===

/// Return a vehicle's projected intersection-occupancy time window `(entry_t, exit_t)` in
/// seconds relative to now, padded by `WINDOW_BUFFER_SECS` on each side.
///
/// The calculation uses the vehicle's **actual current velocity** so that the window
/// automatically shifts later as the vehicle decelerates.
///
/// * Vehicles already inside the intersection receive `entry_t = -WINDOW_BUFFER_SECS`
///   (they have priority); `exit_t` is derived from distance remaining to exit edge.
/// * Vehicles outside but with `velocity == 0` receive `(f32::MAX, f32::MAX)` — no
///   meaningful future window can be computed.
///
/// Formula:
/// $$t_{entry} = \frac{d_{entry}}{v} - B,\quad t_{exit} = \frac{d_{entry} + d_{cross}}{v} + B$$
/// where $B$ = `WINDOW_BUFFER_SECS`.
pub fn vehicle_time_window(v: &Vehicle, intersection: &Intersection) -> (f32, f32) {
    let pos      = v.get_position();
    let vel      = v.get_velocity();
    let in_isect = intersection.contains_point(pos);
    compute_time_window(pos, v.get_direction(), v.get_route(), vel, in_isect, intersection)
}

/// Return `true` when two half-open time windows `[a.0, a.1)` and `[b.0, b.1)` overlap.
///
/// $$\text{overlap} \iff a_0 < b_1 \land b_0 < a_1$$
pub fn check_overlap(window_a: (f32, f32), window_b: (f32, f32)) -> bool {
    windows_overlap(window_a, window_b)
}

/// Predict whether vehicles `a` and `b` will conflict at the intersection.
///
/// Returns `true` when **both** conditions hold:
/// 1. `will_paths_conflict` — the vehicles' physical paths cross inside the intersection.
/// 2. `check_overlap`       — their projected time windows overlap, meaning they are
///    predicted to occupy the crossing region at the same time.
///
/// This is the primary entry point for ATW conflict detection from outside this module.
pub fn predict_atw_conflict(a: &Vehicle, b: &Vehicle, intersection: &Intersection) -> bool {
    if !will_paths_conflict(a.get_direction(), a.get_route(), b.get_direction(), b.get_route()) {
        return false;
    }
    let win_a = vehicle_time_window(a, intersection);
    let win_b = vehicle_time_window(b, intersection);
    check_overlap(win_a, win_b)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_intersection() -> Intersection {
        Intersection::new(700.0, 450.0, 200.0)
    }

    #[test]
    fn extracted_module_compiles() {
        let intersection = make_intersection();
        let mut vehicles: Vec<Vehicle> = Vec::new();
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &intersection, &mut stats);
    }

    // Geometry reference:
    // Intersection center: (700.0, 450.0), size: 200.0, padding: INTERSECTION_PADDING=200.0
    // Near zone: |x - 700| < 400 AND |y - 450| < 400  → for x=700: y in (50, 850) exclusive
    // In-intersection: |x - 700| < 200 AND |y - 450| < 200
    // Far zone: |y - 450| >= 400 → for x=700: y <= 50 or y >= 850 (boundary is FAR)

    #[test]
    fn vehicle_far_from_intersection_gets_fast_velocity() {
        // y=1000 → |1000 - 450| = 550 > 400 → far zone
        let mut vehicle = Vehicle::new(0, Direction::North, Route::Straight);
        vehicle.set_position(700.0, 1000.0);
        let mut vehicles = vec![vehicle];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(vehicles[0].get_velocity_level(), VelocityLevel::Fast);
    }

    #[test]
    fn vehicle_in_intersection_gets_normal_velocity() {
        // (700, 450) is the center — inside on both axes (|0| < 200)
        let mut vehicle = Vehicle::new(0, Direction::North, Route::Straight);
        vehicle.set_position(700.0, 450.0);
        let mut vehicles = vec![vehicle];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(vehicles[0].get_velocity_level(), VelocityLevel::Normal);
    }

    #[test]
    fn single_vehicle_approaching_with_no_conflicts_gets_fast() {
        // With ATW algorithm a single approaching vehicle (no conflicts) gets Fast,
        // not Normal — no reason to slow down when the intersection is clear.
        // y=700: dist_to_entry = 700-(450+200)=50px, inside ETA_LOOKAHEAD(500), no conflicts.
        let mut vehicle = Vehicle::new(0, Direction::North, Route::Straight);
        vehicle.set_position(700.0, 700.0);
        let mut vehicles = vec![vehicle];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(vehicles[0].get_velocity_level(), VelocityLevel::Fast);
    }

    #[test]
    fn safety_distance_reduction_not_overridden_by_baseline_pass() {
        // Two North/Straight vehicles (same lane=1, since Straight → assigned_lane=1)
        // Leader at y=870 (far: |870-450|=420 > 400 → outside near zone)
        // Follower at y=920 (far: |920-450|=470 > 400 → outside near zone)
        // Gap = 50px — within SAFETY_DISTANCE_SLOW (80px) → safety pass must reduce follower to Slow
        //
        // FIX (Task 4): baseline runs FIRST → safety pass reduces follower correctly; baseline cannot override.
        // Gap 50 < SAFETY_DISTANCE_SLOW(80) → Slow tier.
        let mut leader   = Vehicle::new(0, Direction::North, Route::Straight);
        let mut follower  = Vehicle::new(1, Direction::North, Route::Straight);
        leader.set_position(700.0, 870.0);   // ahead (North moves toward smaller y, so leader has smaller y)
        follower.set_position(700.0, 920.0); // 50px behind leader
        let mut vehicles = vec![leader, follower];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(
            vehicles[1].get_velocity_level(),
            VelocityLevel::Slow,
            "follower 50px behind leader is within SAFETY_DISTANCE_SLOW (80px) → must be reduced to Slow"
        );
    }

    #[test]
    fn conflicting_vehicles_atw_priority_closer_proceeds_farther_stops() {
        // North/Straight at (700,700): dist_to_entry=50px
        //   raw_t_entry = 50/160 - 0.3 = 0.0125s  (initial vel = Normal = 160)
        // East/Straight  at (400,450): dist_to_entry=100px
        //   raw_t_entry = 100/160 - 0.3 = 0.325s
        //
        // ATW processes North first (lower t_entry).  No committed conflicts → Fast (260).
        // North committed exit_t = (50+400)/260 + 0.3 ≈ 2.031s.
        // East finds North as blocker: target_vel = 100/(2.031+0.3) ≈ 42.9 px/s
        //   → Custom(~42.9), above CRAWL_SPEED(20), below Slow(80).
        let mut north = Vehicle::new(0, Direction::North, Route::Straight);
        let mut east  = Vehicle::new(1, Direction::East, Route::Straight);
        north.set_position(700.0, 700.0);
        east.set_position(400.0, 450.0);
        let mut vehicles = vec![north, east];
        let mut stats = Statistics::new();
        check_collisions_and_apply_strategy(&mut vehicles, &make_intersection(), &mut stats);
        assert_eq!(vehicles[0].get_velocity_level(), VelocityLevel::Fast,
            "North (closer / earlier t_entry) should proceed at Fast");
        // East is ATW-modulated to a Custom velocity that arrives after North clears.
        assert!(
            matches!(vehicles[1].get_velocity_level(), VelocityLevel::Custom(_)),
            "East should be ATW-modulated (Custom), got {:?}", vehicles[1].get_velocity_level()
        );
        let east_v = vehicles[1].get_velocity();
        assert!(east_v >= 20.0 && east_v < 80.0,
            "East ATW velocity should be in [CRAWL_SPEED, Slow), got {east_v:.2}");
    }

    // --- Public ATW API unit tests ---

    #[test]
    fn check_overlap_non_overlapping_windows() {
        assert!(!check_overlap((0.0, 1.0), (2.0, 3.0)));
        assert!(!check_overlap((2.0, 3.0), (0.0, 1.0)));
    }

    #[test]
    fn check_overlap_touching_windows_are_not_overlapping() {
        // [0,1) and [1,2) share only the boundary — not considered overlapping.
        assert!(!check_overlap((0.0, 1.0), (1.0, 2.0)));
    }

    #[test]
    fn check_overlap_overlapping_windows() {
        assert!(check_overlap((0.0, 2.0), (1.0, 3.0)));
        assert!(check_overlap((1.0, 3.0), (0.0, 2.0)));
        // One window fully inside the other
        assert!(check_overlap((0.0, 10.0), (2.0, 5.0)));
    }

    #[test]
    fn vehicle_time_window_approaching_vehicle() {
        // North/Straight at y=850: dist_to_entry = 850-(450+200) = 200px
        // vel = Fast = 260 px/s (VelocityLevel::Fast); crossing = 400px
        // entry_t = 200/260 - 0.3 ≈ 0.469
        // exit_t  = (200+400)/260 + 0.3 ≈ 2.608
        let mut v = Vehicle::new(0, Direction::North, Route::Straight);
        v.set_position(700.0, 850.0);
        v.set_velocity_level(VelocityLevel::Fast);
        let (entry_t, exit_t) = vehicle_time_window(&v, &make_intersection());
        let eps = 0.001;
        assert!((entry_t - (200.0_f32 / 260.0 - 0.3)).abs() < eps, "entry_t={entry_t}");
        assert!((exit_t  - (600.0_f32 / 260.0 + 0.3)).abs() < eps, "exit_t={exit_t}");
    }

    #[test]
    fn vehicle_time_window_stopped_outside_returns_max() {
        let mut v = Vehicle::new(0, Direction::North, Route::Straight);
        v.set_position(700.0, 850.0);
        v.set_velocity_level(VelocityLevel::Stopped);
        let (entry_t, exit_t) = vehicle_time_window(&v, &make_intersection());
        assert_eq!(entry_t, f32::MAX);
        assert_eq!(exit_t,  f32::MAX);
    }

    #[test]
    fn vehicle_time_window_inside_intersection() {
        // Vehicle already inside: entry_t = -WINDOW_BUFFER_SECS = -0.3
        let mut v = Vehicle::new(0, Direction::North, Route::Straight);
        v.set_position(700.0, 450.0);  // intersection centre
        v.set_velocity_level(VelocityLevel::Normal);
        let (entry_t, _exit_t) = vehicle_time_window(&v, &make_intersection());
        let eps = 0.001;
        assert!((entry_t - (-WINDOW_BUFFER_SECS)).abs() < eps, "entry_t={entry_t}");
    }

    #[test]
    fn predict_atw_conflict_non_conflicting_paths() {
        // Two vehicles from the same direction: paths never cross.
        let mut a = Vehicle::new(0, Direction::North, Route::Straight);
        let mut b = Vehicle::new(1, Direction::North, Route::Right);
        a.set_position(700.0, 850.0);
        b.set_position(750.0, 900.0);
        assert!(!predict_atw_conflict(&a, &b, &make_intersection()));
    }

    #[test]
    fn predict_atw_conflict_conflicting_paths_overlapping_windows() {
        // North/Straight and East/Straight — perpendicular paths, both close.
        let mut north = Vehicle::new(0, Direction::North, Route::Straight);
        let mut east  = Vehicle::new(1, Direction::East, Route::Straight);
        north.set_position(700.0, 700.0);  // 50px from entry
        east.set_position(400.0, 475.0);   // 100px from entry
        north.set_velocity_level(VelocityLevel::Fast);
        east.set_velocity_level(VelocityLevel::Fast);
        assert!(predict_atw_conflict(&north, &east, &make_intersection()));
    }

    #[test]
    fn predict_atw_conflict_conflicting_paths_non_overlapping_windows() {
        // North is very close (nearly through); East is far away → windows don't overlap.
        let mut north = Vehicle::new(0, Direction::North, Route::Straight);
        let mut east  = Vehicle::new(1, Direction::East, Route::Straight);
        // North at entry edge — exits quickly.
        north.set_position(700.0, 652.0);   // ~2px from entry (450+200=650)
        // East very far away — arrives much later.
        east.set_position(-500.0, 475.0);
        north.set_velocity_level(VelocityLevel::Fast);
        east.set_velocity_level(VelocityLevel::Fast);
        assert!(!predict_atw_conflict(&north, &east, &make_intersection()));
    }
}
