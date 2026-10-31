//! Determinism tests for `VisibilitySet`: the same sequence of calls must give
//! bit-identical internal state and results, on every run, every thread and
//! every platform. The trigonometry goes through `libm`, so this also checks
//! that no platform math leaks in.

use glam::Vec2;

use super::VisibilitySet;

/// 64-bit FNV-1a, folded over the bits of every value.
struct Fnv(u64);

impl Fnv {
  fn new() -> Self {
    Self(0xcbf2_9ce4_8422_2325)
  }

  fn write_u32(&mut self, value: u32) {
    for byte in value.to_le_bytes() {
      self.0 ^= byte as u64;
      self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
    }
  }

  fn write_f32(&mut self, value: f32) {
    self.write_u32(value.to_bits());
  }

  fn write_vec2(&mut self, value: Vec2) {
    self.write_f32(value.x);
    self.write_f32(value.y);
  }
}

/// A line segment, as a pair of end points.
type Segment = (Vec2, Vec2);

/// A deterministic sequence of line segments around the origin: walls at
/// several distances, some overlapping, some intersecting, some crossing the
/// positive x axis (which splits them into two cones), plus a few probes.
fn scene() -> (Vec<Segment>, Vec<Segment>) {
  let mut lines = Vec::new();
  let mut probes = Vec::new();
  // A ring of 24 short walls at radius 5, with gaps, from integer points so
  // the inputs are exact.
  let ring = [
    (5, 0),
    (5, 2),
    (4, 3),
    (3, 4),
    (2, 5),
    (0, 5),
    (-2, 5),
    (-3, 4),
    (-4, 3),
    (-5, 2),
    (-5, 0),
    (-5, -2),
    (-4, -3),
    (-3, -4),
    (-2, -5),
    (0, -5),
    (2, -5),
    (3, -4),
    (4, -3),
    (5, -2),
  ];
  for (index, window) in ring.windows(2).enumerate() {
    if index % 4 == 3 {
      // Leave a gap.
      continue;
    }
    let a = Vec2::new(window[0].0 as f32, window[0].1 as f32);
    let b = Vec2::new(window[1].0 as f32, window[1].1 as f32);
    lines.push((a, b));
  }
  // Closer walls, partially in front of the ring.
  lines.push((Vec2::new(2.0, -1.0), Vec2::new(2.0, 1.5)));
  lines.push((Vec2::new(-1.0, 3.0), Vec2::new(-3.0, 1.0)));
  // A wall crossing the positive x axis (split into two cones).
  lines.push((Vec2::new(3.0, -2.0), Vec2::new(3.5, 2.0)));
  // Walls intersecting each other.
  lines.push((Vec2::new(-2.0, -2.0), Vec2::new(-1.0, -3.0)));
  lines.push((Vec2::new(-2.5, -2.5), Vec2::new(-0.5, -2.5)));
  // Walls behind everything.
  lines.push((Vec2::new(8.0, 1.0), Vec2::new(1.0, 8.0)));
  lines.push((Vec2::new(-8.0, -1.0), Vec2::new(-1.0, -8.0)));
  // A degenerate line (ignored).
  lines.push((Vec2::new(1.0, 1.0), Vec2::new(1.0, 1.0)));

  for i in 0..16 {
    let t = i as f32 / 16.0;
    let radius = 1.0 + 6.0 * t;
    let a = Vec2::new(radius, -radius * 0.5 + 3.0 * t);
    let b = Vec2::new(-radius * 0.25 + 2.0 * t, radius);
    probes.push((a, b));
    probes.push((-a, -b));
  }
  (lines, probes)
}

/// Builds the scene and hashes every result and the full internal state.
fn run_scene() -> u64 {
  let (lines, probes) = scene();
  let mut hash = Fnv::new();
  let mut set = VisibilitySet::new();
  for (start, end) in lines {
    let id = set.add_line(start, end);
    hash.write_u32(id.map_or(u32::MAX, |id| id + 1));
    for cone in &set.cones {
      hash.write_u32(cone.line_id);
      hash.write_f32(cone.related_angle);
      for ray in [&cone.left_ray, &cone.right_ray] {
        hash.write_f32(ray.angle);
        hash.write_f32(ray.distance);
        hash.write_vec2(ray.point);
      }
    }
  }
  for id in set.get_visible_line_ids() {
    hash.write_u32(id);
  }
  for (start, end) in probes {
    hash.write_u32(set.is_line_visible(start, end) as u32);
  }
  hash.0
}

#[test]
fn identical_inputs_give_bit_identical_state() {
  let expected = run_scene();
  for _ in 0..5 {
    assert_eq!(run_scene(), expected);
  }
}

#[test]
fn identical_state_across_threads() {
  let expected = run_scene();
  let handles =
    (0..4).map(|_| std::thread::spawn(run_scene)).collect::<Vec<_>>();
  for handle in handles {
    assert_eq!(handle.join().unwrap(), expected);
  }
}

#[test]
fn visible_ids_are_sorted() {
  let (lines, _) = scene();
  let mut set = VisibilitySet::new();
  for (start, end) in lines {
    set.add_line(start, end);
  }
  let ids = set.get_visible_line_ids().into_iter().collect::<Vec<_>>();
  assert!(ids.len() > 2);
  assert!(ids.windows(2).all(|w| w[0] < w[1]), "{ids:?}");
}

/// A recorded golden for `run_scene`. CI checks it on Linux (x86_64) and
/// macOS (aarch64): a mismatch on one platform only means some operation is
/// not portable. If the algorithm is changed on purpose, update the value.
#[test]
fn golden_scene() {
  let hash = run_scene();
  assert_eq!(hash, GOLDEN_VISIBILITY, "actual: {hash:#018x}");
}

const GOLDEN_VISIBILITY: u64 = 0xecb3_4e7d_fdf2_ea3f;
