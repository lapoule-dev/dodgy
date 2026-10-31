use super::*;

#[test]
fn swapped_keys_give_exactly_opposite_directions() {
  for (a, b) in [(0, 1), (1, 0), (7, 3), (u64::MAX, 0), (42, 1 << 40)] {
    let ab = separation_direction(a, b);
    let ba = separation_direction(b, a);
    assert_eq!(ab.x.to_bits(), (-ba.x).to_bits(), "{a} {b}");
    assert_eq!(ab.y.to_bits(), (-ba.y).to_bits(), "{a} {b}");
  }
}

#[test]
fn directions_are_unit_length() {
  for a in 0..64u64 {
    for b in 0..64u64 {
      let length = separation_direction(a, b).length();
      assert!((length - 1.0).abs() < 1e-6, "{a} {b} {length}");
    }
  }
}

#[test]
fn equal_keys_are_deterministic() {
  let first = separation_direction(5, 5);
  let second = separation_direction(5, 5);
  assert_eq!(first.x.to_bits(), second.x.to_bits());
  assert_eq!(first.y.to_bits(), second.y.to_bits());
}

#[test]
fn different_pairs_spread_over_all_quadrants() {
  let mut quadrants = [false; 4];
  for b in 1..32u64 {
    let d = separation_direction(0, b);
    quadrants[(d.x < 0.0) as usize * 2 + (d.y < 0.0) as usize] = true;
  }
  assert_eq!(quadrants, [true; 4]);
}
