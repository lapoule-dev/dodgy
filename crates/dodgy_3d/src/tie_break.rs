//! Deterministic tie-breaking for degenerate agent pairs.
//!
//! When an agent and one of its neighbours are in a perfectly symmetric
//! situation (typically: exactly on top of each other with the same velocity),
//! the velocity obstacle has no natural normal. Upstream dodgy picks a random
//! direction there, which makes the output depend on a thread-local RNG. This
//! module replaces it with a direction derived only from the two agents'
//! [`Agent::id`](crate::Agent::id) keys, using integer hashing, integer to
//! float conversion, `+ - * /` and `sqrt` (all of which are exactly specified
//! by IEEE 754), so the result is bit-identical on every platform.

use glam::Vec3;

/// The SplitMix64 finalizer. A bijective mixer of 64-bit integers.
pub(crate) fn mix64(mut z: u64) -> u64 {
  z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
  z = (z ^ (z >> 30)).wrapping_mul(0xBF58_4D1C_E4E5_B9B9);
  z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
  z ^ (z >> 31)
}

/// Hashes an unordered pair of keys. Returns the hash and `1.0` if `agent_id`
/// is the smaller key of the pair, `-1.0` otherwise.
pub(crate) fn pair_hash(agent_id: u64, neighbour_id: u64) -> (u64, f32) {
  let (low, high, sign) = if agent_id <= neighbour_id {
    (agent_id, neighbour_id, 1.0)
  } else {
    (neighbour_id, agent_id, -1.0)
  };
  (mix64(low ^ mix64(high)), sign)
}

/// Maps the top bits of `bits` to a float in `[-1, 1)`. The conversion from
/// a 24-bit integer to `f32` is exact.
pub(crate) fn unit_component(bits: u64) -> f32 {
  let value = (bits >> 40) as i32 - (1 << 23);
  value as f32 / (1 << 23) as f32
}

/// A unit direction in which `agent_id` should move away from `neighbour_id`
/// when the pair is degenerate. The direction only depends on the unordered
/// pair of keys, except for its sign: the agent with the smaller key gets
/// `d` and the other one gets `-d`, so the two agents separate in opposite
/// directions. If both keys are equal, both agents get the same direction
/// (still deterministic, but they will not separate).
pub(crate) fn separation_direction(agent_id: u64, neighbour_id: u64) -> Vec3 {
  let (hash, sign) = pair_hash(agent_id, neighbour_id);
  let direction = Vec3::new(
    unit_component(hash),
    unit_component(hash << 24),
    unit_component(mix64(hash)),
  );
  let length_squared = direction.length_squared();
  let direction = if length_squared > 0.0 {
    direction / length_squared.sqrt()
  } else {
    Vec3::X
  };
  direction * sign
}

#[cfg(test)]
#[path = "tie_break_test.rs"]
mod test;
