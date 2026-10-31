//! Determinism tests: the same inputs must give bit-identical outputs, on
//! every run, every thread and every platform.

use std::borrow::Cow;

use glam::Vec2;

use crate::{Agent, AvoidanceOptions, Obstacle};

const OPTIONS: AvoidanceOptions = AvoidanceOptions {
  obstacle_margin: 0.1,
  time_horizon: 2.0,
  obstacle_time_horizon: 1.0,
};

fn agent(id: u64, position: Vec2, velocity: Vec2) -> Agent {
  Agent { id, position, velocity, radius: 0.5, avoidance_responsibility: 1.0 }
}

fn bits(v: Vec2) -> [u32; 2] {
  [v.x.to_bits(), v.y.to_bits()]
}

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

  fn write_vec2(&mut self, value: Vec2) {
    self.write_u32(value.x.to_bits());
    self.write_u32(value.y.to_bits());
  }
}

/// A crowd crossing a box with a pillar in the middle. It contains two pairs of
/// agents that start exactly on top of each other (one of them with equal
/// velocities, so it goes through the tie-break), agents with equal and
/// different avoidance responsibilities, and enough density to make the agent
/// constraints infeasible (which uses the 3D linear program).
fn scene() -> (Vec<Agent>, Vec<Vec2>, Vec<Obstacle>) {
  let mut agents = Vec::new();
  let mut goals = Vec::new();
  for i in 0..12u32 {
    // Two rows facing each other, each agent heading to the opposite side.
    let t = i as f32 / 12.0;
    let x = 4.0 * t - 2.0;
    let side = if i % 2 == 0 { 1.0 } else { -1.0 };
    let position = Vec2::new(x * 3.0, side * 6.0);
    agents.push(agent(i as u64 * 3 + 1, position, Vec2::ZERO));
    goals.push(-position);
  }
  // Superposed with equal velocities: the degenerate case.
  agents.push(agent(100, Vec2::new(0.5, 0.25), Vec2::ZERO));
  goals.push(Vec2::new(8.0, 0.0));
  agents.push(agent(101, Vec2::new(0.5, 0.25), Vec2::ZERO));
  goals.push(Vec2::new(-8.0, 0.0));
  // Superposed with different velocities.
  agents.push(agent(200, Vec2::new(-3.0, 1.0), Vec2::new(1.0, 0.0)));
  goals.push(Vec2::new(-3.0, 9.0));
  let mut heavy = agent(201, Vec2::new(-3.0, 1.0), Vec2::new(0.0, 1.0));
  heavy.avoidance_responsibility = 0.25;
  agents.push(heavy);
  goals.push(Vec2::new(-3.0, -9.0));
  // Already inside the margin of the pillar's corner, rushing at it.
  agents.push(agent(300, Vec2::new(2.8, 0.7), Vec2::new(2.0, -2.0)));
  goals.push(Vec2::new(9.0, -6.0));

  let obstacles = vec![
    Obstacle::Closed {
      vertices: vec![
        Vec2::new(-1.0, -1.0),
        Vec2::new(1.0, -1.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(-1.0, 1.0),
      ]
      .into_iter()
      .map(|v| v + Vec2::new(4.0, -0.5))
      .collect(),
    },
    // A box around everything; clockwise, so it keeps agents inside.
    Obstacle::Closed {
      vertices: vec![
        Vec2::new(-12.0, -12.0),
        Vec2::new(-12.0, 12.0),
        Vec2::new(12.0, 12.0),
        Vec2::new(12.0, -12.0),
      ],
    },
  ];
  (agents, goals, obstacles)
}

/// Runs the scene for `steps` steps, the way a game would: neighbours within a
/// query radius, sorted by id. Returns a hash of every intermediate velocity
/// and position.
fn run_scene(steps: u32) -> (u64, Vec<Agent>) {
  let (mut agents, goals, obstacles) = scene();
  let obstacles = obstacles.iter().map(Cow::Borrowed).collect::<Vec<_>>();
  let time_step = 0.1;
  let max_speed = 2.0;
  let mut hash = Fnv::new();
  for _ in 0..steps {
    let mut velocities = Vec::with_capacity(agents.len());
    for (index, agent) in agents.iter().enumerate() {
      let mut neighbours = agents
        .iter()
        .enumerate()
        .filter(|&(other, neighbour)| {
          other != index
            && neighbour.position.distance_squared(agent.position) < 25.0
        })
        .map(|(_, neighbour)| neighbour)
        .collect::<Vec<_>>();
      neighbours.sort_by_key(|neighbour| neighbour.id);
      let neighbours =
        neighbours.into_iter().map(Cow::Borrowed).collect::<Vec<_>>();
      let to_goal = goals[index] - agent.position;
      let preferred = if to_goal.length_squared() > max_speed * max_speed {
        to_goal / to_goal.length_squared().sqrt() * max_speed
      } else {
        to_goal
      };
      velocities.push(agent.compute_avoiding_velocity(
        &neighbours,
        &obstacles,
        preferred,
        max_speed,
        time_step,
        &OPTIONS,
      ));
    }
    for (agent, velocity) in agents.iter_mut().zip(velocities) {
      agent.velocity = velocity;
      agent.position += velocity * time_step;
      hash.write_vec2(agent.velocity);
      hash.write_vec2(agent.position);
    }
  }
  (hash.0, agents)
}

#[test]
fn identical_inputs_give_bit_identical_outputs() {
  let (expected, expected_agents) = run_scene(60);
  for _ in 0..5 {
    let (hash, agents) = run_scene(60);
    assert_eq!(hash, expected);
    for (a, b) in agents.iter().zip(expected_agents.iter()) {
      assert_eq!(bits(a.position), bits(b.position));
      assert_eq!(bits(a.velocity), bits(b.velocity));
    }
  }
}

#[test]
fn identical_outputs_across_threads() {
  let (expected, _) = run_scene(60);
  let handles =
    (0..4).map(|_| std::thread::spawn(|| run_scene(60).0)).collect::<Vec<_>>();
  for handle in handles {
    assert_eq!(handle.join().unwrap(), expected);
  }
}

/// Computes the avoiding velocity of `agent` against a single neighbour, when
/// the agent wants to keep its current velocity.
fn avoid(agent: &Agent, neighbour: &Agent) -> Vec2 {
  agent.compute_avoiding_velocity(
    &[Cow::Borrowed(neighbour)],
    &[],
    agent.velocity,
    /* max_speed= */ 100.0,
    /* time_step= */ 0.1,
    &OPTIONS,
  )
}

#[test]
fn superposed_agents_separate_in_opposite_directions() {
  let a = agent(7, Vec2::new(3.0, -2.0), Vec2::new(0.5, 0.5));
  let b = agent(8, Vec2::new(3.0, -2.0), Vec2::new(0.5, 0.5));

  let va = avoid(&a, &b);
  let vb = avoid(&b, &a);

  assert_ne!(va, Vec2::ZERO);
  // Exactly opposite (relative to the shared velocity they had).
  let shared = Vec2::new(0.5, 0.5);
  let da = va - shared;
  let db = vb - shared;
  assert!(da.dot(db) < 0.0, "{da} {db}");
  assert!((da + db).length() < 1e-6, "{da} {db}");
}

#[test]
fn superposed_agents_at_rest_get_bit_exact_opposite_velocities() {
  let a = agent(7, Vec2::new(3.0, -2.0), Vec2::ZERO);
  let b = agent(8, Vec2::new(3.0, -2.0), Vec2::ZERO);

  let va = avoid(&a, &b);
  let vb = avoid(&b, &a);

  assert_ne!(va, Vec2::ZERO);
  assert_eq!(bits(va), bits(-vb));
}

#[test]
fn superposed_tie_break_depends_only_on_keys() {
  let a = agent(7, Vec2::new(3.0, -2.0), Vec2::ZERO);
  let b = agent(8, Vec2::new(3.0, -2.0), Vec2::ZERO);
  let expected = avoid(&a, &b);
  // Upstream picked a random direction here, so repeated calls disagreed.
  for _ in 0..100 {
    assert_eq!(bits(avoid(&a, &b)), bits(expected));
  }

  // Moving the pair elsewhere does not change the direction.
  let mut a_moved = a.clone();
  let mut b_moved = b.clone();
  a_moved.position = Vec2::new(-40.0, 17.0);
  b_moved.position = Vec2::new(-40.0, 17.0);
  assert_eq!(bits(avoid(&a_moved, &b_moved)), bits(expected));

  // Swapping the keys swaps the directions.
  let mut a_swapped = a.clone();
  let mut b_swapped = b.clone();
  a_swapped.id = 8;
  b_swapped.id = 7;
  assert_eq!(bits(avoid(&a_swapped, &b_swapped)), bits(-expected));

  // Another pair of keys gives another direction.
  let mut c = b.clone();
  c.id = 9;
  assert_ne!(bits(avoid(&a, &c)), bits(expected));
}

#[test]
fn feasible_result_does_not_depend_on_neighbour_order() {
  let me = agent(0, Vec2::ZERO, Vec2::new(1.0, 0.0));
  let neighbours = [
    agent(1, Vec2::new(3.0, 0.5), Vec2::new(-1.0, 0.0)),
    agent(2, Vec2::new(2.0, -2.0), Vec2::new(0.0, 1.0)),
    agent(3, Vec2::new(-1.5, 2.0), Vec2::new(0.5, -0.5)),
    agent(4, Vec2::new(2.5, 2.5), Vec2::new(-0.5, -1.0)),
  ];
  let compute = |order: &[usize]| {
    let neighbours =
      order.iter().map(|&i| Cow::Borrowed(&neighbours[i])).collect::<Vec<_>>();
    me.compute_avoiding_velocity(
      &neighbours,
      &[],
      Vec2::new(2.0, 0.0),
      2.0,
      0.1,
      &OPTIONS,
    )
  };
  let expected = compute(&[0, 1, 2, 3]);
  for order in
    [[3, 2, 1, 0], [1, 3, 0, 2], [2, 0, 3, 1], [0, 2, 1, 3], [3, 0, 2, 1]]
  {
    let result = compute(&order);
    // The optimum is unique, so every order agrees up to rounding. Bitwise
    // equality is only promised for a stable order (see
    // `Agent::compute_avoiding_velocity`).
    assert!(
      result.distance(expected) < 1e-5,
      "{order:?}: {result} vs {expected}"
    );
    // The same order always gives the same bits.
    assert_eq!(bits(compute(&order)), bits(result));
  }
}

/// Obstacle constraints that cannot be satisfied (the agent is squeezed
/// between two obstacles), which takes the zero-velocity fallback, with a
/// neighbour on top of it.
fn infeasible_obstacles_case() -> Vec2 {
  let me = agent(12, Vec2::ZERO, Vec2::new(2.0, 0.0));
  let neighbour = agent(13, Vec2::new(0.25, 0.0), Vec2::new(-1.0, 0.0));
  let obstacles = [
    Obstacle::Closed {
      vertices: vec![
        Vec2::new(1.0, 10.0),
        Vec2::new(1.0, 0.0),
        Vec2::new(2.0, 10.0),
      ],
    },
    Obstacle::Closed {
      vertices: vec![
        Vec2::new(1.0, 1e-6),
        Vec2::new(1.0, -10.0),
        Vec2::new(2.0, -10.0),
      ],
    },
  ];
  me.compute_avoiding_velocity(
    &[Cow::Borrowed(&neighbour)],
    &obstacles.iter().map(Cow::Borrowed).collect::<Vec<_>>(),
    Vec2::new(2.0, 0.0),
    2.0,
    0.01,
    &AvoidanceOptions {
      obstacle_margin: 0.0,
      obstacle_time_horizon: 1.0,
      time_horizon: 1.0,
    },
  )
}

/// A recorded golden for `run_scene`. CI checks it on Linux (x86_64) and
/// macOS (aarch64): a mismatch on one platform only means some operation is
/// not portable. If the algorithm is changed on purpose, update the value.
#[test]
fn golden_scene() {
  let (hash, _) = run_scene(60);
  let mut hash = Fnv(hash);
  hash.write_vec2(infeasible_obstacles_case());
  let hash = hash.0;
  assert_eq!(hash, GOLDEN_2D, "actual: {hash:#018x}");
}

const GOLDEN_2D: u64 = 0x508c_c52a_ec83_33b9;
