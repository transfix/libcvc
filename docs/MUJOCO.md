# MuJoCo robots (`cvc::physics::mujoco_robot`)

`cvc::physics::mujoco_robot` runs one articulated robot in MuJoCo, standing on the libcvc
world's terrain. MuJoCo owns the robot, and the ground is the same `cvc::height_field` that the
renderer, `cvc::nav` and the Jolt world read. The first robot is the Unitree Go2 from the
MuJoCo Menagerie. This is the base for the C-PHAST lab (Phase 6 of
`docs/roadmap/PH-COMPOSE-ROADMAP.md`, on transfix/libcvc#1227): C-PHAST predicts a robot's
futures, and MuJoCo is the ground truth they are checked against, energy included.

Header: `cvc/physics/mujoco_robot.h`. CLI: `cvc mujoco`. Tests: `physics_mujoco_robot_test`.

## Building

MuJoCo is optional and off by default. It comes from cvcpkg only:

```bash
cvcpkg install boost pegtl mujoco mujoco-menagerie --prefix deps --config release --link shared
cmake -S . -B build -DCMAKE_PREFIX_PATH=$PWD/deps -DCVC_WITH_MUJOCO=ON
```

- `find_package(mujoco 4.0)` searches the deps prefix (`CMAKE_PREFIX_PATH` or `mujoco_DIR`) and
  never the system's paths. If it finds nothing, configure fails. cvcpkg's `mujoco3` (3.17)
  installs the same package name and is refused.
- With tests on, configure also needs the Menagerie (`share/mujoco-menagerie/unitree_go2`). It
  fails without it, so the tests never skip quietly.
- Without the option, the header and the class are still there: `mujoco_robot::available()` is
  false, and the constructor throws `cvc::unsupported_exception`.
- CI: the `mujoco` job in `.github/workflows/ci.yml` builds a minimal libcvc this way on
  ubuntu-24.04 and runs the tests and `cvc mujoco stand`. It runs when the robot's code, the
  height field, the build files or the workflow change. No other lane turns MuJoCo on.

### Licensing

MuJoCo is Apache-2.0 and libcvc is LGPL-2.1-only, so MuJoCo stays a separate library. None of
its code is compiled into libcvc. Whether the two can ship together has not been checked yet.

- `src/cvc/physics/mujoco/mujoco_engine.cpp` is the only file that includes MuJoCo. It is its
  own target, `cvc_physics_mujoco`, with `mujoco::mujoco` linked PRIVATE, like Jolt's
  `cvc_physics_jolt`. No public header names a MuJoCo type, and `CVC_WITH_MUJOCO` is a private
  definition.
- A shared libcvc links cvcpkg's shared `libmujoco` (`NEEDED libmujoco.so.4.0.0`) and exports
  no MuJoCo symbol. Configure refuses a static `mujoco::mujoco` there.
- A static libcvc (wasm) leaves `libmujoco` to the consumer's link.
- The Go2 model (BSD-3-Clause) is read from the cvcpkg Menagerie at run time. Nothing of the
  Menagerie is in this repository.

## The robot

```cpp
cvc::physics::mujoco_robot_options o;
o.model = deps + "/share/mujoco-menagerie/unitree_go2/scene.xml";
o.terrain = field;                     // a std::shared_ptr<const cvc::height_field>
cvc::physics::mujoco_robot go2(o);     // the "home" keyframe, servos holding it
go2.apply_wrench(go2.body("base"), {0.0, 60.0, 0.0}); // a push, until cleared
go2.step(50);                          // 0.1 s at the Go2's 2 ms
go2.clear_wrenches();
go2.step(1000);
cvc::physics::mujoco_books b = go2.books(); // the work each force did, and MuJoCo's dE
```

**Terrain.** `terrain` replaces the model's `floor` geom with a MuJoCo hfield. The floor's
contact and visual parameters carry over. Checked against MuJoCo 4.0.0:

- MuJoCo's hfield rows run along +y from the minimum y, and its columns along +x.
- It splits each cell along the diagonal from (r, c) to (r + 1, c + 1).

That is `cvc::height_field`'s layout and split, so the robot stands on `field->height(x, y)`.
`TerrainIsTheHeightFieldsTriangles` places a ball over both triangles of bumpy cells; MuJoCo's
contact distance agrees with the height field's own triangle to 1e-5 m.

- MuJoCo rescales the elevations to [0, 1]. So the geom sits at the field's lowest height, and
  its `size[2]` is the range.
- A field with holes (NaN) is refused, because MuJoCo's height fields have none.
- The Go2's feet carry contact priority 1 and their own friction (0.8). So they set the
  contact's friction, not the terrain.

**Start.** The keyframe's joint angles, with the free base at `(x, y)` heading `yaw`. Its
keyframe height (measured from z = 0) is taken along the terrain's normal there
(`align_to_terrain`), with the base tilted to match. It is then raised until no geom reaches into
the terrain.

**Commands.** Each actuator is in `actuator_mode::position` or `actuator_mode::torque`. The
Menagerie's Go2 has torque motors (±23.7 N m at the hips and thighs, ±45.43 N m at the knees).

- In position mode the robot rewrites that actuator's gain and bias in its own `mjModel`, so
  MuJoCo runs the servo, `kp (target - q) - kd qdot`. The force is clamped to the motor's range
  and the target to the joint's range.
- The defaults are kp = 60 N m/rad and kd = 2 N m s/rad. They hold the Go2 up; on top of them,
  the joints' own damping is 2 N m s/rad.
- `set_mode` restarts the command: a servo holds its joint where it is, and a motor applies 0.
- `command`, `set_gains` and `hold` act per actuator or on all of them.

**Pushes.** `apply_wrench(body, force, torque)` acts at the body's centre of mass, in world axes,
and is held until it is replaced or cleared. `apply_force_at` applies a force at a world point.

**Readouts.**

- The current state: `qpos`, `qvel`, the base's pose, velocity and `base_up_z`, and
  `body_position`.
- The last step: `actuator_forces` and `contacts`. Each contact has the geoms, the bodies, the
  point, the normal, the distance, and the force and torque in world axes.
- `energy` returns MuJoCo's own energies of the current state (`mjENBL_ENERGY`): potential
  (gravity and springs) and kinetic.

**Books.** `books()` adds up the work each kind of force did since the robot was built. A step
of h adds `h f . (v0 + v1) / 2`: the force that acted, against the mean of the step's start and
end velocities. The kinds are:

- actuators;
- pushes;
- contacts;
- joint friction (MuJoCo's frictionloss);
- joint limits;
- joint damping, booked as MuJoCo's Euler step applies it (implicitly, `-D v1`);
- everything else.

`residual()` is MuJoCo's ΔE minus the sum of the work: the integrator's own error.

**Snapshots.** `save()` and `restore()` hold MuJoCo's integration state (time, qpos, qvel,
controls, applied forces, the solver's warm start), plus the actuators' modes and gains and the
books. A restored run replays the original to the bit.

**No process-wide state.** The model, the data and the books live on the object. Nothing
installs `mju_user_*` handlers or registers plugins, so two robots share nothing. MuJoCo's own
fatal errors still go to MuJoCo's default handler. If MuJoCo finds the state unusable (NaN or a
runaway acceleration), it resets the robot, and `step()` throws `cvc::physics_error`.

**Determinism.** MuJoCo documents it for one binary. Two robots built alike and given the same
commands step to the same bits. Reading a robot never moves it:

- the last step's readouts are copied out right after the step;
- current-state reads run `mj_forward` with the solver's warm start put back afterwards.

Runs on different platforms or builds can differ, unlike the Jolt world's cross-platform mode.

## What the tests measure

Linux x86_64, GCC 13, Release, MuJoCo 4.0.0 from cvcpkg; `physics_mujoco_robot_test` runs in
about 11 s.

| Test | Result |
|---|---|
| Stands on flat ground (3 s) | base 0.258 m over the terrain, upright to 2.4e-5, moves 1.5 mm after settling; the feet carry the weight to 1% |
| Stands on a 10° slope, facing up, down and across | moves under 1.2 mm after settling, below 5e-4 m/s; the base keeps the slope's tilt |
| Sideways push on the base for 0.1 s, after 1 s of settling | 60 N: moves up to 1.7 cm and recovers. 150 N: up to 8 cm and recovers. 200 N: falls. 300 N: rolls over. The test checks 60 N and 300 N |
| Energy books (3 s, 60 N push) | residual 3.0e-5 J at dt = 2 ms and 1.5e-5 J at 1 ms, against about 5.9 J of gross work. Tolerance: 2e-5 of the gross work, and the residual must halve (ratio 0.3 to 0.7) when the step halves |
| Determinism | two runs, a run that reads the robot every step, and a restored snapshot all end on the same bits |

## `cvc mujoco`

```bash
cvc mujoco info deps/share/mujoco-menagerie/unitree_go2/scene.xml
cvc mujoco stand deps/share/mujoco-menagerie/unitree_go2/scene.xml --slope 10 --push 0,60,0
```

- `info` prints the model: MuJoCo's version, sizes, mass, step, actuators with their force
  ranges, and bodies.
- `stand` runs `run_mujoco_trial` and prints the result as JSON. The trial is a plane at
  `--slope`, settling until `--push-at`, the push for `--push-for`, then on to `--duration`.
  The result has the settled and final height, tilt and speed, the worst tilt and the largest
  displacement, fell or recovered, and the energy books.

## Not here yet

- **A `legged` plant kind (`legged:go2` in `plant_host`).** Left for later. A `nav` plant takes
  `command(steer, accel)` for a bicycle, and turning that into leg motion needs a locomotion
  policy, which libcvc does not have yet. Without one, a `legged:go2` agent would stand still
  while nav plans as if it were driving, and the drift that `sim_world` measures would be all
  error. It fits once there is a policy that maps (vx, yaw rate) to joint targets: trained Go2
  checkpoints from the C-PHAST authors, or one trained on MuJoCo. Then `plant_kind::legged`
  follows the dynamic plants' pattern.
- **Coupling to the Jolt world.** The plan is a one-way kinematic proxy of the robot in Jolt, and
  the vehicles as mocap bodies in MuJoCo, over the shared height field. Two-way contact comes
  later.
- **pycvc.** A SWIG binding of `mujoco_robot` (`reset` / `step` / readouts) for the gym. It needs
  the pycvc lanes to carry MuJoCo first.
- **Other platforms.** cvcpkg publishes `mujoco` for macOS, Windows, wasm and wasm-mt. Only
  Linux x86_64 is built and tested here.
