# Maintained Lite3_VMC fork guide

This is the active ROS Noetic/catkin checkout. The workspace-root hardware boundary still applies. The sibling `lite3_vmc_upstream` repository is reference-only and must remain clean.

## Scope and ownership

- Upstream simulator code lives mainly under `src/quadruped`, `src/gazebo_model_spawn`, `src/examples`, and `src/contact_plugin_pkg`.
- The ROS integration lives under `src/emotion_bot_ros`.
- EmotionBot domain logic remains in the sibling `emotion-bot` repository and is mounted read-only at `/workspaces/emotion-bot`.
- Keep `src/CMakeLists.txt` pointed at `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake`. Do not restore the upstream Melodic symlink.
- Do not edit generated `build`, `devel`, or `log` output.

## Integration contracts

- `src/emotion_bot_ros/config/default.yaml` is the source of truth for topic/service names, backends, mapping profiles, limits, timeouts, and safety defaults.
- Keep conversation, state, and expression-action schemas versioned and turn/generation correlated. Update the package README and tests with contract changes.
- The chat and emotion adapters never publish actuator commands. The mapper publishes bounded intentions; the safety bridge arbitrates and publishes the sole simulator input on `/emotion_bot/joy_out`.
- The simulation executable consumes `sensor_msgs/Joy`; do not claim `/cmd_vel` works unless a current end-to-end test proves it.
- Preserve exact-zero behavior on stale input, disable, shutdown, readiness loss, invalid data, or upstream failure.
- Normal emotion profiles publish zero x/y/yaw. Locomotion is an explicit supervised simulator-development mode, not a chat default.
- Joy/surprise hop and anger stomp actions must remain bounded, cancellation-aware, generation-ordered, and simulator-only. Do not add external Gazebo wrenches or hardware equivalents to the normal path.
- The integrated launch must not hand an interactive session back with motion disabled after readiness, health, and stance pass. Recover transient faults, re-enable through `/emotion_bot/set_motion_enabled`, and verify status.

## Runtime and coding constraints

- Build and run through the outer wrapper Makefile. Raw ROS commands must source `/opt/ros/noetic/setup.bash` and this workspace's `devel/setup.bash`.
- Keep C++ compatible with the existing C++14 configuration and ROS Python compatible with Noetic's Python 3.8.
- Avoid loading transformers, plotting, microphones, or network clients during package import. The adapter imports the headless `EmotionEngine`; live OpenAI runs in a separate Python 3.12 sidecar.
- Mock OpenAI/network behavior. Unit and integration tests must remain offline and deterministic.
- Preserve upstream topic types, model/joint naming, coordinate semantics, and controller startup behavior unless a compatibility migration is explicitly requested.
- `hardware_brain.launch` is reasoning/uplink orchestration only; it must not acquire MotionSDK or publish robot actuator commands. The physical runner, contact estimator, commissioning evidence, and per-emotion tickets live in the outer wrapper's `hardware-ws`, docs, and `tickets/` paths.
- Existing vendor real-robot SDK/executable code is not authorization to run or extend a hardware path. Do not run `example_lite3_real`, send UDP, or add an actuator-owning hardware launch without explicit current-task hardware direction.

## Change routing

- Emotional appraisal/personality/memory changes belong in `emotion-bot`.
- ROS JSON contracts, chat orchestration, mapping, arbitration, watchdogs, launch files, and package tests belong in `src/emotion_bot_ros`.
- Low-level simulated posture/action mechanics and their trajectory tests belong in `src/quadruped`.
- Docker, Make targets, OpenAI sidecar packaging, operator usage, and troubleshooting belong in the outer `lite3-noetic` wrapper.
- Document new nodes, topics, services, parameters, launch arguments, controller buttons, and recovery behavior in `src/emotion_bot_ros/README.md` plus affected wrapper docs.

## Verification

After catkin source changes, require a clean Release build in the container:

```bash
docker exec lite3-noetic-dev bash -lc \
  'cd /workspaces/lite3-noetic/ws/Lite3_VMC && source /opt/ros/noetic/setup.bash && catkin_make clean && catkin_make -j4 -DCMAKE_BUILD_TYPE=Release'
```

Use outer wrapper targets for focused and end-to-end checks:

```bash
make -C lite3-noetic emotion-unit-tests
make -C lite3-noetic emotion-ros-tests
make -C lite3-noetic emotion-gazebo-test
make -C lite3-noetic emotion-animation-gazebo-test
make -C lite3-noetic verify-emotion
```

For live animation claims, inspect Gazebo close-up and collect state, expression action, Joy output, model pose, and foot-contact telemetry. A log-only state transition is not visual proof. Always verify the final robot is stationary and the graph shuts down without controller errors.
