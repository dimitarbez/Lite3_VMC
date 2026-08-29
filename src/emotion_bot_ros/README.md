# emotion_bot_ros

`emotion_bot_ros` is the simulation-only boundary between the real EmotionBot domain model and the DEEP Robotics Lite3 Gazebo controller. Emotional reasoning, mapping, and actuation are separate nodes. The adapter never publishes Joy, joint effort, motor data, or UDP.

## Runtime and licensing boundary

The package imports the sibling checkout mounted read-only at `/workspaces/emotion-bot`; it does not vendor or duplicate EmotionBot code. The recorded base commit is `34e38b86a3fc0c2fec7a6b84d7e82eae64d989f0`. EmotionBot and this integration package retain the GPL-3.0-only boundary declared in their license metadata. Lite3 upstream source remains in its own workspace and retains its own notices.

ROS Noetic supplies Python 3.8. EmotionBot’s complete pinned application requirements are newer, so the verified runtime uses its minimal headless core and deterministic appraisal backend. Optional Transformers imports and model creation occur only when `backend:=transformers` is explicitly requested; that backend is not installed, exercised, or supported by the offline baseline. OpenAI is also not imported or called by the adapter. Local responses use EmotionBot’s behavior and personality shaping.

The reusable integration API is `emotional_core.engine.EmotionEngine`. It owns `EmotionState`, personality, appraisal/update, conversation memory, local response generation, behavior shaping, and optional seeded randomness without importing the CLI, matplotlib, microphone code, or API-key startup checks.

## Nodes

| Node | Responsibility |
| --- | --- |
| `/emotion_bot/adapter` | Own one EmotionEngine session, accept text/events, validate and latch state, publish local responses and state heartbeats. |
| `/emotion_bot/expression_mapper` | Validate state and play the matching finite-duration Twist intention when sequence increases. A heartbeat refreshes freshness without restarting a pattern. |
| `/emotion_bot/safety_bridge` | Clamp emotion intentions, arbitrate manual Joy, enforce enable/readiness/watchdogs, publish safe status, and convert to the simulator’s Joy axes. |
| `/lite3_controller_loader` | One-shot controller-manager client that loads/starts the 12 effort controllers and joint-state controller without an interactive stdin spawner. |
| `/lite3_sim_runner` | Wait for the model/controllers, supervise `example_lite3_sim`, remap its `/joy` input, and stop its child on shutdown. |
| `emotion_chat.py` | Interactive ROS text client; no EmotionBot CLI automation. |
| `emotion_demo.py` | Deterministic multi-emotion demo with guaranteed disable on exit. |

## Topics and service

| Name | Type | Producer | Meaning |
| --- | --- | --- | --- |
| `/emotion_bot/input` | `std_msgs/String` | user/client | Natural text or exact `event:<emotion>`/`emotion:<emotion>`. |
| `/emotion_bot/state` | `std_msgs/String` | adapter | Latched, validated JSON state; heartbeat republishes the same sequence with a new timestamp. |
| `/emotion_bot/response` | `std_msgs/String` | adapter | Offline response for the submitted input. |
| `/emotion_bot/expression_cmd` | `geometry_msgs/Twist` | mapper | Finite behavior intention; only x, y, yaw are used. |
| `/emotion_bot/safe_cmd` | `geometry_msgs/Twist` | safety bridge | Selected, clamped intention for inspection. This is not the simulator transport. |
| `/emotion_bot/manual_joy` | `sensor_msgs/Joy` | keyboard/manual client | Manual input to arbitration; never publish the keyboard directly to the integrated `/joy` path. |
| `/emotion_bot/joy_out` | `sensor_msgs/Joy` | safety bridge | Sole input remapped into the Lite3 simulation controller. |
| `/emotion_bot/status` | `std_msgs/String` | safety bridge | Latched JSON with `motion_enabled`, `stale`, `backend`, `selected_source`, `last_safety_action`, `sim_ready`, and stamp. |
| `/emotion_bot/sim_controller_ready` | `std_msgs/Bool` | Lite3 sim executable | Latched internal-controller readiness used by the integrated safety interlock. |
| `/lite3_gazebo/joint_states` | `sensor_msgs/JointState` | Gazebo controller | Existing 12-joint simulation state. |

`/emotion_bot/set_motion_enabled` is `std_srvs/SetBool`. True is rejected by the integrated launch until simulation readiness is true. False immediately zeros output and schedules a stand pulse when needed.

## State contract 1.0

The state payload is compact JSON. Example:

```json
{"arousal":0.6,"backend":"deterministic","emotion":"joy","schema_version":"1.0","sequence":1,"source":"event","stamp":{"nsecs":938900000,"secs":1},"valence":0.7}
```

Validation requires a non-negative sequence, ROS `{secs,nsecs}` timestamp, non-empty backend/source, valence in `[-1,1]`, arousal in `[0,1]`, and exactly one of `neutral`, `joy`, `sadness`, `anger`, `fear`, `surprise`, `disgust`, `curiosity`, or `affection`. Malformed state is rejected and the mapper publishes zero.

## Default finite mappings

All values are configurable under `/emotion_bot/mappings` in `config/default.yaml`.

| Emotion | Pattern `(duration s: x, y, yaw)` |
| --- | --- |
| neutral | `0.5: 0, 0, 0` |
| joy | `3.0: +0.06, 0, 0`; `1.0: 0, 0, +0.04` |
| sadness | `2.0: -0.025, 0, 0` |
| anger | `1.2: +0.04, 0, +0.08`; `1.2: +0.04, 0, -0.08` |
| fear | `1.8: -0.04, 0, +0.05` |
| surprise | `1.0: 0, +0.04, 0`; `1.0: 0, -0.04, 0` |
| disgust | `2.0: 0, -0.03, -0.04` |
| curiosity | `1.5: +0.02, 0, +0.04`; `1.5: +0.02, 0, -0.04` |
| affection | `2.5: +0.03, +0.02, +0.025` |

A segment duration must be finite and in `(0,10]`. Completion yields zero; patterns do not loop.

## Parameters

`config/default.yaml` defines all runtime values below the `/emotion_bot` namespace:

- `topics/*`: every topic listed above.
- `services/set_motion_enabled`: enable/disable service name.
- `runtime/emotion_bot_path`, `backend`, `personality`, `seed`, `randomness_enabled`, `heartbeat_rate`.
- `mapper/publish_rate`, `state_timeout`.
- `safety/motion_enabled`, `require_sim_ready`, `publish_rate`, `expression_timeout`, `manual_timeout`, `manual_priority_hold`, `mode_transition_delay` (1.5 s by default so stand completes before automatic locomotion).
- `safety/limits/linear_x`, `linear_y`, `angular_z`.
- `mappings/<emotion>/segments`: finite mapping values.

`core.launch` exposes arguments for checkout path, backend, personality, seed, and initial enable flag. `integrated_sim.launch` additionally exposes `gui`, `headless`, `world`, and `motion_enabled`, and forces `require_sim_ready=true`.

## Safety and arbitration

Initial output is zero and motion is disabled. Emotion commands are clamped to ±0.10 m/s x, ±0.05 m/s y, and ±0.10 rad/s yaw. NaN/Inf becomes zero. Unsupported Twist axes remain zero.

The bridge converts x to Joy axis 4 using the simulator’s 0.2 scale, y to axis 3 using 0.1, and yaw to axis 0 using 0.2. It sends B/stand before X/locomotion for emotion motion. Manual input preserves only axes 0/3/4 and buttons 1/2/3/5, clamps axes to ±1, and takes priority for a bounded hold interval.

Expired emotion/manual input, neutral/completed patterns, disable, readiness loss, mapper/adapter loss, and shutdown select zero and leave locomotion through the B/stand transition. Shutdown publishes zero Twist and a zero Joy carrying the stand button. Hardware is outside this package’s scope.

## Launch and tests

From the repository root:

```bash
make -C lite3-noetic run-emotion-sim
make -C lite3-noetic run-emotion-chat
make -C lite3-noetic emotion-demo EMOTION_MOTION=true
make -C lite3-noetic verify-emotion
```

`test/test_unit.py` covers contracts, all mappings, bounds, clamping, timeouts, defaults, shutdown-equivalent disable, and deterministic behavior. `test/emotion_bot_core.test` covers headless ROS publication, latching, enable, limits, watchdog, manual priority, and adapter loss. `test_gazebo_e2e.py` launches the complete stack, confirms nodes/services/controllers/joints, measures torso displacement and standing height through Gazebo, verifies stop/watchdog/manual paths, captures teardown logs, and rejects real executable, hardware bridge, or motion-host UDP use.
