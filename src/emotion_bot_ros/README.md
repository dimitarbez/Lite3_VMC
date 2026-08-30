# emotion_bot_ros

`emotion_bot_ros` is the simulation-only boundary between the sibling EmotionBot domain model, conversational backends, and the DEEP Robotics Lite3 Gazebo controller. Conversation, emotional reasoning, expression mapping, and actuation safety remain separate. Neither the chat node nor EmotionBot publishes Joy, joint effort, motor data, or UDP.

## Runtime and source boundary

The package imports the read-only `/workspaces/emotion-bot` checkout at recorded commit `20c0c1361434bcf9ebaec4e8a5c9385e61c9c3e2`; it does not vendor or duplicate that GPL-3.0 code. The reusable API is `emotional_core.engine.EmotionEngine`, which owns state, personality, appraisal/update, memory, local response shaping, and seeded randomness without importing the interactive CLI, matplotlib, microphone, or startup key check.

ROS Noetic remains on Python 3.8. The default verified backend is deterministic and headless. Live chat uses a separate `lite3-openai-runtime:local` Python 3.12 image pinned to the official `openai==3.6.0` SDK. The loopback bridge owns the Responses API call; the secret never enters ROS or the mounted workspace.

## Nodes

| Node | Responsibility |
| --- | --- |
| `/emotion_bot/chat_adapter` | Own bounded context and turn ordering; publish immediate acceptance, streamed deltas, retry/fallback, cancellation, and exactly one current-turn completion. |
| `/emotion_bot/adapter` | Feed accepted user/current assistant events into one `EmotionEngine`; validate and latch contract 1.1 state with turn correlation. |
| `/emotion_bot/expression_mapper` | Filter affect, gate category changes, blend/rate-limit the mapped expression, and return smoothly to neutral. |
| `/emotion_bot/safety_bridge` | Clamp emotion/manual intentions, enforce enable/readiness/watchdogs, arbitrate manual priority, publish status, and generate the simulator Joy input. |
| `/lite3_controller_loader` | Load/start 12 effort controllers plus joint-state controller without stdin. |
| `/lite3_sim_runner` | Wait for readiness, supervise `example_lite3_sim`, remap its sole Joy input, and stop it on shutdown. |
| `emotion_chat.py` | Interactive streaming client with turn-correlated response/state display. |
| `emotion_demo.py` | Deterministic multi-emotion demo with guaranteed disable on exit. |

The integrated safety bridge is a required roslaunch node. Its failure tears down the stack instead of allowing the simulation controller to persist with an old input.

## Topics and service

| Name | Type | Meaning |
| --- | --- | --- |
| `/emotion_bot/chat/input` | `std_msgs/String` | Plain text or JSON `{"turn_id":"...","text":"..."}`. |
| `/emotion_bot/chat/cancel` | `std_msgs/String` | Cancel the active turn; empty data means current turn. |
| `/emotion_bot/chat/events` | `std_msgs/String` | JSON lifecycle/stream events for clients. |
| `/emotion_bot/chat/response` | `std_msgs/String` | JSON final response correlated by turn ID/index. |
| `/emotion_bot/conversation/events` | `std_msgs/String` | Semantic accepted/completed/cancel/error feed consumed by EmotionBot. |
| `/emotion_bot/input` | `std_msgs/String` | Compatibility input for natural text or exact `event:<emotion>`. |
| `/emotion_bot/state` | `std_msgs/String` | Latched validated JSON state; heartbeat keeps the sequence and turn ID. |
| `/emotion_bot/response` | `std_msgs/String` | Compatibility local response for `/emotion_bot/input`. |
| `/emotion_bot/expression_cmd` | `geometry_msgs/Twist` | Fluid expression intention; normal profiles use height (`linear.z`), roll (`angular.x`), and pitch (`angular.y`) while standing. |
| `/emotion_bot/expression_action` | `std_msgs/String` | Optional one-shot mapper request: `hop` or `stomp`; it is rejected by the default planted-expression configuration and remains available only for supervised experiments. |
| `/emotion_bot/safe_cmd` | `geometry_msgs/Twist` | Selected and clamped intention for inspection, not simulator transport. |
| `/emotion_bot/manual_joy` | `sensor_msgs/Joy` | Manual arbitration input. |
| `/emotion_bot/joy_out` | `sensor_msgs/Joy` | Sole Joy input remapped to the Lite3 simulation controller. |
| `/emotion_bot/status` | `std_msgs/String` | Latched safety/arbitration status. |
| `/emotion_bot/sim_controller_ready` | `std_msgs/Bool` | Latched internal-controller readiness. |
| `/lite3_gazebo/joint_states` | `sensor_msgs/JointState` | Existing 12-joint Gazebo state. |

`/emotion_bot/set_motion_enabled` is `std_srvs/SetBool`. True is rejected until simulation readiness in the integrated launch. False immediately zeros output and schedules stand when needed.

## Conversation event contract 1.0

Every event contains `schema_version`, floating-point `stamp`, non-empty `turn_id`, positive `turn_index`, and one of:

- `accepted`: immediate user text acknowledgement;
- `started`: backend selected;
- `delta`: an assistant text fragment, published promptly;
- `retrying`: bounded retry notice;
- `offline_fallback`: deterministic backend selected after live failure;
- `completed`: exactly one final assistant text for the current turn;
- `cancelled`: explicit or superseding cancellation;
- `error`: generic terminal error without provider/secret details.

The coordinator retains at most 6 turns/6000 characters, accepts at most 2000 input characters, produces at most 6000 response characters, and limits the sidecar request to 300 output tokens. It waits up to 0.5 seconds for the accepted user's same-turn emotion before starting the backend. Starting a newer turn sets the old cancellation token before its acceptance event. The last accepted turn index is retained on the ROS parameter server so a chat-node restart remains monotonic within the running graph. The adapter's `TurnGate` ignores late, cancelled, duplicate, or out-of-order completion.

## Emotion state contract 1.1

Example compact JSON:

```json
{"arousal":0.6,"backend":"deterministic","emotion":"joy","schema_version":"1.1","sequence":1,"source":"user","stamp":{"nsecs":938900000,"secs":1},"turn_id":"turn-000001","valence":0.7}
```

Validation requires a non-negative sequence, ROS timestamp, non-empty backend/source/turn ID, valence in `[-1,1]`, arousal in `[0,1]`, and exactly one of `neutral`, `joy`, `sadness`, `anger`, `fear`, `surprise`, `disgust`, `curiosity`, or `affection`. Malformed state forces a neutral mapper output.

## Live OpenAI path

The sidecar binds `127.0.0.1:8765`. `/v1/stream` accepts only bounded JSON and emits NDJSON deltas/done. It calls `client.responses.create` using `gpt-5-mini`, `stream=true`, `store=false`, minimal reasoning, low verbosity, a 20-second timeout, and SDK retries disabled. The ROS coordinator retries once, then uses the deterministic backend. Cancellation closes the upstream stream when possible; a timeout bounds an unresponsive request. Errors crossing either boundary are generic.

For persistent local use, put the credential in the workspace root `.env` as `OPENAI_API_KEY=...`; the file is Git-ignored and should remain mode `600`. The wrapper sources it automatically for `start-openai-bridge`. Docker passes only the variable name with `--env OPENAI_API_KEY` (no value in arguments) and starts the sidecar with `--rm`; `/health` exposes only booleans for key presence/test mode.

## Fluid mapping

All nine expression profiles remain configurable under `/emotion_bot/mappings`. Each profile has an entrance `segments` list and a looping `idle_segments` list: a state change immediately interrupts/blends into the new entrance, then repeats the idle body language until another state, timeout, disable, or shutdown. Every emotion exposes `intensity`, `duration`, `speed`, `acceleration`, `cooldown`, `variation`, and `transition_blend` alongside those segment lists. `duration` and `speed` scale the entrance/cadence; `acceleration` scales mapper slew only; `cooldown` prevents a repeated state from restarting too rapidly; and deterministic `variation` prevents a held profile from freezing into a statue.

The fixed-rate controller adds:

- elapsed-time valence/arousal low-pass filters (`0.45`/`0.35` s);
- immediate category interruption with smoothstep profile-specific transition blending (and a `0.35` s neutral return);
- bounded same-category replay after the profile cooldown, so repeated emotional events read without rapid-trigger chatter;
- continuous arousal intensity scaling above a theatrical `0.55` floor;
- body-height slew limit `0.25 m/s` and roll/pitch slew limit `1.50 rad/s`;
- optional x/y slew limit `0.18 m/s²` and yaw slew limit `0.30 rad/s` for supervised locomotion experiments.

Segment values use a deliberately theatrical simulation posture envelope: body height is bounded to ±0.070 m and roll/pitch to ±0.50 rad; normal negative segments stop at -0.055 m to preserve crouch clearance. Neutral settles quietly to exact zero, joy anticipates then bounces/rocks, sadness lowers and bows, anger makes a planted stomp-like brace and tense sway, fear recoils into a trembling crouch, surprise recoils then springs into alertness, disgust leans away, curiosity tilts between sides, and affection sways warmly. Segment duration must be finite and in `(0,10]`. Segments may declare the optional simulation-only action `hop` or `stomp`; no other actions are valid. Default profiles intentionally do not use them because the upstream discrete actions accumulated planar drift during Gazebo validation.

## Safety and manual arbitration

Initial output and initial motion permission are zero/false. After the simulation controller reports ready, the integrated launch enters `JOY_STAND` once and waits four simulated seconds for the upstream stance transition to settle before motion can be enabled. Normal expressions then keep four feet planted, clamp body-height offset to ±0.070 m and roll/pitch to ±0.50 rad, and rate-limit them to 0.25 m/s and 1.50 rad/s. Optional `hop`/`stomp` actions are gated off by default because they were observed to accumulate planar drift; they never select a gait or publish x/y/yaw when explicitly enabled for a supervised experiment. Translational and yaw locomotion are forced to exact zero while `allow_locomotion` is false, preventing the upstream trot controller's accumulated drift and NaN-prone repeated gait transitions. Integrated simulation also monitors all 12 joint states, finite model pose, torso height, roll/pitch, and a 0.040 m planar-displacement limit from the enable point; a violation or a 0.5-second monitoring gap disables motion immediately and is reported in `/emotion_bot/status`.

Manual posture axes are bounded by the same limits and manual activity has priority for 0.75 seconds. `run-emotion-keyboard` selects posture mode: w/s pitch, a/d roll, and q/e height. Locomotion axes and X are blocked by default. Setting `allow_locomotion: true` restores the former supervised B/X/directional path, including one-shot mode buttons, settling delay, bounded velocity, slew limits, and automatic stand timeout. Emotion/manual commands expire after 0.5 seconds and state expires after 1.0 second. Completion, stale state, disable, readiness loss, and upstream node failure converge to exact neutral posture and zero velocity. There is no real executable, UDP bridge, or hardware address in this package.

## Parameters

`config/default.yaml` is the source of truth for every topic/service and these groups:

- `runtime`: EmotionBot path/backend/personality/seed/randomness/heartbeat;
- `chat`: backend/model/bridge endpoint, timeout, token/context/input/response bounds, retries, fallback;
- `mapper`: publish rate, state timeout, affect filters, blend/neutral timing, dwell/hysteresis, slew rates;
- `safety`: initial enable, readiness, publish/watchdog/priority/mode timing, final velocity limits;
- `mappings`: all nine declarative segment lists.

`core.launch` exposes EmotionBot and chat backend/model arguments. `integrated_sim.launch` also exposes GUI/headless/world/motion and forces the simulation readiness interlock.

## Launch and tests

```bash
make -C lite3-noetic run-emotion-sim
make -C lite3-noetic run-emotion-chat
make -C lite3-noetic emotion-demo EMOTION_MOTION=true
make -C lite3-noetic verify-emotion
```

Unit tests cover state/event contracts, every profile's parameters, sustained idle loops, filtering/blending/cooldowns/rates, rapid interruption, neutral return, bounds, timeout/fallback/cancellation/late results, default disable, one-shot manual buttons, and safety arbitration. ROS integration covers deterministic streaming and turn-correlated state plus enable/limits/watchdogs/manual/adapter/mapper loss. The sidecar test crosses the real HTTP process boundary without a key. Gazebo E2E drives all nine profiles, measures each transition and idle movement, tests rapid/repeated changes, validates physical/manual motion and stops, checks standing height/teardown, and rejects real-hardware paths.
