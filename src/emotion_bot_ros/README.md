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
| `/emotion_bot/expression_action` | `std_msgs/String` | Generation-aware JSON `start`/`cancel` requests for `hop` or `stomp`; legacy plain action strings remain accepted. |
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

## Expression action contract 1.0

The mapper publishes compact JSON. An emotion change, stale state, or mapper shutdown emits `{"schema_version":"1.0","kind":"cancel","generation":N}`. An occurrence then emits `{"schema_version":"1.0","kind":"start","action":"stomp","emotion":"anger","generation":N,"occurrence_id":"idle:0:4"}`. The bridge rejects malformed or older generations, suppresses duplicate occurrence IDs, and preserves cancel-before-start order. Plain `hop` and `stomp` strings remain supported for existing tools.

Joy buttons 4 and 5 request the bounded hop and stomp inside the simulation controller. Button 6 is reserved internally for graceful cancellation and recovery, and button 7 is available only to the explicit locomotion-development path; neither internal button is forwarded from manual control.

## Fluid mapping

All nine expression profiles remain configurable under `/emotion_bot/mappings`. Each profile has an entrance `segments` list and a looping `idle_segments` list. A non-neutral category change cancels the old gesture, returns to exact zero for 0.25 seconds, holds the canonical stance for 0.35 seconds, then starts the new entrance from scratch. Same-category conversation appraisals update intensity without restarting the loop. Segment poses are keyframe endpoints joined by quintic smootherstep interpolation, giving zero velocity and acceleration at every keyframe and loop seam. Entrances use a global 1.25 amplitude gain; idle keyframes use 65% amplitude and 1.25x duration for softer recurring movement. `duration` and `speed` scale entrance/cadence, and `acceleration` scales mapper slew.

The fixed-rate controller adds:

- elapsed-time valence/arousal low-pass filters (`0.45`/`0.35` s);
- category changes with an exact-neutral reset before the new entrance;
- continuous same-category loops without duplicate action pulses from user/assistant appraisals;
- continuous arousal intensity scaling above a theatrical `0.82` floor;
- body-height slew limit `0.18 m/s` and roll/pitch slew limit `1.00 rad/s`;
- zero x/y/yaw commands for every normal chat expression.

The enlarged simulation envelope is ±0.100 m height and ±0.625 rad roll/pitch. Active neutral visibly breathes and tilts on a soft 1.375-second loop; stale state, disable, and shutdown reach exact zero. Joy starts with one full centered hop and repeats a 1.40-second high/low rock. Sadness holds a deep bow and slow 1.875-second sway. Anger starts with a forceful symmetric multi-impact stomp and repeats a stomp plus tense low rocking every 2.4375 seconds. Fear rapidly trembles in a full crouch, surprise performs a full hop into a tall pitched-back 1.60-second alert loop, disgust pulses down and away on a 1.90-second loop, curiosity alternates broad head/body tilts every 1.20 seconds, and affection uses a 2.75-second warm sway.

The controller-side 0.75-second stomp follows vertical offsets `-0.069`, `+0.100`, `-0.088`, `+0.063`, `-0.075`, `+0.044`, and `0.0` m, with a short smooth recovery. The 0.55-second hop uses offsets `-0.069`, `+0.100`, `-0.050`, and `0.0` m. Both actions use smooth controller-side interpolation, cap vertical speed at 1.125 m/s, and neither applies an external Gazebo wrench.

## Safety and manual arbitration

The integrated launch begins at zero, enters `JOY_STAND`, waits four simulated seconds for the stance transition, and then enables motion automatically. Every normal emotion profile has zero x/y/yaw, so joy cannot walk away while expressing itself. A simulation-only 40 Nm/rad HipX centering term keeps all four legs under the body instead of allowing contact forces to leave them crossed or splayed; physical-robot control is unchanged. The Gazebo model is not fixed: the bridge records its enable-time center, begins a four-foot recovery at 0.09 m, applies bounded 0.012 m incremental simulator corrections until it is within 0.035 m, then releases it; 0.20 m remains a hard safety boundary. Returning from an explicitly requested gait also guards the upstream transition before releasing a posture or queued action.

Manual posture axes are bounded by the same limits and manual activity has priority for 0.75 seconds. `run-emotion-keyboard` selects posture mode: w/s pitch, a/d roll, and q/e height. Locomotion remains available only for explicit development/manual commands; chat emotions stay centered. Emotion/manual commands expire after 0.5 seconds and state expires after 1.0 second. Completion, stale state, disable, readiness loss, and upstream node failure converge to exact neutral posture and zero velocity. There is no real executable, UDP bridge, or hardware address in this package.

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
make -C lite3-noetic emotion-animation-review
make -C lite3-noetic verify-emotion
```

`emotion-animation-review` launches the planted GUI stack and holds every emotion through at least two complete idle loops. Python unit tests cover interpolation seams, active-neutral/stale behavior, repeating occurrences, generation order, queued actions, gait transitions, and the documented natural-language sequence. The controller's C++ suite samples exact stomp/hop keyframes and verifies cancellation continuity, bounded velocity, moving-target recovery, and recovery interruption. The planted Gazebo diagnostic remains `emotion-gazebo-test`; `emotion-animation-gazebo-test` holds two idle cycles per emotion and checks entrance and idle movement, containment, joint margin, mirrored stomps, recovery, rapid joy→anger→fear changes, falls/NaNs/errors, residual motion, and the no-hardware boundary.
