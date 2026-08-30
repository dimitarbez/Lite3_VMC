"""ROS-independent, cancellable streaming conversation coordination."""

from __future__ import annotations

from collections import deque
import json
import threading
import time
import urllib.request
from typing import Any, Callable, Deque, Dict, Iterable, List, Optional


class ConversationError(RuntimeError):
    pass


CONVERSATION_EVENT_TYPES = (
    "accepted",
    "started",
    "delta",
    "retrying",
    "offline_fallback",
    "completed",
    "cancelled",
    "error",
)


def validate_conversation_event(event: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(event, dict) or event.get("schema_version") != "1.0":
        raise ConversationError("invalid conversation event schema")
    if event.get("type") not in CONVERSATION_EVENT_TYPES:
        raise ConversationError("invalid conversation event type")
    if not isinstance(event.get("turn_id"), str) or not event["turn_id"]:
        raise ConversationError("conversation event requires turn_id")
    index = event.get("turn_index")
    if isinstance(index, bool) or not isinstance(index, int) or index < 1:
        raise ConversationError("conversation event requires positive turn_index")
    stamp = event.get("stamp")
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)):
        raise ConversationError("conversation event requires timestamp")
    if event["type"] in ("accepted", "completed", "delta"):
        if not isinstance(event.get("text"), str) or not event["text"]:
            raise ConversationError("conversation text event requires text")
    return event


def loads_conversation_event(payload: str) -> Dict[str, Any]:
    import json

    try:
        event = json.loads(payload)
    except (TypeError, ValueError) as exc:
        raise ConversationError("malformed conversation event") from exc
    return validate_conversation_event(event)


class TurnGate:
    """Reject late, cancelled, duplicate, and out-of-order emotion events."""

    def __init__(self):
        self.active_turn_id: Optional[str] = None
        self.active_turn_index = 0
        self.cancelled = set()
        self.completed = set()

    def accepts(self, event: Dict[str, Any]) -> bool:
        validate_conversation_event(event)
        event_type = event["type"]
        turn_id = event["turn_id"]
        turn_index = event["turn_index"]
        if event_type == "accepted":
            if turn_index <= self.active_turn_index or turn_id in self.cancelled or turn_id in self.completed:
                return False
            self.active_turn_id = turn_id
            self.active_turn_index = turn_index
            return True
        if event_type == "cancelled":
            self.cancelled.add(turn_id)
            return False
        if event_type != "completed":
            return False
        if turn_id in self.cancelled or turn_id in self.completed:
            return False
        if turn_id != self.active_turn_id or turn_index != self.active_turn_index:
            return False
        self.completed.add(turn_id)
        return True


class DeterministicChatBackend:
    name = "deterministic"

    def stream(
        self,
        text: str,
        history: List[Dict[str, str]],
        emotion: str,
        cancel: threading.Event,
    ) -> Iterable[str]:
        del history
        prefix = {
            "joy": "That sounds genuinely exciting.",
            "sadness": "That sounds difficult, and I’m listening.",
            "anger": "I can see why that would feel frustrating.",
            "fear": "That uncertainty makes sense.",
            "surprise": "That is quite a surprise.",
            "disgust": "That sounds deeply unpleasant.",
            "curiosity": "That is interesting—let’s explore it.",
            "affection": "I appreciate you sharing that.",
            "neutral": "I’m with you.",
        }.get(emotion, "I’m with you.")
        suffix = " What part would you like to focus on next?" if text.rstrip().endswith("?") else " Tell me what matters most about it."
        for chunk in (prefix, suffix):
            if cancel.is_set():
                return
            yield chunk


class OpenAIResponsesBackend:
    """Streaming client for the localhost official-SDK sidecar."""

    name = "openai"

    def __init__(self, model: str, timeout: float, max_output_tokens: int, endpoint: str):
        self.model = model
        self.timeout = float(timeout)
        self.max_output_tokens = int(max_output_tokens)
        self.endpoint = str(endpoint)

    def stream(
        self,
        text: str,
        history: List[Dict[str, str]],
        emotion: str,
        cancel: threading.Event,
    ) -> Iterable[str]:
        payload = json.dumps(
            {
                "model": self.model,
                "text": text,
                "history": history,
                "emotion": emotion,
                "max_output_tokens": self.max_output_tokens,
                "timeout": self.timeout,
            },
            separators=(",", ":"),
        ).encode("utf-8")
        request = urllib.request.Request(
            self.endpoint,
            data=payload,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        stream = urllib.request.urlopen(request, timeout=self.timeout)
        try:
            for line in stream:
                if cancel.is_set():
                    return
                try:
                    event = json.loads(line.decode("utf-8"))
                except (UnicodeError, ValueError) as exc:
                    raise ConversationError("invalid OpenAI bridge event") from exc
                if event.get("type") == "delta" and event.get("text"):
                    yield str(event["text"])
                elif event.get("type") == "error":
                    raise ConversationError("OpenAI bridge request failed")
                elif event.get("type") == "done":
                    return
        finally:
            stream.close()


class ConversationCoordinator:
    """Own bounded context and ensure only the current turn can complete."""

    def __init__(
        self,
        backend: Any,
        emit: Callable[[Dict[str, Any]], None],
        fallback_backend: Optional[Any] = None,
        max_turns: int = 6,
        max_context_chars: int = 6000,
        max_input_chars: int = 2000,
        max_response_chars: int = 6000,
        max_retries: int = 1,
        retry_delay: float = 0.25,
        emotion_sync_timeout: float = 0.0,
        initial_turn_index: int = 0,
        clock: Callable[[], float] = time.time,
    ):
        self.backend = backend
        self.fallback_backend = fallback_backend
        self.emit = emit
        self.max_turns = max(1, int(max_turns))
        self.max_context_chars = max(128, int(max_context_chars))
        self.max_input_chars = max(1, int(max_input_chars))
        self.max_response_chars = max(1, int(max_response_chars))
        self.max_retries = max(0, int(max_retries))
        self.retry_delay = max(0.0, float(retry_delay))
        self.emotion_sync_timeout = max(0.0, float(emotion_sync_timeout))
        self.clock = clock
        self._history: Deque[Dict[str, str]] = deque()
        self._lock = threading.RLock()
        self._emotion_condition = threading.Condition(self._lock)
        self._active_cancel: Optional[threading.Event] = None
        self._active_thread: Optional[threading.Thread] = None
        self._active_turn_id: Optional[str] = None
        self._turn_index = max(0, int(initial_turn_index))
        self._used_turn_ids = set()
        self._emotion = "neutral"
        self._emotion_turn_id: Optional[str] = None
        self._shutdown = False

    def set_emotion(self, emotion: str, turn_id: Optional[str] = None) -> None:
        with self._emotion_condition:
            self._emotion = str(emotion or "neutral")
            self._emotion_turn_id = str(turn_id) if turn_id else None
            self._emotion_condition.notify_all()

    def _event(self, turn_id: str, turn_index: int, event_type: str, **fields: Any) -> Dict[str, Any]:
        event = {
            "schema_version": "1.0",
            "stamp": float(self.clock()),
            "turn_id": turn_id,
            "turn_index": int(turn_index),
            "type": event_type,
        }
        event.update(fields)
        return event

    def _emit(self, event: Dict[str, Any]) -> None:
        self.emit(event)

    def _bounded_history(self) -> List[Dict[str, str]]:
        with self._lock:
            result = list(self._history)[-(self.max_turns * 2) :]
        total = 0
        bounded = []
        for item in reversed(result):
            total += len(item["content"])
            if total > self.max_context_chars:
                break
            bounded.append(dict(item))
        return list(reversed(bounded))

    def submit(self, text: str, turn_id: Optional[str] = None) -> str:
        text = str(text).strip()
        if not text:
            raise ConversationError("chat input must not be empty")
        if len(text) > self.max_input_chars:
            raise ConversationError("chat input exceeds configured limit")
        with self._lock:
            if self._shutdown:
                raise ConversationError("conversation coordinator is shut down")
            previous_id = self._active_turn_id
            previous_cancel = self._active_cancel
            self._turn_index += 1
            turn_index = self._turn_index
            if turn_id is None or not str(turn_id).strip():
                turn_id = "turn-%06d" % turn_index
                suffix = 1
                while turn_id in self._used_turn_ids:
                    turn_id = "turn-%06d-%d" % (turn_index, suffix)
                    suffix += 1
            else:
                turn_id = str(turn_id).strip()
                if len(turn_id) > 128:
                    raise ConversationError("turn_id exceeds configured limit")
                if turn_id in self._used_turn_ids:
                    raise ConversationError("turn_id has already been used")
            self._used_turn_ids.add(turn_id)
            cancel = threading.Event()
            self._active_cancel = cancel
            self._active_turn_id = turn_id
            history = self._bounded_history()
            if previous_cancel is not None and not previous_cancel.is_set():
                previous_cancel.set()
                if previous_id:
                    self._emit(self._event(previous_id, turn_index - 1, "cancelled", reason="superseded"))
            self._emit(self._event(turn_id, turn_index, "accepted", role="user", text=text))
            thread = threading.Thread(
                target=self._run_turn,
                args=(turn_id, turn_index, text, history, cancel),
                name="emotion-chat-%d" % turn_index,
                daemon=True,
            )
            self._active_thread = thread
        thread.start()
        return turn_id

    def _is_current(self, turn_id: str, cancel: threading.Event) -> bool:
        with self._lock:
            return not self._shutdown and self._active_turn_id == turn_id and not cancel.is_set()

    def _run_backend(self, backend: Any, text: str, history: List[Dict[str, str]], emotion: str, cancel: threading.Event):
        return backend.stream(text, history, emotion, cancel)

    def _wait_for_turn_emotion(self, turn_id: str, cancel: threading.Event) -> str:
        deadline = time.monotonic() + self.emotion_sync_timeout
        with self._emotion_condition:
            while self._emotion_turn_id != turn_id and self._is_current(turn_id, cancel):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._emotion_condition.wait(remaining)
            return self._emotion

    def _run_turn(self, turn_id: str, turn_index: int, text: str, history: List[Dict[str, str]], cancel: threading.Event) -> None:
        emotion = self._wait_for_turn_emotion(turn_id, cancel)
        with self._lock:
            if not self._is_current(turn_id, cancel):
                return
            self._emit(self._event(turn_id, turn_index, "started", backend=self.backend.name))
        selected_backend = self.backend
        chunks: List[str] = []
        completed = False
        for attempt in range(self.max_retries + 1):
            try:
                chunks = []
                for delta in self._run_backend(selected_backend, text, history, emotion, cancel):
                    if not self._is_current(turn_id, cancel):
                        return
                    remaining = self.max_response_chars - sum(len(item) for item in chunks)
                    if remaining <= 0:
                        break
                    delta = str(delta)[:remaining]
                    chunks.append(delta)
                    self._emit(self._event(turn_id, turn_index, "delta", role="assistant", text=delta))
                completed = self._is_current(turn_id, cancel)
                if completed:
                    break
            except Exception:
                if not self._is_current(turn_id, cancel):
                    return
                if chunks:
                    self._emit_error(turn_id, turn_index, cancel, "chat stream interrupted")
                    return
                if attempt < self.max_retries:
                    self._emit(self._event(turn_id, turn_index, "retrying", attempt=attempt + 1))
                    cancel.wait(self.retry_delay * (attempt + 1))
                    continue
                if self.fallback_backend is not None and selected_backend is not self.fallback_backend:
                    selected_backend = self.fallback_backend
                    self._emit(self._event(turn_id, turn_index, "offline_fallback", backend=selected_backend.name))
                    try:
                        chunks = []
                        for delta in self._run_backend(selected_backend, text, history, emotion, cancel):
                            if not self._is_current(turn_id, cancel):
                                return
                            remaining = self.max_response_chars - sum(len(item) for item in chunks)
                            if remaining <= 0:
                                break
                            delta = str(delta)[:remaining]
                            chunks.append(delta)
                            self._emit(self._event(turn_id, turn_index, "delta", role="assistant", text=delta))
                        completed = self._is_current(turn_id, cancel)
                    except Exception:
                        completed = False
                if not completed:
                    self._emit_error(turn_id, turn_index, cancel, "chat backend unavailable")
                    return
                break
        if not completed:
            return
        response = "".join(chunks).strip()
        if not response:
            self._emit_error(turn_id, turn_index, cancel, "chat backend returned no text")
            return
        with self._lock:
            if not self._is_current(turn_id, cancel):
                return
            self._history.append({"role": "user", "content": text})
            self._history.append({"role": "assistant", "content": response})
            while len(self._history) > self.max_turns * 2:
                self._history.popleft()
            cancel.set()
            self._emit(self._event(
                turn_id,
                turn_index,
                "completed",
                role="assistant",
                text=response,
                backend=selected_backend.name,
            ))

    def _emit_error(self, turn_id: str, turn_index: int, cancel: threading.Event, message: str) -> None:
        with self._lock:
            if self._active_turn_id != turn_id or self._active_cancel is not cancel or cancel.is_set():
                return
            cancel.set()
            self._emit(self._event(turn_id, turn_index, "error", message=message))

    def cancel(self, turn_id: Optional[str] = None, reason: str = "requested") -> bool:
        with self._lock:
            if self._active_cancel is None or self._active_cancel.is_set():
                return False
            if turn_id and turn_id != self._active_turn_id:
                return False
            active_id = self._active_turn_id
            active_index = self._turn_index
            self._active_cancel.set()
            self._emotion_condition.notify_all()
            if active_id:
                self._emit(self._event(active_id, active_index, "cancelled", reason=reason))
        return True

    def shutdown(self, join_timeout: float = 1.0) -> None:
        with self._lock:
            self._shutdown = True
            cancel = self._active_cancel
            thread = self._active_thread
            if cancel is not None:
                cancel.set()
            self._emotion_condition.notify_all()
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.0, join_timeout))
