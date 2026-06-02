#!/usr/bin/env python3
"""
YAMNet ROS2 LifecycleNode — real-time bell/doorbell detection via microphone.

Audio capture: arecord subprocess (ALSA, no PortAudio needed)
Inference:     YAMNet (TF2) in background thread
Publisher:     ~/sound_detection  (SoundDetection)   — fires on every detection
Action server: ~/listen_for_sound (ListenForSound)   — blocks until detected or timeout

Typical use in a SMACH state:
    goal = ListenForSound.Goal()
    goal.timeout.sec = 30
    client.send_goal(goal)
    # → blocks until doorbell rings, returns detected=True/False
"""

import os
import re
import sys
import time
import threading
import subprocess
import collections

import numpy as np

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sobits_interfaces.msg import SoundDetection
from sobits_interfaces.action import ListenForSound

# Suppress TF startup noise before importing
os.environ.setdefault('TF_CPP_MIN_LOG_LEVEL', '3')
os.environ.setdefault('TF_ENABLE_ONEDNN_OPTS', '0')

# Make bundled YAMNet source importable
_YAMNET_SRC = os.path.join(os.path.dirname(__file__), 'yamnet_src')
sys.path.insert(0, _YAMNET_SRC)


def _is_bell_like(label: str, keywords: list[str]) -> bool:
    label_lower = label.lower()
    return any(
        re.search(r'\b' + re.escape(kw.lower()) + r'\b', label_lower)
        for kw in keywords
    )


class YamnetNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__('yamnet_ros')

        # Parameters — all overridable from config yaml or launch arguments
        self.declare_parameter('weights_path', '')            # '' = resolve via package share
        self.declare_parameter('sample_rate', 16000)          # YAMNet requires 16000; change only if you re-train
        self.declare_parameter('detection_threshold', 0.15)   # min YAMNet score to trigger
        self.declare_parameter('hop_secs', 0.5)               # inference interval (seconds)
        self.declare_parameter('window_secs', 1.0)            # sliding audio window (seconds)
        self.declare_parameter('audio_device', '')             # '' = auto-detect first capture device
        self.declare_parameter('audio_channels', 0)           # 0 = auto (2 for DMIC, 1 for analog)
        self.declare_parameter('cooldown_secs', 2.0)          # min gap between consecutive detections
        self.declare_parameter('target_labels', [             # keywords matched against YAMNet class names
            'doorbell', 'door bell',
            'chime', 'ding', 'dong', 'ding-dong',
            'bell', 'gong', 'jingle',
        ])

        self._model = None
        self._class_names: list[str] = []

        # Mic capture thread state
        self._capture_thread: threading.Thread | None = None
        self._running = False

        # Active action goal state (protected by _lock)
        self._lock = threading.Lock()
        self._active_goal = None
        self._detection_event: threading.Event | None = None
        self._detection_data: tuple | None = None  # (label, score)
        self._active_keywords: list[str] | None = None

    # ------------------------------------------------------------------ #
    # Lifecycle                                                            #
    # ------------------------------------------------------------------ #

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring yamnet_ros...')

        # Resolve weights path
        weights_path = self.get_parameter('weights_path').value
        if not weights_path:
            from ament_index_python.packages import get_package_share_directory
            weights_path = os.path.join(
                get_package_share_directory('yamnet_ros'),
                'weights', 'yamnet.h5'
            )

        if not os.path.isfile(weights_path):
            self.get_logger().error(f'yamnet.h5 not found at: {weights_path}')
            self.get_logger().error('Run  src/yamnet_ros/install.sh  to download weights.')
            return TransitionCallbackReturn.FAILURE

        from ament_index_python.packages import get_package_share_directory
        class_map = os.path.join(
            get_package_share_directory('yamnet_ros'), 'yamnet_src', 'yamnet_class_map.csv'
        )

        self.get_logger().info(f'Loading YAMNet weights: {weights_path}')
        try:
            self._model, self._class_names = _load_yamnet(weights_path, class_map)
        except Exception as exc:
            self.get_logger().error(f'Failed to load YAMNet: {exc}')
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(f'YAMNet ready ({len(self._class_names)} classes).')

        # Cache parameters
        self._sample_rate  = self.get_parameter('sample_rate').value
        self._threshold    = self.get_parameter('detection_threshold').value
        self._hop_secs     = self.get_parameter('hop_secs').value
        self._window_secs  = self.get_parameter('window_secs').value
        self._cooldown     = self.get_parameter('cooldown_secs').value
        self._keywords     = self.get_parameter('target_labels').value

        # Auto-detect ALSA device / channel count if not explicitly set
        device   = self.get_parameter('audio_device').value
        channels = self.get_parameter('audio_channels').value
        if not device:
            device, channels_detected = _detect_alsa_device()
            if not device:
                self.get_logger().error(
                    'No ALSA capture device found. Connect a microphone or set audio_device.'
                )
                return TransitionCallbackReturn.FAILURE
            self.get_logger().info(f'Auto-detected ALSA device: {device}  channels={channels_detected}')
            if channels == 0:
                channels = channels_detected
        elif channels == 0:
            channels = 1  # safe default for explicit device without explicit channels
        self._device   = device
        self._channels = channels

        # Publisher — latched-style (depth 10) for detection events
        self._pub = self.create_lifecycle_publisher(SoundDetection, '~/sound_detection', 10)

        # Action server — accepts goals at any time, but execution rejects if not active
        self._action_server = ActionServer(
            self,
            ListenForSound,
            '~/listen_for_sound',
            execute_callback=self._execute_action,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info('yamnet_ros configured.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating yamnet_ros (starting microphone)...')
        self._running = True
        self._capture_thread = threading.Thread(
            target=self._capture_loop, name='yamnet_capture', daemon=True
        )
        self._capture_thread.start()
        super().on_activate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating yamnet_ros (stopping microphone)...')
        self._running = False
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=3.0)
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._model = None
        self._class_names = []
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._running = False
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------ #
    # Audio capture + inference (background thread)                       #
    # ------------------------------------------------------------------ #

    def _capture_loop(self) -> None:
        sr             = self._sample_rate
        hop_samples    = int(sr * self._hop_secs)
        window_samples = int(sr * self._window_secs)
        chunk_bytes    = hop_samples * 2 * self._channels  # 16-bit per channel

        ring_buffer = collections.deque(
            np.zeros(window_samples, dtype=np.float32), maxlen=window_samples
        )

        arecord_cmd = [
            'arecord', '-D', self._device,
            '-f', 'S16_LE', '-r', str(sr),
            '-c', str(self._channels), '-',
        ]

        last_pub_time = 0.0

        try:
            proc = subprocess.Popen(
                arecord_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            self.get_logger().info(
                f'Microphone open: {self._device}  channels={self._channels}  '
                f'(ALSA direct — no desktop mic indicator, this is normal)'
            )

            while self._running:
                raw = proc.stdout.read(chunk_bytes)
                if not raw or len(raw) < chunk_bytes:
                    err = proc.stderr.read().decode(errors='replace').strip()
                    if err:
                        self.get_logger().error(
                            f'arecord exited unexpectedly on device "{self._device}":\n{err}\n'
                            f'Run  arecord -l  to list available devices and update '
                            f'the audio_device parameter.'
                        )
                    break

                # Decode S16_LE → float32 mono
                pcm = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
                chunk = pcm.reshape(-1, self._channels).mean(axis=1)

                ring_buffer.extend(chunk)
                waveform = np.array(ring_buffer, dtype=np.float32)

                with self._lock:
                    goal = self._active_goal
                    active_keywords = self._active_keywords

                mean_scores, top_indices, top_label, top_score = _run_inference(
                    self._model, self._class_names, waveform
                )
                hits = _select_hits(
                    self._class_names, mean_scores, top_indices,
                    self._threshold, self._keywords
                )
                action_hits = _select_hits(
                    self._class_names, mean_scores, top_indices,
                    self._threshold, active_keywords
                ) if active_keywords is not None else hits

                if action_hits:
                    with self._lock:
                        if self._active_goal is not None and self._detection_event is not None:
                            best_action_label, best_action_score = max(
                                action_hits, key=lambda x: x[1]
                            )
                            self._detection_data = (best_action_label, best_action_score)
                            self._detection_event.set()

                # Send action feedback every hop
                if goal is not None:
                    fb = ListenForSound.Feedback()
                    fb.current_top_label  = top_label
                    fb.current_top_score  = top_score
                    fb.candidate_detected = bool(action_hits)
                    try:
                        goal.publish_feedback(fb)
                    except Exception:
                        pass

                if not hits:
                    continue

                now = time.time()
                if (now - last_pub_time) < self._cooldown:
                    continue
                last_pub_time = now

                best_label, best_score = max(hits, key=lambda x: x[1])

                # Publish on topic
                msg = SoundDetection()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.label     = best_label
                msg.score     = best_score
                msg.top_label = top_label
                msg.top_score = top_score
                self._pub.publish(msg)

                self.get_logger().info(
                    f'[DETECTED] "{best_label}"  score={best_score:.3f}'
                )

        except Exception as exc:
            self.get_logger().error(f'Capture loop crashed: {exc}')
        finally:
            try:
                proc.terminate()
            except Exception:
                pass
            self.get_logger().info('Microphone closed.')

    # ------------------------------------------------------------------ #
    # Action server                                                        #
    # ------------------------------------------------------------------ #

    def _goal_callback(self, goal_request):
        if not self._running:
            self.get_logger().warn('Goal rejected: node is not active (lifecycle inactive).')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute_action(self, goal_handle) -> ListenForSound.Result:
        timeout = _duration_to_seconds(goal_handle.request.timeout)
        target_labels = list(goal_handle.request.target_labels)
        active_keywords = target_labels if target_labels else list(self._keywords)

        self.get_logger().info(
            f'ListenForSound started  timeout={timeout}s  '
            f'target_labels={active_keywords}  threshold={self._threshold}'
        )

        event = threading.Event()
        with self._lock:
            self._active_goal       = goal_handle
            self._detection_event   = event
            self._detection_data    = None
            self._active_keywords   = active_keywords

        start = time.time()

        try:
            while True:
                # Check cancellation
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('ListenForSound cancelled.')
                    goal_handle.canceled()
                    result = ListenForSound.Result()
                    result.detected      = False
                    _set_duration(result.elapsed_time, time.time() - start)
                    return result

                # Wait up to 0.2 s for a detection signal
                detected = event.wait(timeout=0.2)
                elapsed  = float(time.time() - start)

                if detected:
                    with self._lock:
                        data = self._detection_data
                    result = ListenForSound.Result()
                    result.detected     = True
                    result.label        = data[0] if data else ''
                    result.score        = float(data[1]) if data else 0.0
                    _set_duration(result.elapsed_time, elapsed)
                    self.get_logger().info(
                        f'ListenForSound succeeded: "{result.label}" score={result.score:.3f}'
                    )
                    goal_handle.succeed()
                    return result

                if timeout > 0.0 and elapsed >= timeout:
                    self.get_logger().info(
                        f'ListenForSound timed out after {elapsed:.1f}s.'
                    )
                    result = ListenForSound.Result()
                    result.detected     = False
                    _set_duration(result.elapsed_time, elapsed)
                    goal_handle.succeed()
                    return result
        finally:
            with self._lock:
                self._active_goal     = None
                self._detection_event = None
                self._detection_data  = None
                self._active_keywords = None


# ------------------------------------------------------------------ #
# Module-level helpers (stateless — easier to test)                   #
# ------------------------------------------------------------------ #

def _detect_alsa_device() -> tuple[str, int]:
    """Return (device, channels) for the first usable ALSA capture device."""
    try:
        out = subprocess.check_output(['arecord', '-l'], stderr=subprocess.DEVNULL,
                                      text=True, timeout=5)
    except Exception:
        return '', 0

    capture_devices: list[tuple[str, str, str]] = []
    for line in out.splitlines():
        # Matches lines like: "カード 1: PCH [...], デバイス 0: ALC257 Analog [...]"
        # or the English equivalent: "card 1: PCH [...], device 0: ALC257 Analog [...]"
        m = re.search(r'(?:card|カード)\s+(\d+).*?(?:device|デバイス)\s+(\d+)', line, re.IGNORECASE)
        if m:
            capture_devices.append((m.group(1), m.group(2), line.lower()))

    if not capture_devices:
        return '', 0

    candidates: list[tuple[str, int]] = []
    dmic16_devices = [
        dev for dev in capture_devices
        if 'dmic16khz' in dev[2]
    ]
    dmic_devices = [
        dev for dev in capture_devices
        if 'dmic' in dev[2] and dev not in dmic16_devices
    ]
    analog_devices = [
        dev for dev in capture_devices
        if dev not in dmic16_devices and dev not in dmic_devices
    ]

    for card, device_idx, _line in dmic16_devices + dmic_devices + analog_devices:
        is_dmic = (card, device_idx, _line) in dmic16_devices + dmic_devices
        preferred_channels = 2 if is_dmic else 1
        for alsa_name in (f'hw:{card},{device_idx}', f'plughw:{card},{device_idx}'):
            candidates.append((alsa_name, preferred_channels))
            candidates.append((alsa_name, 1 if preferred_channels == 2 else 2))

    # Try default only after concrete devices. In containers it is often present
    # as a name but not backed by a working PCM route.
    candidates.extend([('default', 1), ('default', 2)])

    seen: set[tuple[str, int]] = set()
    for device, channels in candidates:
        if (device, channels) in seen:
            continue
        seen.add((device, channels))
        if _probe_alsa_device(device, channels):
            return device, channels

    return '', 0


def _probe_alsa_device(device: str, channels: int) -> bool:
    cmd = [
        'arecord', '-D', device,
        '-f', 'S16_LE', '-r', '16000',
        '-c', str(channels), '-',
    ]
    try:
        subprocess.run(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=False,
            timeout=0.8,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return True
    except Exception:
        return False
    return False


def _load_yamnet(weights_path: str, class_map_path: str):
    import yamnet as yamnet_model
    import params as yamnet_params
    p = yamnet_params.Params()
    model = yamnet_model.yamnet_frames_model(p)
    model.load_weights(weights_path)
    names = []
    with open(class_map_path) as f:
        next(f)  # skip header
        for line in f:
            parts = line.strip().split(',')
            if len(parts) >= 3:
                names.append(parts[2].strip().strip('"'))
    return model, names


def _duration_to_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) / 1e9


def _set_duration(duration, seconds: float) -> None:
    seconds = max(0.0, float(seconds))
    whole_sec = int(seconds)
    duration.sec = whole_sec
    duration.nanosec = int(round((seconds - whole_sec) * 1e9))
    if duration.nanosec >= 1000000000:
        duration.sec += 1
        duration.nanosec -= 1000000000


def _run_inference(model, class_names: list, waveform: np.ndarray):
    """Returns (mean_scores, top_indices, top_label, top_score)."""
    scores, _, _ = model(waveform)
    mean_scores  = scores.numpy().mean(axis=0)
    top_indices  = np.argsort(mean_scores)[::-1][:10]

    top_label = class_names[top_indices[0]]
    top_score = float(mean_scores[top_indices[0]])
    return mean_scores, top_indices, top_label, top_score


def _select_hits(class_names: list, mean_scores: np.ndarray, top_indices: np.ndarray,
                 threshold: float, keywords: list[str]):
    """Returns hits = [(label, score), ...]."""
    hits = [
        (class_names[i], float(mean_scores[i]))
        for i in top_indices
        if _is_bell_like(class_names[i], keywords) and float(mean_scores[i]) >= threshold
    ]
    return hits


# ------------------------------------------------------------------ #
# Entry point                                                          #
# ------------------------------------------------------------------ #

def main(args=None) -> None:
    rclpy.init(args=args)
    node = YamnetNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
