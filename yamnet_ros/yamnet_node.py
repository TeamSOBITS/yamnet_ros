#!/usr/bin/env python3
"""
YAMNet ROS2 LifecycleNode — real-time bell/doorbell detection via microphone.

Audio capture: arecord subprocess (ALSA, no PortAudio needed)
Inference:     YAMNet (TF2) in background thread
Publisher:     ~/sound_detection  (SoundDetection)   — fires on every detection
Action server: ~/listen_for_sound (ListenForSound)   — blocks until detected or timeout

Typical use in a SMACH state:
    client.send_goal(ListenForSound.Goal(timeout_sec=30.0))
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
        re.search(r'\b' + re.escape(kw) + r'\b', label_lower)
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

                # Determine effective threshold (may be overridden by active goal)
                with self._lock:
                    goal = self._active_goal

                hits, top_label, top_score = _run_inference(
                    self._model, self._class_names, waveform,
                    self._threshold, self._keywords
                )

                # Send action feedback every hop
                if goal is not None:
                    fb = ListenForSound.Feedback()
                    fb.current_top_label  = top_label
                    fb.current_top_score  = top_score
                    fb.candidate_detected = bool(hits)
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

                # Signal active action goal
                with self._lock:
                    if self._active_goal is not None and self._detection_event is not None:
                        self._detection_data = (best_label, best_score)
                        self._detection_event.set()

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
        timeout   = goal_handle.request.timeout_sec
        threshold = goal_handle.request.threshold
        if threshold > 0.0:
            self._threshold = threshold  # override node default for this session

        self.get_logger().info(
            f'ListenForSound started  timeout={timeout}s  threshold={self._threshold}'
        )

        event = threading.Event()
        with self._lock:
            self._active_goal       = goal_handle
            self._detection_event   = event
            self._detection_data    = None

        start = time.time()

        try:
            while True:
                # Check cancellation
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('ListenForSound cancelled.')
                    goal_handle.canceled()
                    result = ListenForSound.Result()
                    result.detected      = False
                    result.elapsed_time  = float(time.time() - start)
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
                    result.elapsed_time = elapsed
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
                    result.elapsed_time = elapsed
                    goal_handle.succeed()
                    return result
        finally:
            with self._lock:
                self._active_goal     = None
                self._detection_event = None
                self._detection_data  = None


# ------------------------------------------------------------------ #
# Module-level helpers (stateless — easier to test)                   #
# ------------------------------------------------------------------ #

def _detect_alsa_device() -> tuple[str, int]:
    """Return (device, channels) for the first available ALSA capture device.

    Parses `arecord -l` output. DMIC subdevices (subdevice index > 0 or
    known DMIC names) get 2 channels; analog mics get 1.
    Returns ('', 0) if no device is found.
    """
    try:
        out = subprocess.check_output(['arecord', '-l'], stderr=subprocess.DEVNULL,
                                      text=True, timeout=5)
    except Exception:
        return '', 0

    card = device_idx = None
    channels = 1
    for line in out.splitlines():
        # Matches lines like: "カード 1: PCH [...], デバイス 0: ALC257 Analog [...]"
        # or the English equivalent: "card 1: PCH [...], device 0: ALC257 Analog [...]"
        m = re.search(r'(?:card|カード)\s+(\d+).*?(?:device|デバイス)\s+(\d+)', line, re.IGNORECASE)
        if m:
            card, device_idx = m.group(1), m.group(2)
            # DMIC cards typically have "DMIC" or high subdevice counts in the name
            if 'dmic' in line.lower():
                channels = 2
            else:
                channels = 1
            break  # use the first capture device found

    if card is None:
        return '', 0
    # DMIC (stereo) → hw: directly; it supports 16 kHz natively and is not
    # held by PipeWire/PulseAudio, so direct hardware access is fine.
    # Analog mic (mono) → PipeWire/PulseAudio holds the device exclusively on
    # modern Ubuntu; use 'default' which routes through it and avoids
    # "device busy" errors while still resampling to any rate we request.
    if channels == 2:
        return f'hw:{card},{device_idx}', 2
    return 'default', 1


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


def _run_inference(model, class_names: list, waveform: np.ndarray,
                   threshold: float, keywords: list[str]):
    """Returns (hits, top_label, top_score).  hits = [(label, score), ...]"""
    scores, _, _ = model(waveform)
    mean_scores  = scores.numpy().mean(axis=0)
    top_indices  = np.argsort(mean_scores)[::-1][:10]

    hits = [
        (class_names[i], float(mean_scores[i]))
        for i in top_indices
        if _is_bell_like(class_names[i], keywords) and float(mean_scores[i]) >= threshold
    ]
    top_label = class_names[top_indices[0]]
    top_score = float(mean_scores[top_indices[0]])
    return hits, top_label, top_score


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
