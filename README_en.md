<a name="readme-top"></a>

[EN](README_en.md) | [JA](README.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# YAMNet ROS

<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#overview">Overview</a>
    </li>
    <li>
      <a href="#setup">Setup</a>
      <ul>
        <li><a href="#environment">Environment</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#parameters">Parameters</a></li>
    <li><a href="#topics--actions">Topics & Actions</a></li>
    <li><a href="#references">References</a></li>
  </ol>
</details>


## Overview
`yamnet_ros` is a ROS 2 wrapper for Google's **YAMNet** audio classification model, bringing real-time sound event detection to robots via microphone.

YAMNet recognises **521 sound classes**. The default configuration targets doorbell and bell sounds, but any acoustic event can be detected by simply changing `target_labels` in the config — no code changes required.

Audio capture uses ALSA (`arecord`) directly — no extra audio libraries needed.

**Main Features:**
- Real-time detection of any of YAMNet's 521 sound classes (default: bell/doorbell)
- Publishes a `SoundDetection` message on every detected event
- Action server (`ListenForSound`) for SMACH state machines or behavior trees — blocks until detected or timeout
- Fully configurable via YAML — change detection targets, threshold, device, and more
- LifecycleNode — zero CPU when inactive, clean activate/deactivate from task code

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Setup

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Environment

| System | Version |
| ------ | ------- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS    | Jazzy Jalisco |
| Python | 3.12 |

### Installation
1. Move to your ROS 2 `src` directory.
   ```sh
   cd ~/colcon_ws/src/
   ```
2. Clone this repository.
   ```sh
   git clone -b jazzy-devel https://github.com/TeamSOBITS/yamnet_ros.git
   ```
3. Navigate into the repository.
   ```sh
   cd yamnet_ros
   ```
4. Run the install script (downloads weights, YAMNet source, and Python dependencies).
   ```sh
   bash install.sh
   ```
5. Build the package.
   ```sh
   cd ~/colcon_ws/
   colcon build --packages-select sobits_interfaces yamnet_ros
   source install/setup.bash
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Usage

1. Launch the node.
   ```sh
   ros2 launch yamnet_ros yamnet_ros.launch.py
   ```

2. In a separate terminal, configure and activate the LifecycleNode.
   ```sh
   ros2 lifecycle set /yamnet_ros configure
   ros2 lifecycle set /yamnet_ros activate
   ```

3. Listen for detection events.
   ```sh
   ros2 topic echo /yamnet_ros/sound_detection
   ```

4. Use the action server to wait for a bell in a script or state machine.
   ```sh
   # Wait up to 30 seconds for a bell (returns immediately on detection)
   ros2 action send_goal /yamnet_ros/listen_for_sound \
     sobits_interfaces/action/ListenForSound \
     "{timeout_sec: 30.0, threshold: 0.15}"
   ```

5. Override parameters at launch time without editing any file.
   ```sh
   ros2 launch yamnet_ros yamnet_ros.launch.py \
     detection_threshold:=0.25 \
     hop_secs:=0.25 \
     audio_device:=hw:1,0
   ```

6. List available microphone devices.
   ```sh
   arecord -l
   ```

> **Note:** This node accesses ALSA directly, so the desktop mic indicator will **not** appear. This is expected — bypassing PulseAudio is intentional for lower latency.

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Parameters

All parameters are set in [`config/yamnet_ros.yaml`](config/yamnet_ros.yaml) and can be overridden from the launch file.

| Parameter | Description | Default |
| --------- | ----------- | ------- |
| `weights_path` | Path to `yamnet.h5`. Empty = auto-resolved from package share. | `''` |
| `sample_rate` | Audio sample rate in Hz. YAMNet requires 16000. | `16000` |
| `detection_threshold` | Minimum YAMNet score [0–1] to publish a detection. Lower = more sensitive. | `0.15` |
| `hop_secs` | How often (seconds) to run inference. Lower = faster response, more CPU. | `0.5` |
| `window_secs` | Sliding audio window length fed to YAMNet each call. Minimum ~0.96 s. | `1.0` |
| `audio_device` | ALSA capture device name. Use `arecord -l` to list devices. | `hw:1,7` |
| `audio_channels` | Capture channels. DMIC (`hw:1,7`) needs 2; analog mics usually need 1. | `2` |
| `cooldown_secs` | Minimum gap (seconds) between consecutive published detections. | `2.0` |
| `target_labels` | List of keywords matched against YAMNet class names to count as a bell. | see yaml |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## Topics & Actions

### Publications

| Topic | Type | Description |
| ----- | ---- | ----------- |
| `/yamnet_ros/sound_detection` | `sobits_interfaces/SoundDetection` | Published each time a bell-like sound is detected. Contains label, score, and top overall class. |

### Action Servers

| Action | Type | Description |
| ------ | ---- | ----------- |
| `/yamnet_ros/listen_for_sound` | `sobits_interfaces/action/ListenForSound` | Blocks until a bell is detected or `timeout_sec` expires. Returns `detected`, `label`, `score`, `elapsed_time`. Sends live feedback every hop. |

### SoundDetection.msg fields

| Field | Type | Description |
| ----- | ---- | ----------- |
| `header` | `std_msgs/Header` | Timestamp of the detection |
| `label` | `string` | Best matching bell-like class (e.g. `"Doorbell"`, `"Bell"`) |
| `score` | `float32` | YAMNet score for that class |
| `top_label` | `string` | Top overall YAMNet class (for diagnostics) |
| `top_score` | `float32` | Top overall score (for diagnostics) |

<p align="right">(<a href="#readme-top">back to top</a>)</p>


## References
- [YAMNet — TensorFlow Models (AudioSet)](https://github.com/tensorflow/models/tree/master/research/audioset/yamnet)
- [AudioSet Ontology](https://research.google.com/audioset/ontology/index.html)
- [Google Research — YAMNet](https://tfhub.dev/google/yamnet/1)

[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/yamnet_ros.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/yamnet_ros/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/yamnet_ros.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/yamnet_ros/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/yamnet_ros.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/yamnet_ros/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/yamnet_ros.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/yamnet_ros/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/yamnet_ros.svg?style=for-the-badge
[license-url]: LICENSE
