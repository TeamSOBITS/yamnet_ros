#!/bin/bash
set -e

ROS_DISTRO=${ROS_DISTRO:-jazzy}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YAMNET_SRC="${SCRIPT_DIR}/yamnet_ros/yamnet_src"
WEIGHTS_DIR="${SCRIPT_DIR}/weights"
YAMNET_BASE="https://raw.githubusercontent.com/tensorflow/models/master/research/audioset/yamnet"

echo "╔══╣ Install: YAMNet ROS (STARTING) ╠══╗"

# ── System dependencies ─────────────────────────────────────────────
sudo apt-get update -qq
sudo apt-get install -y alsa-utils   # provides arecord for microphone capture

# ── Python dependencies ─────────────────────────────────────────────
python3 -m pip install --break-system-packages "tensorflow-cpu==2.16.2"
python3 -m pip install --break-system-packages "tf_keras==2.16.0"
python3 -m pip install --break-system-packages "resampy>=0.4.0" "soundfile>=0.12.0" "numpy>=1.21,<2.0"

# ── YAMNet source files ─────────────────────────────────────────────
mkdir -p "${YAMNET_SRC}"
for f in yamnet.py params.py features.py yamnet_class_map.csv; do
    if [ ! -f "${YAMNET_SRC}/${f}" ]; then
        echo "Downloading ${f}..."
        curl -sL "${YAMNET_BASE}/${f}" -o "${YAMNET_SRC}/${f}"
    fi
done

# ── YAMNet weights (~15 MB) ─────────────────────────────────────────
mkdir -p "${WEIGHTS_DIR}"
if [ ! -f "${WEIGHTS_DIR}/yamnet.h5" ]; then
    echo "Downloading yamnet.h5..."
    curl -L "https://storage.googleapis.com/audioset/yamnet.h5" -o "${WEIGHTS_DIR}/yamnet.h5"
fi

echo "╚══╣ Install: YAMNet ROS (FINISHED) ╠══╝"
