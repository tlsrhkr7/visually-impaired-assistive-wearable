#!/usr/bin/env bash
# Run once after cloning to:
#  1. Initialize submodule (ORB_SLAM3 fork with Jetson fixes)
#  2. Substitute hardcoded map paths in YAML configs to the current clone location
#  3. Create maps/ directory
#  4. Build the ORB_SLAM3 library (Pangolin/g2o/DBoW2/Sophus)
#
# Usage:
#   git clone --recursive https://github.com/tlsrhkr7/visually-impaired-assistive-wearable.git
#   cd visually-impaired-assistive-wearable
#   ./setup.sh
#
# After this, run from your ros2_ws root:
#   colcon build --packages-select orb_slam3_ros2
#   source install/setup.bash

set -e

cd "$(dirname "$0")"
REPO_ROOT="$(pwd)"

echo "=========================================="
echo "Repo root: $REPO_ROOT"
echo "=========================================="

# 1. Initialize/update submodule (in case clone wasn't --recursive)
echo ""
echo "[1/4] Initializing submodule..."
git submodule update --init --recursive

# 2. Substitute hardcoded paths in YAML configs
echo ""
echo "[2/4] Substituting map paths in YAML configs..."
NEW_MAP_PATH="${REPO_ROOT}/maps/d435i_map"
for yaml in config/RGBD-Inertial/RealSense_D435i.yaml \
            config/RGBD-Inertial/RealSense_D435i_localization.yaml; do
    if [[ -f "$yaml" ]]; then
        sed -i.bak -E "s|/home/[^\"]*/maps/d435i_map|${NEW_MAP_PATH}|g" "$yaml"
        rm -f "${yaml}.bak"
        echo "  ✓ $yaml"
    fi
done

# 3. Create maps directory
echo ""
echo "[3/4] Creating maps/ directory..."
mkdir -p "${REPO_ROOT}/maps"
echo "  ✓ ${REPO_ROOT}/maps"

# 4. Build ORB_SLAM3 library (its dependencies + main lib)
echo ""
echo "[4/4] Building ORB_SLAM3 library (this takes 10–20 minutes)..."
cd "${REPO_ROOT}/ORB_SLAM3"
chmod +x build.sh
./build.sh

cd "$REPO_ROOT"
echo ""
echo "=========================================="
echo "Setup complete."
echo ""
echo "Next steps from your ros2_ws root:"
echo "  colcon build --packages-select orb_slam3_ros2"
echo "  source install/setup.bash"
echo ""
echo "Then run mapping:"
echo "  ros2 launch orb_slam3_ros2 mapping.launch.py"
echo "=========================================="
