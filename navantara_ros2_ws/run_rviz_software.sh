#!/usr/bin/env bash
# ==============================================================================
# Helper Script to launch RViz2 with Software GL (Mesa LLVMpipe)
# Resolves Intel HD Graphics GLSL 8-bit texture sampler conflict for Costmaps.
# ==============================================================================
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

export LIBGL_ALWAYS_SOFTWARE=1
echo "[INFO] Launching RViz2 with Software Rendering (LIBGL_ALWAYS_SOFTWARE=1)..."
exec rviz2 -d "${WS_DIR}/install/navantara_navigation/share/navantara_navigation/config/navantara_view.rviz" "$@"
