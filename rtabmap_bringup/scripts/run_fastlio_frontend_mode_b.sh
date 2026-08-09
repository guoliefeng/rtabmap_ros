#!/usr/bin/env bash
# Start only the FAST-LIO frontend required by Mode B. No GPS/localization nodes.
set -euo pipefail

RTABMAP_WS="${RTABMAP_WS:-/home/glf/dataDisk/Study/rtabMap_ws}"
FASTLIO_WS="${FASTLIO_WS:-/home/glf/proj/FAST_LIO_ws}"
FASTLIO_CONFIG="${FASTLIO_CONFIG:-${FASTLIO_WS}/src/FAST_LIO/config/hainan.yaml}"

source /opt/ros/noetic/setup.bash
source "${RTABMAP_WS}/install_isolated/setup.bash"
source "${FASTLIO_WS}/devel/setup.bash"

# FAST_LIO's setup.bash replaces overlays on this installation. Restore RTAB-Map
# package, executable and shared-library lookup paths explicitly.
export ROS_PACKAGE_PATH="${RTABMAP_WS}/install_isolated/share:${HOME}/opt/ros-deps/opt/ros/noetic/share:${ROS_PACKAGE_PATH}"
export CMAKE_PREFIX_PATH="${RTABMAP_WS}/install_isolated:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${HOME}/opt/rtabmap-noetic/lib:${RTABMAP_WS}/install_isolated/lib:${HOME}/opt/ros-deps/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${RTABMAP_WS}/install_isolated/lib/python3/dist-packages:${PYTHONPATH:-}"

if [[ ! -f "${FASTLIO_CONFIG}" ]]; then
    echo "FAST-LIO config not found: ${FASTLIO_CONFIG}" >&2
    exit 2
fi

rosparam set /use_sim_time true
rosparam load "${FASTLIO_CONFIG}" /fast_lio_ns

# These are the existing mapping_hainan.launch frontend settings. They are set at
# runtime so that this Mode B entrypoint neither edits nor starts that launch's
# GPS/localization/RViz auxiliary nodes.
rosparam set /fast_lio_ns/feature_extract_enable false
rosparam set /fast_lio_ns/point_filter_num 4
rosparam set /fast_lio_ns/max_iteration 3
rosparam set /fast_lio_ns/filter_size_surf 0.5
rosparam set /fast_lio_ns/filter_size_map 0.5
rosparam set /fast_lio_ns/cube_side_length 1000.0
rosparam set /fast_lio_ns/runtime_pos_log_enable false
rosparam set /fast_lio_ns/pcd_save/pcd_save_en false

exec rosrun fast_lio fastlio_mapping __name:=fastlio_mapping __ns:=/fast_lio_ns
