#!/usr/bin/env bash
set -euo pipefail

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../../" && pwd)"
RTABMAP_INSTALL_PREFIX="${RTABMAP_INSTALL_PREFIX:-${HOME}/opt/rtabmap-noetic}"
ROS_DEPS_PREFIX="${ROS_DEPS_PREFIX:-${HOME}/opt/ros-deps/opt/ros/noetic}"
if [[ -f "${WORKSPACE_ROOT}/install_isolated/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE_ROOT}/install_isolated/setup.bash"
elif [[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE_ROOT}/devel/setup.bash"
elif [[ -f "${WORKSPACE_ROOT}/devel_isolated/rtabmap_bringup/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE_ROOT}/devel_isolated/rtabmap_bringup/setup.bash"
fi
if [[ -d "${ROS_DEPS_PREFIX}/share" ]]; then
  export ROS_PACKAGE_PATH="${ROS_DEPS_PREFIX}/share${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
  export CMAKE_PREFIX_PATH="${RTABMAP_INSTALL_PREFIX}:${ROS_DEPS_PREFIX}:/opt/ros/noetic${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
else
  export CMAKE_PREFIX_PATH="${RTABMAP_INSTALL_PREFIX}:/opt/ros/noetic${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
fi
export LD_LIBRARY_PATH="${RTABMAP_INSTALL_PREFIX}/lib:${WORKSPACE_ROOT}/install_isolated/lib:/opt/ros/noetic/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

command -v roscore >/dev/null || { echo "FAIL: ROS Noetic is not sourced" >&2; exit 1; }
command -v roslaunch >/dev/null || { echo "FAIL: roslaunch is not available" >&2; exit 1; }
rospack find rtabmap_bringup >/dev/null || { echo "FAIL: source the built catkin workspace containing rtabmap_bringup" >&2; exit 1; }

BAG_DIR="${YANGPU_BAG_DIR:-${HOME}/dataDisk/hainan/yangpu/qc}"
OUTPUT_DIR="${RTABMAP_OUTPUT_DIR:-${BAG_DIR}/rtabmap_output}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/lidar_preprocessor/meta_cloud}"
IMU_TOPIC="${IMU_TOPIC:-/ins_driver/imu}"
DELETE_DB_ON_START="${DELETE_DB_ON_START:-false}"
DB_PATH="${OUTPUT_DIR}/rtabmap.db"

for arg in "$@"; do
  case "$arg" in
    output_dir:=*) OUTPUT_DIR="${arg#output_dir:=}"; DB_PATH="${OUTPUT_DIR}/rtabmap.db";;
    delete_db_on_start:=true) DELETE_DB_ON_START=true;;
    delete_db_on_start:=false) DELETE_DB_ON_START=false;;
    cloud_topic:=*) CLOUD_TOPIC="${arg#cloud_topic:=}";;
    imu_topic:=*) IMU_TOPIC="${arg#imu_topic:=}";;
  esac
done

if [[ ! -d "$BAG_DIR" ]]; then
  echo "FAIL: bag directory does not exist: ${BAG_DIR}" >&2
  exit 1
fi
if ! find "$BAG_DIR" -maxdepth 1 -type f -name '2026-06-22-12-*' -print -quit | grep -q .; then
  echo "FAIL: no Yangpu bag matched ${BAG_DIR}/2026-06-22-12-*" >&2
  exit 1
fi
mkdir -p "$OUTPUT_DIR"

if [[ -e "$DB_PATH" || -e "${DB_PATH}-journal" ]]; then
  if [[ "$DELETE_DB_ON_START" == true && "${ALLOW_DB_DELETE:-0}" == 1 ]]; then
    echo "WARN: existing database will be deleted by RTAB-Map because delete_db_on_start=true and ALLOW_DB_DELETE=1: ${DB_PATH}"
  else
    echo "FAIL: database already exists: ${DB_PATH}" >&2
    echo "      Choose a new RTABMAP_OUTPUT_DIR, or explicitly set ALLOW_DB_DELETE=1 with delete_db_on_start:=true." >&2
    exit 1
  fi
fi

echo "PASS: ROS Noetic and rtabmap_bringup are available"
echo "PASS: bag directory: ${BAG_DIR}"
echo "INFO: cloud topic: ${CLOUD_TOPIC}"
echo "INFO: IMU topic: ${IMU_TOPIC}"
echo "INFO: output database: ${DB_PATH}"
echo "INFO: after this launch starts, play the bag in another terminal:"
echo "  cd ${BAG_DIR}"
echo "  rosbag play 2026-06-22-12-* --clock -u 30"
echo "INFO: full run command:"
echo "  rosbag play 2026-06-22-12-* --clock -u 515"

exec roslaunch rtabmap_bringup lidar_imu_mapping.launch \
  use_sim_time:=true \
  delete_db_on_start="${DELETE_DB_ON_START}" \
  cloud_topic="${CLOUD_TOPIC}" \
  imu_topic="${IMU_TOPIC}" \
  output_dir="${OUTPUT_DIR}" \
  "$@"
