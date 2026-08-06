#!/usr/bin/env bash
set -euo pipefail

if [[ -f /opt/ros/noetic/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../../" && pwd)"
RTABMAP_INSTALL_PREFIX="${RTABMAP_INSTALL_PREFIX:-${HOME}/opt/rtabmap-noetic}"
if [[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE_ROOT}/devel/setup.bash"
elif [[ -f "${WORKSPACE_ROOT}/devel_isolated/rtabmap_bringup/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE_ROOT}/devel_isolated/rtabmap_bringup/setup.bash"
fi
export LD_LIBRARY_PATH="${RTABMAP_INSTALL_PREFIX}/lib:/opt/ros/noetic/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

check_topic() {
  local topic="$1"
  if rostopic list 2>/dev/null | grep -Fxq "$topic"; then
    echo "PASS: topic advertised: ${topic}"
  else
    echo "FAIL: topic not advertised: ${topic}"
  fi
}

topic_hz() {
  local topic="$1"
  echo "INFO: rostopic hz ${topic}"
  timeout 8s rostopic hz -w 3 "$topic" 2>&1 | sed -n '1,12p' || echo "WARN: no complete hz window for ${topic}"
}

echo "## Runtime checks $(date --iso-8601=seconds)"
for topic in \
  /lidar_preprocessor/meta_cloud \
  /ins_driver/imu \
  /rtabmap/odom \
  /rtabmap/odom_info \
  /rtabmap/info; do
  check_topic "$topic"
done

for topic in \
  /lidar_preprocessor/meta_cloud \
  /ins_driver/imu \
  /rtabmap/odom \
  /rtabmap/odom_info \
  /rtabmap/info; do
  topic_hz "$topic"
done

if timeout 8s rostopic echo -n 1 /rtabmap/odom >/tmp/rtabmap_runtime_odom.$$ 2>/dev/null; then
  if grep -Eq 'orientation:|child_frame_id:' /tmp/rtabmap_runtime_odom.$$; then
    echo "PASS: /rtabmap/odom received"
  else
    echo "WARN: /rtabmap/odom received but quaternion parsing needs manual inspection"
  fi
else
  echo "FAIL: no /rtabmap/odom message received"
fi
rm -f /tmp/rtabmap_runtime_odom.$$

lost_value="$(timeout 8s rostopic echo -n 1 /rtabmap/odom_info/lost 2>/dev/null || true)"
if [[ "$lost_value" == "False" ]]; then
  echo "PASS: /rtabmap/odom_info/lost=False"
elif [[ "$lost_value" == "True" ]]; then
  echo "WARN: latest /rtabmap/odom_info/lost=True"
else
  echo "WARN: no /rtabmap/odom_info/lost value was read"
fi

tf_check() {
  local parent="$1"
  local child="$2"
  local output
  output="$(timeout 5s rosrun tf tf_echo "$parent" "$child" 2>&1 || true)"
  if grep -q "At time\|Translation:" <<<"$output"; then
    echo "PASS: TF ${parent} <- ${child}"
    sed -n '1,12p' <<<"$output"
  else
    echo "FAIL: missing TF ${parent} <- ${child}"
    sed -n '1,4p' <<<"$output"
  fi
}
tf_check odom_rtabmap base_link
tf_check map odom_rtabmap

if rosnode list 2>/dev/null | grep -Eq '/rtabmap/(icp_odometry|rtabmap)'; then
  echo "PASS: RTAB-Map nodes are running"
else
  echo "FAIL: expected RTAB-Map nodes are not both visible"
fi

db_path="$(rosparam get /rtabmap/database_path 2>/dev/null | tr -d "'" || true)"
db_path="${db_path:-${RTABMAP_DB:-}}"
if [[ -n "$db_path" && -f "$db_path" ]]; then
  size_before=$(stat -c %s "$db_path")
  sleep 2
  size_after=$(stat -c %s "$db_path")
  echo "INFO: database ${db_path} size ${size_before} -> ${size_after} bytes"
  if (( size_after > size_before )); then
    echo "PASS: database is growing"
  else
    echo "WARN: database did not grow during the 2-second sample"
  fi
  if command -v sqlite3 >/dev/null; then
    echo "INFO: database node count: $(sqlite3 "$db_path" 'select count(*) from Node;' 2>/dev/null || echo unavailable)"
  fi
else
  echo "FAIL: RTAB-Map database file was not found"
fi

echo "INFO: inspect roslaunch output for repeated: missing transform, did not receive data, not enough correspondences, odometry lost, invalid consecutive stamps, or dropping imu data."
