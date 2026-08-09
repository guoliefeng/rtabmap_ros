#!/usr/bin/env bash
# Reproducible headless Mode B bag test. Run under `setsid` for long replays.
set -euo pipefail

RTABMAP_WS="${RTABMAP_WS:-/home/glf/dataDisk/Study/rtabMap_ws}"
FASTLIO_WS="${FASTLIO_WS:-/home/glf/proj/FAST_LIO_ws}"
GNSS_WS="${GNSS_WS:-/home/glf/proj/fast_lio-sam_loop-gps_ws}"
BAG_DIR="${BAG_DIR:-/home/glf/dataDisk/hainan/yangpu/qc}"
BAG_PATTERN="${BAG_PATTERN:-2026-06-22-12-*}"
DURATION="${DURATION:-30}"
ROS_PORT="${ROS_PORT:-11337}"
OUTPUT_DIR="${OUTPUT_DIR:-}"
USE_GPS=false

usage() {
    cat <<'EOF'
Usage: run_fastlio_rtabmap_bag_test.sh --output DIR [--duration SEC] [--gps] [--port PORT]
                                       [--bag-dir DIR] [--bag-pattern GLOB]

This starts an isolated ROS master, the FAST-LIO frontend only, the Mode B
RTAB-Map backend and `rosbag play BAG_PATTERN --clock -u SEC`.
With --gps, gnss_poser and the covariance-aware RTAB global-pose adapter are
started and the GPS-specific backend configuration is selected.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output) OUTPUT_DIR="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --gps) USE_GPS=true; shift ;;
        --port) ROS_PORT="$2"; shift 2 ;;
        --bag-dir) BAG_DIR="$2"; shift 2 ;;
        --bag-pattern) BAG_PATTERN="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -z "${OUTPUT_DIR}" || ! "${DURATION}" =~ ^[0-9]+([.][0-9]+)?$ || ! "${ROS_PORT}" =~ ^[0-9]+$ ]]; then
    usage >&2
    exit 2
fi
if [[ -e "${OUTPUT_DIR}/rtabmap.db" ]]; then
    echo "Refusing to overwrite existing database: ${OUTPUT_DIR}/rtabmap.db" >&2
    exit 2
fi
GPS_ORIGIN_FILE="${OUTPUT_DIR}/gps_origin.json"
if [[ "${USE_GPS}" == true && -e "${GPS_ORIGIN_FILE}" ]]; then
    echo "Refusing to reuse an existing GPS origin: ${GPS_ORIGIN_FILE}" >&2
    exit 2
fi

mapfile -t BAGS < <(find "${BAG_DIR}" -maxdepth 1 -type f -name "${BAG_PATTERN}" -print | sort)
if (( ${#BAGS[@]} == 0 )); then
    echo "No bags matched ${BAG_DIR}/${BAG_PATTERN}" >&2
    exit 2
fi

mkdir -p "${OUTPUT_DIR}"
RUN_DIR="$(dirname "${OUTPUT_DIR}")"
STATE_FILE="${RUN_DIR}/mode_b_run_status.txt"

source /opt/ros/noetic/setup.bash
source "${RTABMAP_WS}/install_isolated/setup.bash"
source "${FASTLIO_WS}/devel/setup.bash"
if [[ "${USE_GPS}" == true ]]; then
    source "${GNSS_WS}/devel/setup.bash"
fi
export ROS_PACKAGE_PATH="${FASTLIO_WS}/src:${GNSS_WS}/src:${RTABMAP_WS}/install_isolated/share:${HOME}/opt/ros-deps/opt/ros/noetic/share:${ROS_PACKAGE_PATH}"
export CMAKE_PREFIX_PATH="${RTABMAP_WS}/install_isolated:${FASTLIO_WS}/devel:${GNSS_WS}/devel:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${HOME}/opt/rtabmap-noetic/lib:${RTABMAP_WS}/install_isolated/lib:${FASTLIO_WS}/devel/lib:${GNSS_WS}/devel/lib:${HOME}/opt/ros-deps/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${RTABMAP_WS}/install_isolated/lib/python3/dist-packages:${PYTHONPATH:-}"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP=127.0.0.1

cleanup() {
    for pid in "${GPS_TRACE_PID:-}" "${MONITOR_PID:-}" "${SYNC_PID:-}" "${RTABMAP_PID:-}" "${GNSS_PID:-}" "${FASTLIO_PID:-}" "${MASTER_PID:-}"; do
        [[ -n "${pid}" ]] && kill "${pid}" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

printf 'started=%s\nduration_sec=%s\nbag_dir=%s\nbag_pattern=%s\noutput_dir=%s\ngps_enabled=%s\ngps_origin_file=%s\n' \
    "$(date -Is)" "${DURATION}" "${BAG_DIR}" "${BAG_PATTERN}" "${OUTPUT_DIR}" "${USE_GPS}" \
    "${GPS_ORIGIN_FILE}" > "${STATE_FILE}"
roscore -p "${ROS_PORT}" > "${RUN_DIR}/roscore.log" 2>&1 & MASTER_PID=$!
for _ in $(seq 1 100); do
    rosparam list >/dev/null 2>&1 && break
    sleep 0.1
done
rosparam list >/dev/null

rosparam set /use_sim_time true
rosparam load "${FASTLIO_WS}/src/FAST_LIO/config/hainan.yaml" /fast_lio_ns
rosparam set /fast_lio_ns/feature_extract_enable false
rosparam set /fast_lio_ns/point_filter_num 4
rosparam set /fast_lio_ns/max_iteration 3
rosparam set /fast_lio_ns/filter_size_surf 0.5
rosparam set /fast_lio_ns/filter_size_map 0.5
rosparam set /fast_lio_ns/cube_side_length 1000.0
rosparam set /fast_lio_ns/runtime_pos_log_enable false
rosparam set /fast_lio_ns/pcd_save/pcd_save_en false
FASTLIO_EXECUTABLE="${FASTLIO_WS}/devel/lib/fast_lio/fastlio_mapping"
if [[ ! -x "${FASTLIO_EXECUTABLE}" ]]; then
    echo "FAST-LIO executable is missing: ${FASTLIO_EXECUTABLE}" >&2
    exit 3
fi
"${FASTLIO_EXECUTABLE}" __name:=fastlio_mapping __ns:=/fast_lio_ns \
    > "${RUN_DIR}/fastlio.log" 2>&1 & FASTLIO_PID=$!
sleep 0.5
if ! kill -0 "${FASTLIO_PID}" 2>/dev/null; then
    echo "FAST-LIO exited during startup; see ${RUN_DIR}/fastlio.log" >&2
    exit 3
fi

RTABMAP_CONFIG="${RTABMAP_WS}/install_isolated/share/rtabmap_bringup/config/fastlio_rtabmap_mapping.yaml"
RTABMAP_ODOM_TOPIC="/fast_lio_ns/loc_result"
if [[ "${USE_GPS}" == true ]]; then
    RTABMAP_CONFIG="${RTABMAP_WS}/install_isolated/share/rtabmap_bringup/config/fastlio_rtabmap_gps_mapping.yaml"
    RTABMAP_ODOM_TOPIC="/fast_lio_ns/loc_result_rtabmap"
    roslaunch gnss_poser gnss_poser.launch > "${RUN_DIR}/gnss_poser.log" 2>&1 & GNSS_PID=$!
fi

roslaunch rtabmap_bringup fastlio_rtabmap_mapping.launch \
    use_sim_time:=true output_dir:="${OUTPUT_DIR}" delete_db_on_start:=false rviz:=false \
    config_path:="${RTABMAP_CONFIG}" use_global_pose:="${USE_GPS}" \
    global_pose_origin_file:="${GPS_ORIGIN_FILE}" \
    use_odom_covariance_adapter:="${USE_GPS}" odom_topic:="${RTABMAP_ODOM_TOPIC}" \
    > "${RUN_DIR}/rtabmap.log" 2>&1 & RTABMAP_PID=$!
sleep 3

if [[ "${USE_GPS}" == true ]]; then
    timeout "$(awk "BEGIN { print ${DURATION} + 15 }")" \
        rostopic echo -p /rtabmap/global_pose > "${RUN_DIR}/global_pose.csv" 2> "${RUN_DIR}/global_pose_trace.log" &
    GPS_TRACE_PID=$!
fi

rosrun rtabmap_bringup check_fastlio_sync.py --duration "$(awk "BEGIN { print ${DURATION} + 7 }")" \
    --map-frame rtabmap_map --output-json "${RUN_DIR}/sync.json" \
    --trajectory-csv "${RUN_DIR}/fastlio_odom.csv" > "${RUN_DIR}/sync.log" 2>&1 & SYNC_PID=$!

(
    sleep 30
    {
        printf 'checkpoint=30s\n'
        date -Is
        rosnode list
        ls -lh "${OUTPUT_DIR}/rtabmap.db" 2>&1 || true
        timeout 8 rostopic hz /fast_lio_ns/loc_result || true
        timeout 8 rostopic hz /rtabmap/info || true
    } > "${RUN_DIR}/runtime_30.log" 2>&1
    if (( $(awk "BEGIN { print (${DURATION} > 270) ? 1 : 0 }") )); then
        sleep 222
        {
            printf 'checkpoint=260s\n'
            date -Is
            rosnode list
            ls -lh "${OUTPUT_DIR}/rtabmap.db" 2>&1 || true
            timeout 8 rostopic hz /fast_lio_ns/cloud_registered_body || true
            timeout 8 rostopic hz /rtabmap/info || true
        } > "${RUN_DIR}/runtime_260.log" 2>&1
    fi
) & MONITOR_PID=$!

rosbag play "${BAGS[@]}" --clock -r 1 -u "${DURATION}" > "${RUN_DIR}/rosbag.log" 2>&1
if [[ "${USE_GPS}" == true ]]; then
    kill "${GPS_TRACE_PID}" 2>/dev/null || true
    wait "${GPS_TRACE_PID}" 2>/dev/null || true
fi
wait "${SYNC_PID}" || SYNC_STATUS=$?
SYNC_STATUS="${SYNC_STATUS:-0}"
wait "${MONITOR_PID}" || true
kill -INT "${RTABMAP_PID}" 2>/dev/null || true
wait "${RTABMAP_PID}" || true

ANALYSIS_DIR="${RUN_DIR}/analysis"
python3 "${RTABMAP_WS}/src/rtabmap_ros/rtabmap_bringup/scripts/analyze_rtabmap_db.py" \
    "${OUTPUT_DIR}/rtabmap.db" --output-dir "${ANALYSIS_DIR}" \
    --fastlio-trajectory "${RUN_DIR}/fastlio_odom.csv" > "${RUN_DIR}/analysis.log" 2>&1

printf 'finished=%s\nsync_status=%s\ndatabase=%s\nanalysis=%s\n' \
    "$(date -Is)" "${SYNC_STATUS}" "${OUTPUT_DIR}/rtabmap.db" "${ANALYSIS_DIR}" >> "${STATE_FILE}"
trap - EXIT INT TERM
cleanup
exit "${SYNC_STATUS}"
