#!/usr/bin/env bash
# Append a later Yangpu bag segment to a copy of an existing Mode B database.
set -euo pipefail

RTABMAP_WS="${RTABMAP_WS:-/home/glf/dataDisk/Study/rtabMap_ws}"
FASTLIO_WS="${FASTLIO_WS:-/home/glf/proj/FAST_LIO_ws}"
GNSS_WS="${GNSS_WS:-/home/glf/proj/fast_lio-sam_loop-gps_ws}"
BAG_DIR="${BAG_DIR:-/home/glf/dataDisk/hainan/yangpu/qc}"
BAG_PATTERN="${BAG_PATTERN:-2026-06-22-12-*}"
START_OFFSET="${START_OFFSET:-555}"
DURATION="${DURATION:-402.058}"
ROS_PORT="${ROS_PORT:-11338}"
WARMUP="${WARMUP:-0}"
CORE_LOG_LEVEL="${CORE_LOG_LEVEL:-4}"
SEED_DATABASE="${SEED_DATABASE:-}"
OUTPUT_DIR="${OUTPUT_DIR:-}"
USE_GPS=false

usage() {
    cat <<'EOF'
Usage: run_fastlio_rtabmap_multisession_test.sh \
  --seed-database FILE --output DIR [--start SEC] [--duration SEC] \
  [--warmup SEC] [--gps] [--core-log-level 0..4] [--port PORT] \
  [--bag-dir DIR] [--bag-pattern GLOB]

The seed database is copied, never modified in place. RTAB-Map loads the copy
paused, then resumes as soon as FAST-LIO publishes its first odometry message.
--warmup adds an optional delay after that message and should normally stay 0.
With --gps, gnss_poser and covariance-aware global pose priors are enabled.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --seed-database) SEED_DATABASE="$2"; shift 2 ;;
        --output) OUTPUT_DIR="$2"; shift 2 ;;
        --start) START_OFFSET="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --warmup) WARMUP="$2"; shift 2 ;;
        --gps) USE_GPS=true; shift ;;
        --core-log-level) CORE_LOG_LEVEL="$2"; shift 2 ;;
        --port) ROS_PORT="$2"; shift 2 ;;
        --bag-dir) BAG_DIR="$2"; shift 2 ;;
        --bag-pattern) BAG_PATTERN="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

NUMBER_PATTERN='^[0-9]+([.][0-9]+)?$'
if [[ -z "${SEED_DATABASE}" || -z "${OUTPUT_DIR}" ||
      ! "${START_OFFSET}" =~ ${NUMBER_PATTERN} || ! "${DURATION}" =~ ${NUMBER_PATTERN} ||
      ! "${WARMUP}" =~ ${NUMBER_PATTERN} || ! "${ROS_PORT}" =~ ^[0-9]+$ ||
      ! "${CORE_LOG_LEVEL}" =~ ^[0-4]$ ]]; then
    usage >&2
    exit 2
fi
if [[ ! -f "${SEED_DATABASE}" ]]; then
    echo "Seed database does not exist: ${SEED_DATABASE}" >&2
    exit 2
fi
if [[ -e "${OUTPUT_DIR}/rtabmap.db" ]]; then
    echo "Refusing to overwrite existing database: ${OUTPUT_DIR}/rtabmap.db" >&2
    exit 2
fi

mapfile -t BAGS < <(find "${BAG_DIR}" -maxdepth 1 -type f -name "${BAG_PATTERN}" -print | sort)
if (( ${#BAGS[@]} == 0 )); then
    echo "No bags matched ${BAG_DIR}/${BAG_PATTERN}" >&2
    exit 2
fi

mkdir -p "${OUTPUT_DIR}"
RUN_DIR="$(dirname "${OUTPUT_DIR}")"
STATE_FILE="${RUN_DIR}/multisession_run_status.txt"
cp --reflink=auto --preserve=timestamps "${SEED_DATABASE}" "${OUTPUT_DIR}/rtabmap.db"
GPS_ORIGIN_FILE="${OUTPUT_DIR}/gps_origin.json"
SEED_GPS_ORIGIN_FILE="$(dirname "${SEED_DATABASE}")/gps_origin.json"
if [[ "${USE_GPS}" == true ]]; then
    if [[ ! -f "${SEED_GPS_ORIGIN_FILE}" ]]; then
        echo "GPS seed origin does not exist: ${SEED_GPS_ORIGIN_FILE}" >&2
        exit 2
    fi
    cp --preserve=timestamps "${SEED_GPS_ORIGIN_FILE}" "${GPS_ORIGIN_FILE}"
fi

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
    for pid in "${GPS_TRACE_PID:-}" "${BAG_PID:-}" "${RTABMAP_WATCHDOG_PID:-}" "${MONITOR_PID:-}" "${SYNC_PID:-}" "${RTABMAP_PID:-}" "${GNSS_PID:-}" "${FASTLIO_PID:-}" "${MASTER_PID:-}"; do
        [[ -n "${pid}" ]] && kill "${pid}" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

{
    printf 'started=%s\n' "$(date -Is)"
    printf 'seed_database=%s\noutput_dir=%s\nbag_dir=%s\nbag_pattern=%s\n' \
        "${SEED_DATABASE}" "${OUTPUT_DIR}" "${BAG_DIR}" "${BAG_PATTERN}"
    printf 'start_offset_sec=%s\nduration_sec=%s\nwarmup_sec=%s\ncore_log_level=%s\n' \
        "${START_OFFSET}" "${DURATION}" "${WARMUP}" "${CORE_LOG_LEVEL}"
    printf 'gps_enabled=%s\n' "${USE_GPS}"
    printf 'seed_gps_origin_file=%s\ngps_origin_file=%s\n' \
        "${SEED_GPS_ORIGIN_FILE}" "${GPS_ORIGIN_FILE}"
    sqlite3 "${OUTPUT_DIR}/rtabmap.db" "select 'seed_nodes=' || count(*) from Node; select 'seed_maps=' || count(distinct map_id) from Node;"
} > "${STATE_FILE}"

roscore -p "${ROS_PORT}" > "${RUN_DIR}/roscore.log" 2>&1 & MASTER_PID=$!
for _ in $(seq 1 100); do
    rosparam list >/dev/null 2>&1 && break
    sleep 0.1
done
rosparam list >/dev/null

rosparam set /use_sim_time true
rosparam set /rtabmap/rtabmap/log_to_rosout_level "${CORE_LOG_LEVEL}"
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

MULTI_CONFIG="${RTABMAP_WS}/install_isolated/share/rtabmap_bringup/config/fastlio_rtabmap_multisession.yaml"
RTABMAP_ODOM_TOPIC="/fast_lio_ns/loc_result"
ODOM_PUBLISH_DELAY="0.0"
if [[ "${USE_GPS}" == true ]]; then
    MULTI_CONFIG="${RTABMAP_WS}/install_isolated/share/rtabmap_bringup/config/fastlio_rtabmap_gps_multisession.yaml"
    RTABMAP_ODOM_TOPIC="/fast_lio_ns/loc_result_rtabmap"
    roslaunch gnss_poser gnss_poser.launch > "${RUN_DIR}/gnss_poser.log" 2>&1 & GNSS_PID=$!
fi
roslaunch rtabmap_bringup fastlio_rtabmap_mapping.launch \
    use_sim_time:=true output_dir:="${OUTPUT_DIR}" delete_db_on_start:=false \
    config_path:="${MULTI_CONFIG}" start_paused:=true use_global_pose:="${USE_GPS}" \
    global_pose_origin_file:="${GPS_ORIGIN_FILE}" \
    odom_covariance_align_global_pose:="${USE_GPS}" \
    odom_covariance_publish_delay_sec:="${ODOM_PUBLISH_DELAY}" \
    odom_pose_from_message:="${USE_GPS}" \
    use_odom_covariance_adapter:="${USE_GPS}" odom_topic:="${RTABMAP_ODOM_TOPIC}" rviz:=false \
    > "${RUN_DIR}/rtabmap.log" 2>&1 & RTABMAP_PID=$!
for _ in $(seq 1 600); do
    rosservice list 2>/dev/null | grep -qx '/rtabmap/resume' && break
    sleep 0.2
done
if ! rosservice list 2>/dev/null | grep -qx '/rtabmap/resume'; then
    echo "RTAB-Map resume service did not become ready" >&2
    exit 3
fi

BACKEND_ODOM_FRAME_ARGS=()
if [[ "${USE_GPS}" == true ]]; then
    BACKEND_ODOM_FRAME_ARGS=(--backend-odom-frame gps_local_odom)
fi
rosrun rtabmap_bringup check_fastlio_sync.py "${BACKEND_ODOM_FRAME_ARGS[@]}" --duration "$(awk "BEGIN { print ${DURATION} + 8 }")" \
    --map-frame rtabmap_map --output-json "${RUN_DIR}/sync.json" \
    --trajectory-csv "${RUN_DIR}/fastlio_odom.csv" > "${RUN_DIR}/sync.log" 2>&1 & SYNC_PID=$!

if [[ "${USE_GPS}" == true ]]; then
    timeout "$(awk "BEGIN { print ${DURATION} + 15 }")" \
        rostopic echo -p /rtabmap/global_pose > "${RUN_DIR}/global_pose.csv" 2> "${RUN_DIR}/global_pose_trace.log" &
    GPS_TRACE_PID=$!
fi

rosbag play "${BAGS[@]}" --clock -r 1 -s "${START_OFFSET}" -u "${DURATION}" \
    > "${RUN_DIR}/rosbag.log" 2>&1 & BAG_PID=$!
ODOM_READY=false
for _ in $(seq 1 30); do
    if timeout 2 rostopic echo -n 1 /fast_lio_ns/loc_result/pose/pose \
        > "${RUN_DIR}/pre_resume_odom.txt" 2>&1; then
        ODOM_VALUES="$(awk '
            /^position:/ {in_position=1; next}
            /^orientation:/ {in_position=0}
            in_position && $1 == "x:" {x=$2}
            in_position && $1 == "y:" {y=$2}
            in_position && $1 == "z:" {z=$2}
            END {if (x != "" && y != "" && z != "") print x, y, z}
        ' "${RUN_DIR}/pre_resume_odom.txt")"
        if [[ -n "${ODOM_VALUES}" ]]; then
            read -r ODOM_X ODOM_Y ODOM_Z <<< "${ODOM_VALUES}"
            if awk -v x="${ODOM_X}" -v y="${ODOM_Y}" -v z="${ODOM_Z}" \
                'BEGIN {exit !((x*x + y*y + z*z) > 0.0025)}'; then
                ODOM_READY=true
                break
            fi
        fi
    fi
done
if [[ "${ODOM_READY}" != true ]]; then
    echo "FAST-LIO did not publish a non-zero odometry pose within 30 attempts" >&2
    exit 3
fi
sleep "${WARMUP}"
rosservice call /rtabmap/resume > "${RUN_DIR}/resume_service.log" 2>&1
printf 'resumed=%s\n' "$(date -Is)" >> "${STATE_FILE}"

# roslaunch stays alive when its rtabmap child crashes, so without this guard a
# bag would keep playing and leave a misleading partial database. Require three
# failed XML-RPC pings before treating the backend as failed.
RTABMAP_FAILURE_FILE="${RUN_DIR}/rtabmap_backend_failure.txt"
(
    failures=0
    while kill -0 "${BAG_PID}" 2>/dev/null; do
        if ! kill -0 "${RTABMAP_PID}" 2>/dev/null; then
            failures=3
        elif timeout 3 rosnode ping /rtabmap -c 1 >/dev/null 2>&1; then
            failures=0
        else
            failures=$((failures + 1))
            if (( failures >= 3 )); then
                {
                    printf 'detected=%s\n' "$(date -Is)"
                    printf 'reason=rtabmap_node_unreachable\n'
                } > "${RTABMAP_FAILURE_FILE}"
                kill -TERM "${BAG_PID}" 2>/dev/null || true
                exit 0
            fi
        fi
        sleep 5
    done
) & RTABMAP_WATCHDOG_PID=$!

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
    if (( $(awk "BEGIN { print (${DURATION} > 230) ? 1 : 0 }") )); then
        sleep 150
        {
            printf 'checkpoint=180s\n'
            date -Is
            rosnode list
            ls -lh "${OUTPUT_DIR}/rtabmap.db" 2>&1 || true
            timeout 8 rostopic hz /fast_lio_ns/cloud_registered_body || true
            timeout 8 rostopic hz /rtabmap/info || true
        } > "${RUN_DIR}/runtime_180.log" 2>&1
    fi
) & MONITOR_PID=$!

wait "${BAG_PID}"
wait "${RTABMAP_WATCHDOG_PID}" || true
if [[ "${USE_GPS}" == true ]]; then
    kill "${GPS_TRACE_PID}" 2>/dev/null || true
    wait "${GPS_TRACE_PID}" 2>/dev/null || true
fi
wait "${SYNC_PID}" || SYNC_STATUS=$?
SYNC_STATUS="${SYNC_STATUS:-0}"
if [[ -f "${RTABMAP_FAILURE_FILE}" ]]; then
    SYNC_STATUS=4
fi
wait "${MONITOR_PID}" || true
kill -INT "${RTABMAP_PID}" 2>/dev/null || true
wait "${RTABMAP_PID}" || true

ANALYSIS_DIR="${RUN_DIR}/analysis"
python3 "${RTABMAP_WS}/src/rtabmap_ros/rtabmap_bringup/scripts/analyze_rtabmap_db.py" \
    "${OUTPUT_DIR}/rtabmap.db" --output-dir "${ANALYSIS_DIR}" \
    --fastlio-trajectory "${RUN_DIR}/fastlio_odom.csv" > "${RUN_DIR}/analysis.log" 2>&1

{
    printf 'finished=%s\nsync_status=%s\ndatabase=%s\nanalysis=%s\n' \
        "$(date -Is)" "${SYNC_STATUS}" "${OUTPUT_DIR}/rtabmap.db" "${ANALYSIS_DIR}"
    sqlite3 "${OUTPUT_DIR}/rtabmap.db" "select 'final_nodes=' || count(*) from Node; select 'final_maps=' || count(distinct map_id) from Node;"
} >> "${STATE_FILE}"
trap - EXIT INT TERM
cleanup
exit "${SYNC_STATUS}"
