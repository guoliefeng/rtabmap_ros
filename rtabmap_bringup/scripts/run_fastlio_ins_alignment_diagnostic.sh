#!/usr/bin/env bash
# Run FAST-LIO plus raw INS collection, then estimate fixed SE(2)+Z alignment.
set -euo pipefail

RTABMAP_WS="${RTABMAP_WS:-/home/glf/dataDisk/Study/rtabMap_ws}"
FASTLIO_WS="${FASTLIO_WS:-/home/glf/proj/FAST_LIO_ws}"
BAG_DIR="${BAG_DIR:-/home/glf/dataDisk/hainan/yangpu/qc}"
BAG_PATTERN="${BAG_PATTERN:-2026-06-22-12-*}"
DURATION="${DURATION:-120}"
ROS_PORT="${ROS_PORT:-11373}"
OUTPUT_DIR="${OUTPUT_DIR:-}"

usage() {
    cat <<'EOF'
Usage: run_fastlio_ins_alignment_diagnostic.sh --output DIR
       [--duration SEC] [--port PORT] [--bag-dir DIR] [--bag-pattern GLOB]

Starts an isolated ROS master and FAST-LIO, records /fast_lio_ns/loc_result and
/localization/ins independently, then searches -0.5..+0.5 s and estimates a
fixed-scale SE(2)+Z alignment in a translation-localized INS ENU frame.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output) OUTPUT_DIR="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --port) ROS_PORT="$2"; shift 2 ;;
        --bag-dir) BAG_DIR="$2"; shift 2 ;;
        --bag-pattern) BAG_PATTERN="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

NUMBER_PATTERN='^[0-9]+([.][0-9]+)?$'
if [[ -z "${OUTPUT_DIR}" || ! "${DURATION}" =~ ${NUMBER_PATTERN} ||
      ! "${ROS_PORT}" =~ ^[0-9]+$ ]]; then
    usage >&2
    exit 2
fi
if [[ -e "${OUTPUT_DIR}/fastlio.csv" || -e "${OUTPUT_DIR}/ins.csv" ||
      -e "${OUTPUT_DIR}/alignment/ins_fastlio_alignment.yaml" ]]; then
    echo "Refusing to overwrite an existing alignment diagnostic: ${OUTPUT_DIR}" >&2
    exit 2
fi
mapfile -t BAGS < <(find "${BAG_DIR}" -maxdepth 1 -type f -name "${BAG_PATTERN}" -print | sort)
if (( ${#BAGS[@]} == 0 )); then
    echo "No bags matched ${BAG_DIR}/${BAG_PATTERN}" >&2
    exit 2
fi

mkdir -p "${OUTPUT_DIR}" "${OUTPUT_DIR}/alignment"
STATE_FILE="${OUTPUT_DIR}/diagnostic_status.txt"
source /opt/ros/noetic/setup.bash
source "${RTABMAP_WS}/install_isolated/setup.bash"
source "${FASTLIO_WS}/devel/setup.bash"
export ROS_PACKAGE_PATH="${FASTLIO_WS}/src:${RTABMAP_WS}/install_isolated/share:${HOME}/opt/ros-deps/opt/ros/noetic/share:${ROS_PACKAGE_PATH}"
export CMAKE_PREFIX_PATH="${RTABMAP_WS}/install_isolated:${FASTLIO_WS}/devel:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${HOME}/opt/rtabmap-noetic/lib:${RTABMAP_WS}/install_isolated/lib:${FASTLIO_WS}/devel/lib:${HOME}/opt/ros-deps/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${RTABMAP_WS}/install_isolated/lib/python3/dist-packages:${PYTHONPATH:-}"
export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
export ROS_IP=127.0.0.1
export MPLCONFIGDIR="/tmp/rtabmap_bringup_matplotlib"

cleanup() {
    for pid in "${COLLECTOR_PID:-}" "${FASTLIO_PID:-}" "${MASTER_PID:-}"; do
        [[ -n "${pid}" ]] && kill "${pid}" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

printf 'started=%s\nduration_sec=%s\nbag_dir=%s\nbag_pattern=%s\noutput_dir=%s\n' \
    "$(date -Is)" "${DURATION}" "${BAG_DIR}" "${BAG_PATTERN}" "${OUTPUT_DIR}" \
    > "${STATE_FILE}"

roscore -p "${ROS_PORT}" > "${OUTPUT_DIR}/roscore.log" 2>&1 & MASTER_PID=$!
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
"${FASTLIO_EXECUTABLE}" __name:=fastlio_mapping __ns:=/fast_lio_ns \
    > "${OUTPUT_DIR}/fastlio.log" 2>&1 & FASTLIO_PID=$!
rosrun rtabmap_bringup collect_fastlio_ins_trajectory.py \
    _output_dir:="${OUTPUT_DIR}" > "${OUTPUT_DIR}/collector.log" 2>&1 & COLLECTOR_PID=$!
sleep 1
if ! kill -0 "${FASTLIO_PID}" 2>/dev/null || ! kill -0 "${COLLECTOR_PID}" 2>/dev/null; then
    echo "FAST-LIO or trajectory collector exited during startup" >&2
    exit 3
fi

rosbag play "${BAGS[@]}" --clock -r 1 -u "${DURATION}" \
    > "${OUTPUT_DIR}/rosbag.log" 2>&1
rosnode kill /collect_fastlio_ins_trajectory >/dev/null 2>&1 || true
for _ in $(seq 1 50); do
    ! kill -0 "${COLLECTOR_PID}" 2>/dev/null && break
    sleep 0.1
done
kill -TERM "${COLLECTOR_PID}" 2>/dev/null || true
wait "${COLLECTOR_PID}" || true
rosnode kill /fast_lio_ns/fastlio_mapping >/dev/null 2>&1 || true
for _ in $(seq 1 50); do
    ! kill -0 "${FASTLIO_PID}" 2>/dev/null && break
    sleep 0.1
done
kill -TERM "${FASTLIO_PID}" 2>/dev/null || true
wait "${FASTLIO_PID}" || true

python3 "${RTABMAP_WS}/install_isolated/lib/rtabmap_bringup/estimate_fastlio_ins_alignment.py" \
    --fastlio-csv "${OUTPUT_DIR}/fastlio.csv" \
    --ins-csv "${OUTPUT_DIR}/ins.csv" \
    --output-dir "${OUTPUT_DIR}/alignment" \
    > "${OUTPUT_DIR}/alignment_estimator.log" 2>&1

printf 'finished=%s\nalignment=%s\n' "$(date -Is)" \
    "${OUTPUT_DIR}/alignment/ins_fastlio_alignment.yaml" >> "${STATE_FILE}"
trap - EXIT INT TERM
cleanup
echo "FAST-LIO/INS alignment diagnostic complete: ${OUTPUT_DIR}"
