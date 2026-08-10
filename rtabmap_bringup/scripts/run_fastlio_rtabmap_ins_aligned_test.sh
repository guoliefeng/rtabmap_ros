#!/usr/bin/env bash
# Guarded Mode B run using a fixed calibrated FAST-LIO-to-INS-local transform.
set -euo pipefail

RTABMAP_WS="${RTABMAP_WS:-/home/glf/dataDisk/Study/rtabMap_ws}"
FASTLIO_WS="${FASTLIO_WS:-/home/glf/proj/FAST_LIO_ws}"
BAG_DIR="${BAG_DIR:-/home/glf/dataDisk/hainan/yangpu/qc}"
BAG_PATTERN="${BAG_PATTERN:-2026-06-22-12-*}"
DURATION="${DURATION:-120}"
ROS_PORT="${ROS_PORT:-11374}"
ALIGNMENT_FILE="${ALIGNMENT_FILE:-}"
RUN_DIR="${RUN_DIR:-}"
USE_INS_PRIOR=false

usage() {
    cat <<'EOF'
Usage: run_fastlio_rtabmap_ins_aligned_test.sh --alignment FILE --output DIR
       [--duration SEC] [--port PORT] [--bag-dir DIR] [--bag-pattern GLOB]
       [--ins-prior]

Runs FAST-LIO, fixed INS-frame odometry alignment and RTAB-Map without GNSS
poser, global pose priors or RTAB-Map IMU. The bag is played from its beginning.
With --ins-prior, /localization/ins is converted through the recorded
base_link<-ins extrinsic and added as a translation-only, position-only prior.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --alignment) ALIGNMENT_FILE="$2"; shift 2 ;;
        --output) RUN_DIR="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --port) ROS_PORT="$2"; shift 2 ;;
        --bag-dir) BAG_DIR="$2"; shift 2 ;;
        --bag-pattern) BAG_PATTERN="$2"; shift 2 ;;
        --ins-prior) USE_INS_PRIOR=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

NUMBER_PATTERN='^[0-9]+([.][0-9]+)?$'
if [[ -z "${ALIGNMENT_FILE}" || ! -f "${ALIGNMENT_FILE}" ||
      -z "${RUN_DIR}" || ! "${DURATION}" =~ ${NUMBER_PATTERN} ||
      ! "${ROS_PORT}" =~ ^[0-9]+$ ]]; then
    usage >&2
    exit 2
fi
if [[ -e "${RUN_DIR}/rtabmap.db" ]]; then
    echo "Refusing to overwrite existing database: ${RUN_DIR}/rtabmap.db" >&2
    exit 2
fi
mapfile -t BAGS < <(find "${BAG_DIR}" -maxdepth 1 -type f -name "${BAG_PATTERN}" -print | sort)
if (( ${#BAGS[@]} == 0 )); then
    echo "No bags matched ${BAG_DIR}/${BAG_PATTERN}" >&2
    exit 2
fi

ANALYSIS_DIR="${RUN_DIR}/analysis"
mkdir -p "${RUN_DIR}" "${ANALYSIS_DIR}"
STATE_FILE="${RUN_DIR}/ins_aligned_run_status.txt"
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
    for pid in "${GLOBAL_POSE_TRACE_PID:-}" "${BAG_PID:-}" \
               "${RTABMAP_WATCHDOG_PID:-}" "${SYNC_PID:-}" \
               "${COLLECTOR_PID:-}" "${RTABMAP_PID:-}" \
               "${FASTLIO_PID:-}" "${MASTER_PID:-}"; do
        [[ -n "${pid}" ]] && kill "${pid}" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

printf 'started=%s\nduration_sec=%s\nbag_dir=%s\nbag_pattern=%s\nrun_dir=%s\nalignment=%s\n' \
    "$(date -Is)" "${DURATION}" "${BAG_DIR}" "${BAG_PATTERN}" "${RUN_DIR}" \
    "${ALIGNMENT_FILE}" > "${STATE_FILE}"
printf 'ins_position_prior=%s\n' "${USE_INS_PRIOR}" >> "${STATE_FILE}"

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

RTABMAP_CONFIG="${RTABMAP_WS}/install_isolated/share/rtabmap_bringup/config/fastlio_rtabmap_mapping.yaml"
INS_ORIGIN_FILE="${RUN_DIR}/ins_prior_origin.json"
if [[ "${USE_INS_PRIOR}" == true ]]; then
    RTABMAP_CONFIG="${RTABMAP_WS}/install_isolated/share/rtabmap_bringup/config/fastlio_rtabmap_ins_prior_mapping.yaml"
    INS_ORIGIN_FILE="${ALIGNMENT_FILE}"
fi
printf 'rtabmap_config=%s\n' "${RTABMAP_CONFIG}" >> "${STATE_FILE}"
printf 'ins_origin_file=%s\n' "${INS_ORIGIN_FILE}" >> "${STATE_FILE}"

"${FASTLIO_WS}/devel/lib/fast_lio/fastlio_mapping" \
    __name:=fastlio_mapping __ns:=/fast_lio_ns \
    > "${RUN_DIR}/fastlio.log" 2>&1 & FASTLIO_PID=$!
roslaunch rtabmap_bringup fastlio_rtabmap_ins_aligned_mapping.launch \
    alignment_file:="${ALIGNMENT_FILE}" output_dir:="${RUN_DIR}" \
    delete_db_on_start:=false use_ins_position_prior:="${USE_INS_PRIOR}" \
    ins_origin_file:="${INS_ORIGIN_FILE}" \
    config_path:="${RTABMAP_CONFIG}" rviz:=false \
    > "${RUN_DIR}/rtabmap.log" 2>&1 & RTABMAP_PID=$!
PRIOR_TOPIC=""
if [[ "${USE_INS_PRIOR}" == true ]]; then
    PRIOR_TOPIC="/rtabmap/global_pose"
fi
rosrun rtabmap_bringup collect_fastlio_ins_trajectory.py \
    _output_dir:="${ANALYSIS_DIR}" \
    _fastlio_filename:=fastlio_raw.csv _ins_filename:=ins.csv \
    _aligned_topic:=/fast_lio_ns/loc_result_ins_aligned \
    _aligned_filename:=fastlio_aligned.csv \
    _prior_topic:="${PRIOR_TOPIC}" _prior_filename:=ins_body_local.csv \
    > "${RUN_DIR}/collector.log" 2>&1 & COLLECTOR_PID=$!
sleep 3
for pid in "${FASTLIO_PID}" "${RTABMAP_PID}" "${COLLECTOR_PID}"; do
    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "A mapping process exited during startup; inspect ${RUN_DIR} logs" >&2
        exit 3
    fi
done

rosrun rtabmap_bringup check_fastlio_sync.py \
    --duration "$(awk "BEGIN { print ${DURATION} + 8 }")" \
    --map-frame rtabmap_ins_map --backend-odom-frame ins_local_odom \
    --output-json "${RUN_DIR}/sync.json" \
    > "${RUN_DIR}/sync.log" 2>&1 & SYNC_PID=$!

if [[ "${USE_INS_PRIOR}" == true ]]; then
    timeout "$(awk "BEGIN { print ${DURATION} + 15 }")" \
        rostopic echo -p /rtabmap/global_pose \
        > "${ANALYSIS_DIR}/ins_position_prior.csv" \
        2> "${RUN_DIR}/ins_position_prior_trace.log" &
    GLOBAL_POSE_TRACE_PID=$!
fi

rosbag play "${BAGS[@]}" --clock -r 1 -u "${DURATION}" \
    > "${RUN_DIR}/rosbag.log" 2>&1 & BAG_PID=$!

# roslaunch can remain alive after its required RTAB-Map child crashes. Abort
# bag playback after three failed node pings so a partial database cannot be
# reported as a successful full-duration run.
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

wait "${BAG_PID}" || BAG_STATUS=$?
BAG_STATUS="${BAG_STATUS:-0}"
wait "${RTABMAP_WATCHDOG_PID}" || true
if [[ "${USE_INS_PRIOR}" == true ]]; then
    kill "${GLOBAL_POSE_TRACE_PID}" 2>/dev/null || true
    wait "${GLOBAL_POSE_TRACE_PID}" 2>/dev/null || true
fi
rosnode kill /collect_fastlio_ins_trajectory >/dev/null 2>&1 || true
for _ in $(seq 1 50); do
    ! kill -0 "${COLLECTOR_PID}" 2>/dev/null && break
    sleep 0.1
done
kill -TERM "${COLLECTOR_PID}" 2>/dev/null || true
wait "${COLLECTOR_PID}" || true
wait "${SYNC_PID}" || SYNC_STATUS=$?
SYNC_STATUS="${SYNC_STATUS:-0}"
if [[ -f "${RTABMAP_FAILURE_FILE}" ]]; then
    SYNC_STATUS=4
fi

# RTAB-Map needs SIGINT for a clean SQLite save. roslaunch propagates it to the
# required child and exits after the database has been closed.
kill -INT "${RTABMAP_PID}" 2>/dev/null || true
wait "${RTABMAP_PID}" || true
rosnode kill /fast_lio_ns/fastlio_mapping >/dev/null 2>&1 || true
for _ in $(seq 1 50); do
    ! kill -0 "${FASTLIO_PID}" 2>/dev/null && break
    sleep 0.1
done
kill -TERM "${FASTLIO_PID}" 2>/dev/null || true
wait "${FASTLIO_PID}" || true

python3 "${RTABMAP_WS}/install_isolated/lib/rtabmap_bringup/analyze_rtabmap_db.py" \
    "${RUN_DIR}/rtabmap.db" --output-dir "${ANALYSIS_DIR}" \
    --fastlio-trajectory "${ANALYSIS_DIR}/fastlio_raw.csv" \
    > "${RUN_DIR}/analysis.log" 2>&1
EVALUATION_REFERENCE_ARGS=()
if [[ "${USE_INS_PRIOR}" == true ]]; then
    EVALUATION_REFERENCE_ARGS=(--ins-reference-csv "${ANALYSIS_DIR}/ins_body_local.csv")
fi
python3 "${RTABMAP_WS}/install_isolated/lib/rtabmap_bringup/evaluate_ins_alignment.py" \
    --ins-csv "${ANALYSIS_DIR}/ins.csv" \
    "${EVALUATION_REFERENCE_ARGS[@]}" \
    --fastlio-raw-csv "${ANALYSIS_DIR}/fastlio_raw.csv" \
    --fastlio-aligned-csv "${ANALYSIS_DIR}/fastlio_aligned.csv" \
    --rtabmap-optimized-csv "${ANALYSIS_DIR}/rtabmap_optimized.csv" \
    --alignment-yaml "${ALIGNMENT_FILE}" --output-dir "${ANALYSIS_DIR}" \
    > "${RUN_DIR}/evaluation.log" 2>&1

printf 'finished=%s\nsync_status=%s\ndatabase=%s\nanalysis=%s\n' \
    "$(date -Is)" "${SYNC_STATUS}" "${RUN_DIR}/rtabmap.db" "${ANALYSIS_DIR}" \
    >> "${STATE_FILE}"
trap - EXIT INT TERM
cleanup
echo "FAST-LIO/RTAB-Map INS-aligned run complete: ${RUN_DIR}"
exit "${SYNC_STATUS}"
