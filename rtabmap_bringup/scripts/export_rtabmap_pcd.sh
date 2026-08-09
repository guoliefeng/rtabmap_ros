#!/usr/bin/env bash
# Export all optimized graph components as one GPS-aligned 3D cloud.
set -euo pipefail

RTABMAP_WS="${RTABMAP_WS:-/home/glf/dataDisk/Study/rtabMap_ws}"
DATABASE=""
EXPORT_DIR=""
ROS_PORT="${ROS_PORT:-11450}"
VOXEL_SIZE="${VOXEL_SIZE:-0.25}"

usage() {
    cat <<'EOF'
Usage: export_rtabmap_pcd.sh --database FILE --output DIR [--voxel-size M] [--port PORT]

Loads the database read-only, decompresses the retained LiDAR scans, transforms
all graph components with optimized poses, and writes one voxelized binary PCD.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --database) DATABASE="$2"; shift 2 ;;
        --output) EXPORT_DIR="$2"; shift 2 ;;
        --voxel-size) VOXEL_SIZE="$2"; shift 2 ;;
        --port) ROS_PORT="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

NUMBER_PATTERN='^[0-9]+([.][0-9]+)?$'
if [[ -z "${DATABASE}" || -z "${EXPORT_DIR}" || ! -f "${DATABASE}" ||
      ! "${VOXEL_SIZE}" =~ ${NUMBER_PATTERN} || ! "${ROS_PORT}" =~ ^[0-9]+$ ]]; then
    usage >&2
    exit 2
fi

FINAL_PCD="${EXPORT_DIR}/yangpu_gps_multisession.pcd"
if [[ -e "${FINAL_PCD}" ]]; then
    echo "Refusing to overwrite existing PCD: ${FINAL_PCD}" >&2
    exit 2
fi
mkdir -p "${EXPORT_DIR}"
TEMP_DIR="$(mktemp -d)"
WORK_DATABASE="${TEMP_DIR}/rtabmap.db"
cp --reflink=auto --preserve=timestamps "${DATABASE}" "${WORK_DATABASE}"

source /opt/ros/noetic/setup.bash
source "${RTABMAP_WS}/install_isolated/setup.bash"
export LD_LIBRARY_PATH="${HOME}/opt/rtabmap-noetic/lib:${RTABMAP_WS}/install_isolated/lib:${HOME}/opt/ros-deps/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"

cleanup() {
    [[ -f "${WORK_DATABASE}" ]] && rm -f "${WORK_DATABASE}"
    [[ -d "${TEMP_DIR}" ]] && rmdir "${TEMP_DIR}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

"${RTABMAP_WS}/install_isolated/lib/rtabmap_bringup/rtabmap_db_to_pcd" \
    --database "${WORK_DATABASE}" --output "${FINAL_PCD}" \
    --voxel-size "${VOXEL_SIZE}" --range-min 2.0 --range-max 100.0 \
    > "${EXPORT_DIR}/database_to_pcd.log" 2>&1
python3 "${RTABMAP_WS}/install_isolated/lib/rtabmap_bringup/inspect_pcd.py" \
    "${FINAL_PCD}" --output-dir "${EXPORT_DIR}/inspection" \
    > "${EXPORT_DIR}/inspection.log" 2>&1
printf 'database=%s\npcd=%s\nvoxel_size_m=%s\nfinished=%s\n' \
    "${DATABASE}" "${FINAL_PCD}" "${VOXEL_SIZE}" "$(date -Is)" \
    > "${EXPORT_DIR}/export_status.txt"

trap - EXIT INT TERM
cleanup
echo "PCD export complete: ${FINAL_PCD}"
