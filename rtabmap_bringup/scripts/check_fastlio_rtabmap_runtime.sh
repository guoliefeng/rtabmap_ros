#!/usr/bin/env bash
set -euo pipefail

ODOM_TOPIC="${FASTLIO_ODOM_TOPIC:-/fast_lio_ns/loc_result}"
CLOUD_TOPIC="${FASTLIO_CLOUD_TOPIC:-/fast_lio_ns/cloud_registered_body}"
FASTLIO_ODOM_FRAME="${FASTLIO_ODOM_FRAME:-map}"
BASE_FRAME="${FASTLIO_BASE_FRAME:-base_link_fast_lio}"
RTABMAP_MAP_FRAME="${RTABMAP_MAP_FRAME:-rtabmap_map}"

check_topic() {
  local topic="$1"
  if rostopic list 2>/dev/null | grep -Fxq "$topic"; then
    echo "PASS: topic advertised: $topic"
  else
    echo "FAIL: topic not advertised: $topic"
  fi
}

sample_rate() {
  local topic="$1"
  echo "INFO: sampling $topic"
  timeout 8s rostopic hz -w 3 "$topic" 2>&1 | sed -n '1,12p' || true
}

check_tf() {
  local parent="$1"
  local child="$2"
  local output
  output="$(timeout 6s rosrun tf tf_echo "$parent" "$child" 2>&1 || true)"
  if grep -q "Translation:" <<<"$output"; then
    echo "PASS: TF $parent <- $child"
    sed -n '1,12p' <<<"$output"
  else
    echo "FAIL: missing TF $parent <- $child"
    sed -n '1,6p' <<<"$output"
  fi
}

echo "## FAST-LIO + RTAB-Map Mode B runtime checks"
for topic in "$ODOM_TOPIC" "$CLOUD_TOPIC" /rtabmap/info /rtabmap/mapPath; do
  check_topic "$topic"
done

for topic in "$ODOM_TOPIC" "$CLOUD_TOPIC" /rtabmap/info; do
  sample_rate "$topic"
done

check_tf "$FASTLIO_ODOM_FRAME" "$BASE_FRAME"
check_tf "$RTABMAP_MAP_FRAME" "$FASTLIO_ODOM_FRAME"

if rosnode list 2>/dev/null | grep -Fxq /rtabmap/rtabmap; then
  echo "PASS: RTAB-Map backend node is running"
else
  echo "FAIL: /rtabmap/rtabmap is not running"
fi

db_path="$(rosparam get /rtabmap/database_path 2>/dev/null | tr -d "'" || true)"
if [[ -n "$db_path" && -f "$db_path" ]]; then
  echo "PASS: database exists: $db_path ($(stat -c %s "$db_path") bytes)"
  if command -v sqlite3 >/dev/null; then
    echo "INFO: Node count: $(sqlite3 "$db_path" 'select count(*) from Node;' 2>/dev/null || echo unavailable)"
  fi
else
  echo "FAIL: database file is unavailable"
fi
