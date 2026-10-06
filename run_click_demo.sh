#!/usr/bin/env bash
# Mở Webots demo click chọn lỗ (Play rồi click chấm đen).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
export WEBOTS_HOME="${WEBOTS_HOME:-/usr/local/webots}"
cd "$ROOT"
# Dọn instance cũ nếu còn
pkill -f 'webots-bin' 2>/dev/null || true
sleep 0.5
exec webots "$ROOT/worlds/ur3_needle.wbt"
