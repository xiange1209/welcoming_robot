#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[deprecated] setup_rpi4_quick.sh 已改為相容入口，實際部署流程請使用 setup_ubuntu.sh。"
exec bash "$SCRIPT_DIR/setup_ubuntu.sh" "$@"
