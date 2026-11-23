#!/usr/bin/env bash
# Ask local Qwen to do a task. The web console must already be running.
#   ./ask_qwen.sh "find the red cup"     Qwen on this GPU
#   ./ask_qwen.sh                        interactive: type tasks, /reset, /quit
#   QWEN_MODEL=Qwen/Qwen2.5-VL-7B-Instruct ./ask_qwen.sh "..."
# First time: ./qwen_agent/setup_env.sh
set -euo pipefail
STACK=$(dirname "$(readlink -f "$0")")
export QWEN_BACKEND=local
export GO2_AGENT_NAME=${GO2_AGENT_NAME:-qwen-agent}
exec "$STACK/qwen_agent/run.sh" "$@"
