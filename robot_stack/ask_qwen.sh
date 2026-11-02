#!/usr/bin/env bash
# Ask the agent to do a task. The web console must already be running.
#   ./ask_qwen.sh "find the red cup"          local Qwen on this GPU (default)
#   ./ask_qwen.sh                             interactive: type tasks, /reset, /quit
#   QWEN_BACKEND=openai QWEN_URL=https://api.openai.com/v1 QWEN_MODEL=gpt-4o ./ask_qwen.sh "..."
#       that one needs OPENAI_API_KEY in the environment
# First time for local Qwen only: ./qwen_agent/setup_env.sh
set -euo pipefail
STACK=$(dirname "$(readlink -f "$0")")
exec "$STACK/qwen_agent/run.sh" "$@"
