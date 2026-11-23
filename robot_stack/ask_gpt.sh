#!/usr/bin/env bash
# Ask GPT to do a task. The web console must already be running.
#   OPENAI_API_KEY=sk-... ./ask_gpt.sh "find the red cup"     gpt-4o
#   ./ask_gpt.sh                                              interactive: type tasks, /reset, /quit
#   QWEN_MODEL=gpt-4.1 ./ask_gpt.sh "..."                     another OpenAI vision model
set -euo pipefail
STACK=$(dirname "$(readlink -f "$0")")
export QWEN_BACKEND=openai
export QWEN_URL=${QWEN_URL:-https://api.openai.com/v1}
export QWEN_MODEL=${QWEN_MODEL:-gpt-4o}
export GO2_AGENT_NAME=${GO2_AGENT_NAME:-gpt-agent}
exec "$STACK/qwen_agent/run.sh" "$@"
