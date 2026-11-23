#!/usr/bin/env bash
# Shared runner for ask_qwen.sh (local Qwen) and ask_gpt.sh (OpenAI). The agent runs on this PC,
# outside docker, and only talks to the console at http://localhost:8080.
#   QWEN_BACKEND=local ./run.sh "find the red cup"
#   QWEN_BACKEND=openai QWEN_MODEL=gpt-4o ./run.sh "find the red cup"    needs OPENAI_API_KEY
#   QWEN_URL=http://127.0.0.1:8000/v1 QWEN_MODEL=Qwen/Qwen3-VL-8B-Instruct QWEN_BACKEND=openai ./run.sh
#       a local vLLM server; that URL does not use OpenAI function tools
# Watch it on the console; Take over or STOP there always wins.
# Uses the conda env "qwen_agent" in /home/khang/miniconda3. First time for local Qwen: ./setup_env.sh
set -euo pipefail
HERE=$(dirname "$(readlink -f "$0")")
CONDA=${CONDA:-/home/khang/miniconda3/bin/conda}
ENV_NAME=${CONDA_ENV:-qwen_agent}
export QWEN_BACKEND=${QWEN_BACKEND:-local}
if [ "$QWEN_BACKEND" = "openai" ]; then
    export QWEN_URL=${QWEN_URL:-https://api.openai.com/v1}
    export QWEN_MODEL=${QWEN_MODEL:-gpt-4o}
    export GO2_AGENT_NAME=${GO2_AGENT_NAME:-gpt-agent}
else
    export GO2_AGENT_NAME=${GO2_AGENT_NAME:-qwen-agent}
fi
curl -sf -o /dev/null http://localhost:8080/ || { echo "The console isn't running: start it with ./start_console.sh"; exit 1; }
PY="$("$CONDA" env list | awk -v n="$ENV_NAME" '$1==n {print $NF}')/bin/python"
[ -x "$PY" ] || { echo "Conda env $ENV_NAME is missing: run $HERE/setup_env.sh first"; exit 1; }
export CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-1}
export PYTORCH_CUDA_ALLOC_CONF=${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}
cd "$HERE"
exec "$PY" "$HERE/agent.py" "$@"
