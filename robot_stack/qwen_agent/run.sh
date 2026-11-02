#!/usr/bin/env bash
# Give the robot dog a task and let a local Qwen3-VL agent do it. Runs on this PC (GPU 1 by
# default), outside docker; it only talks to the console at http://localhost:8080.
#   ./run.sh "find the red cup"                       one task, local Qwen on this GPU
#   ./run.sh                                          interactive (type tasks)
#   QWEN_MODEL=Qwen/Qwen2.5-VL-7B-Instruct ./run.sh "..."       another local Qwen
#   QWEN_BACKEND=openai QWEN_URL=http://127.0.0.1:8000/v1 ./run.sh "..."   Qwen served by vLLM
#   QWEN_BACKEND=openai QWEN_URL=https://api.openai.com/v1 QWEN_MODEL=gpt-4o ./run.sh "..."
#       uses OPENAI_API_KEY; the model must accept images
# Watch it on the console; Take over or STOP there always wins.
# Uses the conda env "qwen_agent" in /home/khang/miniconda3. First time: ./setup_env.sh
set -euo pipefail
HERE=$(dirname "$(readlink -f "$0")")
CONDA=${CONDA:-/home/khang/miniconda3/bin/conda}
ENV_NAME=${CONDA_ENV:-qwen_agent}
curl -sf -o /dev/null http://localhost:8080/ || { echo "The console isn't running: start it with run_go2_keys.sh"; exit 1; }
PY="$("$CONDA" env list | awk -v n="$ENV_NAME" '$1==n {print $NF}')/bin/python"
[ -x "$PY" ] || { echo "Conda env $ENV_NAME is missing: run $HERE/setup_env.sh first"; exit 1; }
export CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-1}
export PYTORCH_CUDA_ALLOC_CONF=${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}
export GO2_AGENT_NAME=${GO2_AGENT_NAME:-qwen-agent}
cd "$HERE"
exec "$PY" "$HERE/agent.py" "$@"
