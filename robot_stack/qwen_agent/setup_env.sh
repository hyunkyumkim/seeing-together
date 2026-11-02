#!/usr/bin/env bash
# One-time setup, on the host (nothing is installed in the docker image).
# Creates the conda env "qwen_agent" (Python 3.10) in /home/khang/miniconda3/envs/qwen_agent,
# installs torch, transformers and qwen-vl-utils there, then fetches the model weights
# into ~/.cache/huggingface (about 17 GB for the 8B model).
#   ./setup_env.sh                                   default model Qwen/Qwen3-VL-8B-Instruct
#   QWEN_MODEL=Qwen/Qwen2.5-VL-7B-Instruct ./setup_env.sh
# Base Miniconda is Python 3.14, which this stack does not support, so the env is separate.
set -euo pipefail
HERE=$(dirname "$(readlink -f "$0")")
CONDA=${CONDA:-/home/khang/miniconda3/bin/conda}
ENV_NAME=${CONDA_ENV:-qwen_agent}
MODEL=${QWEN_MODEL:-Qwen/Qwen3-VL-8B-Instruct}
[ -x "$CONDA" ] || { echo "No conda at $CONDA"; exit 1; }
if ! "$CONDA" env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    "$CONDA" create -y -n "$ENV_NAME" python=3.10
fi
PY="$("$CONDA" env list | awk -v n="$ENV_NAME" '$1==n {print $NF}')/bin/python"
PIP="$("$CONDA" env list | awk -v n="$ENV_NAME" '$1==n {print $NF}')/bin/pip"
"$PIP" install -q -U pip
"$PIP" install -q torch torchvision --index-url https://download.pytorch.org/whl/cu126
"$PIP" install -q -r "$HERE/requirements.txt"
"$PY" - <<EOF
import torch, transformers
print("torch", torch.__version__, "cuda", torch.cuda.is_available(), "| transformers", transformers.__version__)
EOF
echo "Fetching $MODEL (resumable; re-run if it stops)..."
"$PY" -c "from huggingface_hub import snapshot_download; print(snapshot_download('$MODEL'))"
echo "Done. conda activate $ENV_NAME"
echo "Start the console (run_go2_keys.sh), then: $HERE/run.sh \"find the red cup\""
