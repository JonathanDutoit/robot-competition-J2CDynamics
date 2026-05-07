#!/usr/bin/env bash
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: This script must be sourced, not executed directly."
  echo "  Run: source ${BASH_SOURCE[0]}"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Find conda binary ─────────────────────────────────────────────────────────
CONDA_BIN="${CONDA_EXE:-$(which conda 2>/dev/null)}"
if [[ -z "$CONDA_BIN" ]]; then
  for p in "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/mambaforge" "$HOME/miniforge3" "/opt/conda"; do
    [[ -f "$p/bin/conda" ]] && CONDA_BIN="$p/bin/conda" && break
  done
fi
if [[ -z "$CONDA_BIN" ]]; then
  echo "ERROR: conda not found." && return 1
fi

# ── Init conda shell hooks ────────────────────────────────────────────────────
__conda_setup="$("$CONDA_BIN" 'shell.bash' 'hook' 2>/dev/null)"
if [[ $? -eq 0 ]]; then eval "$__conda_setup"; else echo "ERROR: conda hook failed." && return 1; fi

# ── Activate conda env ────────────────────────────────────────────────────────
conda activate ros2_sim

# ── Simulation environment ────────────────────────────────────────────────────
export ROS_DOMAIN_ID=99

echo "✔ ROS 2 Humble sim environment ready  [SIMULATION MODE]"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "  CYCLONEDDS=${CYCLONEDDS_URI}"