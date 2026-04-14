#!/usr/bin/env bash
# !! Must be run as: source conda/setup_dev.sh  (not bash conda/setup_dev.sh)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/../docker/.env"
DDS_CONFIG="$SCRIPT_DIR/../dds/cyclonedds_dev.xml"

# ── Init conda shell hooks (required for conda activate to work) ──────────────
__conda_setup="$("${CONDA_PREFIX:-$HOME/miniconda3}/bin/conda" 'shell.bash' 'hook' 2>/dev/null)"
if [ $? -eq 0 ]; then
  eval "$__conda_setup"
else
  echo "ERROR: conda not found. Is miniconda/mambaforge installed?" && return 1
fi

# ── Load .env ─────────────────────────────────────────────────────────────────
if [[ -f "$ENV_FILE" ]]; then
  set -a  # auto-export all variables
  source "$ENV_FILE"
  set +a
fi

# ── Activate conda env ────────────────────────────────────────────────────────
conda activate ros2_dev

# ── Set CycloneDDS ────────────────────────────────────────────────────────────
export CYCLONEDDS_URI="file://${DDS_CONFIG}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ── Source the workspace if built ─────────────────────────────────────────────
if [[ -f "$SCRIPT_DIR/../ros2_ws/install/setup.bash" ]]; then
  source "$SCRIPT_DIR/../ros2_ws/install/setup.bash"
fi

echo "✔ ROS 2 Humble dev environment ready"
echo "  CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}"