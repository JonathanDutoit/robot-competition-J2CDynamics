#!/usr/bin/env bash
# !! Must be run as: source setup_dev.sh  (not bash setup_dev.sh)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/../docker/.env"
DDS_CONFIG="$SCRIPT_DIR/../dds/cyclonedds_dev.xml"

# ── Init conda shell hooks (required for conda activate to work) ──────────────
# Find conda binary
CONDA_BIN="${CONDA_EXE:-$(which conda 2>/dev/null)}"
if [[ -z "$CONDA_BIN" ]]; then
    for p in "$HOME/miniconda3" "$HOME/anaconda3" "$HOME/mambaforge" "$HOME/miniforge3" "/opt/conda"; do
        [[ -f "$p/bin/conda" ]] && CONDA_BIN="$p/bin/conda" && break
    done
fi

if [[ -z "$CONDA_BIN" ]]; then
    echo "ERROR: conda not found." && return 1
fi

__conda_setup="$("$CONDA_BIN" 'shell.bash' 'hook' 2>/dev/null)"

# ── Load .env ─────────────────────────────────────────────────────────────────
if [[ -f "$ENV_FILE" ]]; then
  while IFS='=' read -r key value; do
    # skip comments, blank lines, and readonly bash vars
    [[ "$key" =~ ^#.*$ || -z "$key" ]] && continue
    [[ "$key" =~ ^(UID|GID|USER|GROUPS|BASH.*)$ ]] && continue
    export "$key"="$value"
  done < "$ENV_FILE"
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