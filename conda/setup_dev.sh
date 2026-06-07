#!/usr/bin/env bash
# !! Must be run as: source setup_dev.sh [robot_name]
# Examples:
#   source setup_dev.sh do     # Loads docker/do.env
#   source setup_dev.sh da     # Loads docker/da.env
#   source setup_dev.sh        # Loads docker/.env (default)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Parse robot name argument ─────────────────────────────────────────────────
ROBOT_NAME="${1:-}"  # First argument, empty if not provided

if [[ -n "$ROBOT_NAME" ]]; then
    ENV_FILE="$SCRIPT_DIR/../docker/${ROBOT_NAME}.env"
    echo "Loading environment for robot: $ROBOT_NAME"
else
    ENV_FILE="$SCRIPT_DIR/../docker/.env"
    echo "Loading default environment"
fi

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

# ── Load env file ─────────────────────────────────────────────────────────────
if [[ -f "$ENV_FILE" ]]; then
  while IFS='=' read -r key value; do
    # skip comments, blank lines, and readonly bash vars
    [[ "$key" =~ ^#.*$ || -z "$key" ]] && continue
    [[ "$key" =~ ^(UID|GID|USER|GROUPS|BASH.*)$ ]] && continue
    export "$key"="$value"
  done < "$ENV_FILE"
  echo "✔ Loaded: $ENV_FILE"
else
  echo "⚠ WARNING: Env file not found: $ENV_FILE"
fi

# ── Set robot name explicitly ─────────────────────────────────────────────────
if [[ -n "$ROBOT_NAME" ]]; then
    export ROBOT_NAME="$ROBOT_NAME"
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
echo "  Robot:        ${ROBOT_NAME:-default}"
echo "  CYCLONEDDS_URI=${CYCLONEDDS_URI}"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}"