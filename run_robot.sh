#!/usr/bin/env bash
set -e

ROBOT=$1
ACTION=$2

if [ -z "$ROBOT" ] || [ -z "$ACTION" ]; then
  echo "Usage:"
  echo "  ./run_robot.sh da up"
  echo "  ./run_robot.sh da down"
  exit 1
fi

ENV_FILE="robots/${ROBOT}/manifest.env"

if [ ! -f "$ENV_FILE" ]; then
  echo "Robot not found: $ROBOT"
  exit 1
fi

# Load env for this shell (optional but useful)
set -a
source "$ENV_FILE"
set +a

echo "Robot: $ROBOT"
echo "Action: $ACTION"
echo "Env: $ENV_FILE"

PROJECT_NAME="robot_${ROBOT}"

case "$ACTION" in
  up)
    docker compose \
      --env-file "$ENV_FILE" \
      -p "$PROJECT_NAME" \
      up -d --build
    ;;
    
  down)
    docker compose \
      --env-file "$ENV_FILE" \
      -p "$PROJECT_NAME" \
      down
    ;;

  restart)
    docker compose \
      --env-file "$ENV_FILE" \
      -p "$PROJECT_NAME" \
      down
    docker compose \
      --env-file "$ENV_FILE" \
      -p "$PROJECT_NAME" \
      up -d --build
    ;;

  logs)
    docker compose \
      --env-file "$ENV_FILE" \
      -p "$PROJECT_NAME" \
      logs -f
    ;;

  *)
    echo "Unknown action: $ACTION"
    echo "Use: up | down | restart | logs"
    exit 1
    ;;
esac