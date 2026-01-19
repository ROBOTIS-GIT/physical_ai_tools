#!/bin/bash
# Reusable ROS2 service run script template
# This script launches ROS2 commands for any service
# Usage:
#   SERVICE_NAME=<name> ROS2_COMMAND="<full command>" /path/to/ros2_service_run.sh
#   If ROS2_COMMAND is not set, defaults to: ros2 launch physical_ai_server ${SERVICE_NAME}.launch.py
# Note: This script is called via /command/with-contenv, so environment is already set up

set -e

# Service name must be provided via environment variable
SERVICE_NAME="${SERVICE_NAME}"
if [ -z "${SERVICE_NAME}" ]; then
    echo "Error: SERVICE_NAME environment variable must be set" >&2
    exit 1
fi

# Set ROS_DOMAIN_ID if not already set (default to 30)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
export COLCON_WS=${COLCON_WS:-/root/ros2_ws}
export PATH=/opt/venv/bin:${PATH}
export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:${PYTHONPATH}
export PYTHONPATH=${COLCON_WS}/src/physical_ai_tools/lerobot/src:${PYTHONPATH}

# Enable s6-overlay debug logging (set S6_VERBOSITY=1 for more verbose output)
export S6_VERBOSITY=${S6_VERBOSITY:-1}

echo "[${SERVICE_NAME}] Starting service..."
echo "[${SERVICE_NAME}] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[${SERVICE_NAME}] ROS_DISTRO=${ROS_DISTRO}"
echo "[${SERVICE_NAME}] COLCON_WS=${COLCON_WS}"
echo "[${SERVICE_NAME}] PID: $$"

# Record process group ID so the finish script can target the whole group
PGID=$(ps -o pgid= -p $$ | tr -d ' ')
echo "[${SERVICE_NAME}] Process group: ${PGID}"
echo "${PGID}" > /run/${SERVICE_NAME}.pgid || true

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${COLCON_WS}/install/setup.bash

# Determine the command to execute
# If ROS2_COMMAND is set, use it; otherwise, use the default launch command
if [ -n "${ROS2_COMMAND}" ]; then
    ROS2_CMD="${ROS2_COMMAND}"
    echo "[${SERVICE_NAME}] Executing custom command: ${ROS2_CMD}"
else
    ROS2_CMD="ros2 launch physical_ai_server ${SERVICE_NAME}.launch.py"
    echo "[${SERVICE_NAME}] Executing default command: ${ROS2_CMD}"
fi

# Execute the ROS2 command
# Using 'exec' ensures the command becomes PID 1 of this service,
# which allows s6 to properly signal it and its children
# Note: stdout/stderr are automatically piped to ${SERVICE_NAME}-log via producer-for/consumer-for
exec bash -c "${ROS2_CMD}"

