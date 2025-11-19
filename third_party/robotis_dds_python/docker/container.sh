#!/bin/bash

# Directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONTAINER_NAME="robotis_dds_python"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"

# ---------------------- Helper ----------------------
show_help() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  help        Show this help message"
    echo "  start       Start the DDS container"
    echo "  enter       Enter the running container (bash)"
    echo "  stop        Stop and remove the container"
    echo ""
    echo "Examples:"
    echo "  $0 start"
    echo "  $0 enter"
    echo "  $0 stop"
}

# ---------------------- Start ----------------------
start_container() {
    # Setup X11 forwarding if DISPLAY exists
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker >/dev/null 2>&1 || true
    else
        echo "Warning: DISPLAY is not set. GUI apps will not work."
    fi

    echo "Starting DDS container..."

    # Pull latest image if available
    docker compose -f "$COMPOSE_FILE" pull

    # Start container
    docker compose -f "$COMPOSE_FILE" up -d

    echo "Container started: $CONTAINER_NAME"
}

# ---------------------- Enter ----------------------
enter_container() {
    # Check running container
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "Error: Container '$CONTAINER_NAME' is not running."
        exit 1
    fi

    # Setup X11 forwarding
    if [ -n "$DISPLAY" ]; then
        echo "Setting up X11 forwarding..."
        xhost +local:docker >/dev/null 2>&1 || true
    else
        echo "Warning: DISPLAY is not set. GUI apps will not work."
    fi

    docker exec -it "$CONTAINER_NAME" bash
}

# ---------------------- Stop ----------------------
stop_container() {
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "Error: Container '$CONTAINER_NAME' is not running."
        exit 1
    fi

    echo "Warning: This will stop and remove the container."
    read -p "Are you sure? [y/N] " -n 1 -r
    echo

    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker compose -f "$COMPOSE_FILE" down
        echo "Container removed."
    else
        echo "Cancelled."
        exit 0
    fi
}

# ---------------------- Main Switch ----------------------
case "$1" in
    "help")
        show_help
        ;;
    "start")
        start_container
        ;;
    "enter")
        enter_container
        ;;
    "stop")
        stop_container
        ;;
    *)
        echo "Error: Unknown command '$1'"
        show_help
        exit 1
        ;;
esac
