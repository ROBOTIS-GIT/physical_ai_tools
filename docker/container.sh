#!/bin/bash

# Get the directory where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
CONTAINER_NAME="physical_ai_manager"

# Function to display help
show_help() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  help           Show this help message"
    echo "  start          Start the container"
    echo "  enter          Enter the running container"
    echo "  stop           Stop the container"
    echo ""
    echo "Examples:"
    echo "  $0 start       Start the container"
    echo "  $0 enter       Enter the running container"
    echo "  $0 stop        Stop the container"
}

# Function to start the container
start_container() {
    docker compose -f "$SCRIPT_DIR/docker-compose.yml" up -d --build physical_ai_manager
}

# Function to enter the container
enter_container() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi
    docker exec -it "$CONTAINER_NAME" sh
}

# Function to stop the container
stop_container() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        echo "Error: Container is not running"
        exit 1
    fi
    echo "Warning: This will stop and remove the container. All unsaved data in the container will be lost."
    read -p "Are you sure you want to continue? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker compose -f "$SCRIPT_DIR/docker-compose.yml" stop "$CONTAINER_NAME"
        docker compose -f "$SCRIPT_DIR/docker-compose.yml" rm -f "$CONTAINER_NAME"
    else
        echo "Operation cancelled."
        exit 0
    fi
}

# Main command handling
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
        echo "Error: Unknown command"
        show_help
        exit 1
        ;;
esac
