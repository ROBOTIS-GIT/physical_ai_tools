#!/bin/bash
# LeRobot Docker Container Entrypoint
# Supports multiple run modes: server, train, infer, shell

set -e

# Ensure cache directories exist
mkdir -p "${HF_HOME:-/root/.cache/huggingface}"
mkdir -p "${HF_LEROBOT_HOME:-/root/.cache/huggingface/lerobot}"
mkdir -p "${TORCH_HOME:-/root/.cache/torch}"

# Function to print banner
print_banner() {
    echo "=============================================="
    echo "  LeRobot Docker Container"
    echo "  Mode: $1"
    echo "  Zenoh Router: ${ZENOH_ROUTER_IP}:${ZENOH_ROUTER_PORT}"
    echo "  SHM Enabled: ${ZENOH_SHM_ENABLED:-false}"
    echo "=============================================="
}

# Function to start Zenoh server mode
start_server() {
    print_banner "Zenoh Server"
    echo "Starting LeRobot Zenoh server..."
    echo "Waiting for commands from physical_ai_server..."
    exec python /app/lerobot_zenoh_server.py
}

# Function to run training directly
run_train() {
    print_banner "Training"
    shift  # Remove 'train' from arguments
    echo "Starting training with args: $@"
    exec python -m lerobot.scripts.train "$@"
}

# Function to run inference directly
run_infer() {
    print_banner "Inference"
    shift  # Remove 'infer' from arguments
    echo "Starting inference with args: $@"
    exec python -m lerobot.scripts.eval "$@"
}

# Function to run interactive shell
run_shell() {
    print_banner "Interactive Shell"
    exec /bin/bash
}

# Main entrypoint logic
case "${1:-server}" in
    server)
        start_server
        ;;
    train)
        run_train "$@"
        ;;
    infer|inference|eval)
        run_infer "$@"
        ;;
    shell|bash)
        run_shell
        ;;
    *)
        # If unknown command, pass to python directly
        echo "Running custom command: $@"
        exec "$@"
        ;;
esac
