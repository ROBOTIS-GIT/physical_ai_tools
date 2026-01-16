# LeRobot Docker Container with Zenoh Communication Support
# This container provides isolated LeRobot environment with Zenoh-based communication
# for training and inference commands from physical_ai_server (ROS2)

FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set up locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install system dependencies (cmake, ninja-build, pkg-config required for egl-probe)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    curl \
    wget \
    ca-certificates \
    cmake \
    pkg-config \
    ninja-build \
    libglib2.0-0 \
    libegl1-mesa \
    libegl1-mesa-dev \
    libgl1-mesa-glx \
    libgles2-mesa \
    ffmpeg \
    libusb-1.0-0-dev \
    speech-dispatcher \
    libgeos-dev \
    portaudio19-dev \
    python3.11 \
    python3.11-venv \
    python3.11-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install uv (fast Python package manager)
RUN curl -LsSf https://astral.sh/uv/install.sh | sh \
    && mv /root/.local/bin/uv /usr/local/bin/uv

# Set Python 3.11 as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.11 1 \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1

# Create working directory for LeRobot
WORKDIR /lerobot

# Create virtual environment
RUN uv venv /lerobot/.venv --python 3.11

# Set virtual environment as default
ENV VIRTUAL_ENV=/lerobot/.venv
ENV PATH="/lerobot/.venv/bin:$PATH"

# Copy LeRobot source and install dependencies
# Note: Build context is project root (physical_ai_tools)
# Must include setup.py and MANIFEST.in for dynamic readme resolution
COPY third_party/lerobot/setup.py third_party/lerobot/pyproject.toml third_party/lerobot/README.md third_party/lerobot/MANIFEST.in ./
COPY third_party/lerobot/src/ ./src/

# Install LeRobot with all dependencies
RUN uv pip install --no-cache ".[all]"

# Copy the rest of LeRobot (configs, tests, etc.)
COPY third_party/lerobot/ ./

# Zenoh dependencies (zenoh_ros2_sdk mounted as volume at /zenoh_sdk)
# Use uv pip to install into virtual environment
RUN uv pip install eclipse-zenoh>=1.0.0 rosbags>=0.11.0 GitPython>=3.1.18 \
    && mkdir -p /zenoh_sdk

# Create application directory for Zenoh server
WORKDIR /app

# Copy entrypoint and server scripts
COPY docker/lerobot/entrypoint.sh /entrypoint.sh
COPY docker/lerobot/lerobot_zenoh_server.py /app/

# Make entrypoint executable
RUN chmod +x /entrypoint.sh

# Environment variables for HuggingFace cache (mapped via volume)
ENV HF_HOME=/root/.cache/huggingface
ENV HF_LEROBOT_HOME=/root/.cache/huggingface/lerobot
ENV TORCH_HOME=/root/.cache/torch

# Zenoh configuration
ENV ZENOH_ROUTER_IP=127.0.0.1
ENV ZENOH_ROUTER_PORT=7447
ENV ZENOH_SHM_ENABLED=true

# Shared Memory configuration for Zenoh
# These are typically set via docker-compose with ipc: host
ENV ZENOH_TRANSPORT_SHM_ENABLED=true

# Default working directory
WORKDIR /lerobot

# Healthcheck
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python -c "import zenoh; print('Zenoh OK')" || exit 1

# Entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command: start Zenoh server for receiving training/inference commands
CMD ["server"]
