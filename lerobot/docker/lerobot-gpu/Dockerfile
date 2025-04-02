# CUDA 12.8 기반 Ubuntu 24.04 (Jazzy) 사용
FROM nvidia/cuda:12.8.0-base-ubuntu24.04

# 환경 변수 설정
ARG PYTHON_VERSION=3.12
ARG DEBIAN_FRONTEND=noninteractive

# 필수 패키지 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git git-lfs \
    libglib2.0-0 libgl1 libegl1 ffmpeg \
    speech-dispatcher libgeos-dev \
    python${PYTHON_VERSION}-dev python${PYTHON_VERSION}-venv \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Python 심볼릭 링크 생성
RUN ln -s /usr/bin/python${PYTHON_VERSION} /usr/bin/python

# 가상 환경 생성 및 활성화
RUN python -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
RUN echo "source /opt/venv/bin/activate" >> /root/.bashrc

# LeRobot 설치
RUN git lfs install
RUN git clone https://github.com/huggingface/lerobot.git /lerobot
WORKDIR /lerobot
RUN pip install --upgrade --no-cache-dir pip
RUN pip install --no-cache-dir ".[dynamixel]"

# PyTorch (CUDA 12.8 지원) 설치
RUN pip install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128

