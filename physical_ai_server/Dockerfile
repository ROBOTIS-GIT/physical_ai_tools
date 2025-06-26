FROM robotis/ros:jazzy-ros-base-torch2.7.0-cuda12.8.0 AS physical-ai-tools

ARG TARGETARCH

RUN if [ "$TARGETARCH" = "arm64" ]; then \
    echo 'export PATH=/opt/venv/bin:$PATH' >> /root/.bashrc && \
    echo 'export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:$PYTHONPATH' >> /root/.bashrc && \
    export PATH=/opt/venv/bin:$PATH && \
    export PYTHONPATH=/opt/venv/lib/python3.12/site-packages:$PYTHONPATH; \
    fi

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install realsense and physical_ai_tools dependencies
RUN apt-get update && apt-get install -y \
    ffmpeg \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

ENV COLCON_WS=/root/ros2_ws
WORKDIR ${COLCON_WS}

RUN mkdir -p ${COLCON_WS}/src && \
    cd ${COLCON_WS}/src && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/physical_ai_tools.git --recursive

RUN cd ${COLCON_WS} && \
    apt-get update && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys="physical_ai_tools" -y && \
    rm -rf /var/lib/apt/lists/*

RUN cd ${COLCON_WS}/src/physical_ai_tools/lerobot && \
    pip install --ignore-installed -e .

RUN pip install setuptools==68.1.2
RUN pip install 'numpy<2'

RUN if [ "$TARGETARCH" = "arm64" ]; then \
    echo 'export PYTHONPATH=${COLCON_WS}/src/physical_ai_tools/lerobot:$PYTHONPATH' >> /root/.bashrc && \
    export PYTHONPATH=${COLCON_WS}/src/physical_ai_tools/lerobot:$PYTHONPATH; \
    fi

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${COLCON_WS} && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc  && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc  && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc && \
    echo "alias ai_server='ros2 launch physical_ai_server physical_ai_server_bringup.launch.py'" >> ~/.bashrc
