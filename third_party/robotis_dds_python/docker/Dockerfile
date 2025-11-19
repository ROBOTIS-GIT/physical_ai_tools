FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    libssl-dev \
    python3 \
    python3-pip \
    python3-dev \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

RUN git clone https://github.com/eclipse-cyclonedds/cyclonedds.git && \
    cd cyclonedds && \
    git checkout 0.10.2 && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX=/root/cyclonedds/install -DBUILD_EXAMPLES=ON .. && \
    cmake --build . && \
    cmake --install .

RUN echo 'export CYCLONEDDS_HOME=/root/cyclonedds/install' >> ~/.bashrc && \
    echo 'export CMAKE_PREFIX_PATH=$CYCLONEDDS_HOME:$CMAKE_PREFIX_PATH' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=$CYCLONEDDS_HOME/lib:$LD_LIBRARY_PATH' >> ~/.bashrc && \
    echo 'export PATH=$CYCLONEDDS_HOME/bin:$PATH' >> ~/.bashrc


WORKDIR /root/physical_ai_tools/third_party/robotis_dds_python
RUN pip3 install -e .

WORKDIR /root/physical_ai_tools/third_party/Isaac-GR00T
RUN pip3 install -e .

CMD ["bash"]
