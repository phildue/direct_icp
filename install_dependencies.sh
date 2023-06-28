#!/bin/bash

apt-get update && apt-get -y --no-install-recommends install \
    build-essential \
    clang \
    cmake \
    wget \
    libeigen3-dev \
    libfmt-dev \
    python2 \
    ca-certificates \
    curl \
    git \
    libopencv-dev

curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py && python2.7 get-pip.py

pip install numpy    

git clone https://github.com/strasdat/Sophus.git &&\
cd Sophus && mkdir build && cd build && cmake .. -D BUILD_SOPHUS_TESTS=OFF -D BUILD_SOPHUS_EXAMPLES=OFF && \
make -j2 && make install

