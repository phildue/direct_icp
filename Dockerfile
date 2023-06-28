FROM ubuntu:20.04
LABEL Description="Build environment"

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get -y --no-install-recommends install \
    build-essential \
    clang \
    cmake \
    wget \
    libeigen3-dev \
    libfmt-dev
RUN apt-get install -y --no-install-recommends git libopencv-dev
WORKDIR /opt    
#RUN git clone https://github.com/strasdat/Sophus.git && cd Sophus && mkdir build && cd build && cmake .. -D BUILD_SOPHUS_TESTS=OFF -D BUILD_SOPHUS_EXAMPLES=OFF && make -j2 && make install && rm /opt/Sophus -r
ADD Sophus /opt/Sophus
RUN cd Sophus && mkdir build && cd build && cmake .. -D BUILD_SOPHUS_TESTS=OFF -D BUILD_SOPHUS_EXAMPLES=OFF && make -j2 && make install && rm /opt/Sophus -r
RUN apt-get install -y --no-install-recommends python2 ca-certificates curl
RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py && python2.7 get-pip.py
RUN pip install numpy
