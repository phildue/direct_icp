FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ADD install_dependencies.sh /opt/direct_icp/install_dependencies.sh
WORKDIR /opt/
RUN bash -c /opt/direct_icp/install_dependencies.sh

ADD src /opt/direct_icp/src
ADD CMakeLists.txt /opt/direct_icp/CMakeLists.txt
ADD build.sh /opt/direct_icp/build.sh
WORKDIR /opt/direct_icp
RUN bash -c /opt/direct_icp/build.sh


