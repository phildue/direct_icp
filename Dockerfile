FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ADD . /opt/direct_icp
WORKDIR /opt/
RUN bash -c /opt/direct_icp/install_dependencies.sh
WORKDIR /opt/direct_icp
RUN bash -c /opt/direct_icp/build.sh
