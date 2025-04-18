ARG ROS_DISTRO=humble
FROM registry.gitlab.com/hitt-testing/docker/rviz-docker:${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

RUN apt update && \
    apt install -y python-is-python3 python3-pip && \
    pip install kiss-slam
