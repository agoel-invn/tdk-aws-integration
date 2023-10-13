FROM sumedhrk0/tdk_robotics_ros:roscon2023 AS build-base

RUN apt install -y python-setuptools

RUN mkdir -p /aws_cloud/src
WORKDIR /aws_cloud
COPY ./src /aws_cloud/src

RUN cd /aws_cloud/src

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --install-base /opt/aws_cloud

WORKDIR /
COPY app_entrypoint.sh /app_entrypoint.sh
RUN chmod +x /app_entrypoint.sh
ENTRYPOINT ["/app_entrypoint.sh"]