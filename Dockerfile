ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO} as base

ENV ROS_DISTRO=${ROS_DISTRO}

ARG NODE_NAME
ENV NODE_NAME=${NODE_NAME}

RUN if [ -z "$NODE_NAME" ]; then echo 'Environment variable NODE_NAME must be specified. Exiting.'; exit 1; fi

SHELL ["/bin/bash", "-c"]


# Create Colcon workspace and copy in all of the packages required for the current node
RUN mkdir -p /sailbot_ws/src
WORKDIR /sailbot_ws/src
RUN mkdir /sailbot_ws/src/${NODE_NAME}
RUN mkdir /sailbot_ws/src/sailbot_msgs
COPY ${NODE_NAME} /sailbot_ws/src/${NODE_NAME}
COPY sailbot_msgs /sailbot_ws/src/sailbot_msgs


# install python dependencies
RUN sudo apt-get update \
 && sudo apt install python3-pip -y

# this is necessary for open cv (TODO: Maybe find a way to only install these libraries if we are installing opencv in the image)
RUN apt-get update && apt-get install ffmpeg libsm6 libxext6  -y
RUN pip3 install setuptools==58.2.0
RUN pip3 install -r /sailbot_ws/src/${NODE_NAME}/requirements.txt


# Build the base Colcon workspace, installing dependencies first.
WORKDIR /sailbot_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
 && colcon build --symlink-install

# Start the ROS node through ros2 run
CMD source /opt/ros/${ROS_DISTRO}/setup.bash \ 
    && source install/setup.bash \
    && ros2 run ${NODE_NAME} launch_${NODE_NAME}_node