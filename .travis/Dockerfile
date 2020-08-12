ARG ROS_DISTRO=kinetic
FROM ros:${ROS_DISTRO}-ros-core

COPY ./ /catkin_ws/src/urg_stamped
RUN apt-get -qq update \
  && if [ "${ROS_DISTRO}" = "noetic" ]; then ROS_PYTHON_VERSION=3; fi \
  && apt-get install -y --no-install-recommends \
    build-essential \
    libgtest-dev \
    libxml2-utils \
    python${ROS_PYTHON_VERSION}-rosdep \
    sudo \
  && rosdep init \
  && rosdep update \
  && cd /catkin_ws \
  && rosdep install -y --from-paths src --ignore-src \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*
RUN cd /catkin_ws/src \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && catkin_init_workspace

COPY .travis/gh-pr-comment /usr/local/bin/

WORKDIR /catkin_ws
