ARG ROS_DISTRO=kinetic
FROM seqsense/ros-baremetal:${ROS_DISTRO}-ros-baremetal

RUN rosdep update

RUN mkdir -p /catkin_ws/src \
  && catkin_init_workspace

COPY ./ /catkin_ws/src/urg_stamped
RUN apt-get -qq update \
  && apt-get install -y --no-install-recommends \
    libgtest-dev \
    libxml2-utils \
    python-pip \
    sudo \
  && cd /catkin_ws \
  && rosdep install -y --from-paths src --ignore-src \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*
RUN pip install gh-pr-comment

WORKDIR /catkin_ws
