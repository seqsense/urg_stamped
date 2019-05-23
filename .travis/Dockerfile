ARG ROS_DISTRO=kinetic
FROM seqsense/ros-baremetal:${ROS_DISTRO}-ros-baremetal

RUN rosdep update

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
RUN cd /catkin_ws/src \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && catkin_init_workspace
RUN pip install gh-pr-comment

WORKDIR /catkin_ws
