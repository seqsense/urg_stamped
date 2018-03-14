FROM ros:kinetic

RUN apt-get -qq update && \
  apt-get install -y --no-install-recommends sudo libgtest-dev python-pip && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/*

RUN rosdep update && \
  mkdir -p /catkin_ws/src && \
  bash -c "cd /catkin_ws/src && . /opt/ros/${ROS_DISTRO}/setup.bash && catkin_init_workspace && cd .. && catkin_make"

COPY ./ /catkin_ws/src/urg_stamped
RUN apt-get -qq update && \
  cd /catkin_ws && \
  rosdep install --from-paths src --ignore-src && \
  apt-get clean && \
  rm -rf /var/lib/apt/lists/*
