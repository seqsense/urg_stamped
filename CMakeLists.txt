cmake_minimum_required(VERSION 3.0.2)
project(urg_stamped)

set(CATKIN_DEPENDS
  roscpp
  sensor_msgs
  std_msgs
)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDS}
  message_generation
)

add_message_files(FILES Status.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
  ${CATKIN_DEPENDS}
  message_runtime
)

find_package(Boost 1.53 REQUIRED system chrono thread)

add_compile_options(-std=c++11)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)

add_executable(urg_stamped
  src/urg_stamped.cpp
  src/urg_stamped_node.cpp
  src/scip2/logger.cpp
  src/ros_logger.cpp
)
target_link_libraries(urg_stamped ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(urg_stamped ${urg_stamped_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(urg_sim
  src/urg_sim/encode.cpp
  src/urg_sim/urg_sim.cpp
  src/urg_sim/urg_sim_exec.cpp
)
target_link_libraries(urg_sim ${catkin_LIBRARIES} ${Boost_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(rosunit REQUIRED)
  find_package(roslint REQUIRED)
  set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-build/c++11")
  roslint_cpp()
  roslint_add_test()

  add_subdirectory(test)
endif()

install(TARGETS
    urg_stamped
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
