cmake_minimum_required(VERSION 3.0.2)
project(urg_stamped)

set(CATKIN_DEPENDS
  roscpp
  sensor_msgs
)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
find_package(Boost 1.53 REQUIRED system chrono thread)

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDS}
)

add_compile_options(-std=c++11)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)

add_executable(urg_stamped src/urg_stamped.cpp src/scip2/logger.cpp src/ros_logger.cpp)
target_link_libraries(urg_stamped ${catkin_LIBRARIES} ${Boost_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(rosunit REQUIRED)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()

  catkin_add_gtest(test_decode test/src/test_decode.cpp)
  target_link_libraries(test_decode ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  catkin_add_gtest(test_first_order_filter test/src/test_first_order_filter.cpp)

  catkin_add_gtest(test_timestamp_moving_average test/src/test_timestamp_moving_average.cpp)
  target_link_libraries(test_timestamp_moving_average ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  catkin_add_gtest(test_timestamp_outlier_remover test/src/test_timestamp_outlier_remover.cpp)
  target_link_libraries(test_timestamp_outlier_remover ${catkin_LIBRARIES} ${Boost_LIBRARIES})

  catkin_add_gtest(test_walltime test/src/test_walltime.cpp)
  target_link_libraries(test_walltime ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()

install(TARGETS
    urg_stamped
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
