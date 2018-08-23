cmake_minimum_required(VERSION 2.8.3)
project(urg_stamped)

## Find catkin and any catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    roscpp

    sensor_msgs
)
find_package(Boost 1.53 REQUIRED system chrono thread)

if(Boost_VERSION LESS 106000)
  message(STATUS "Using old boost version without namespaced placeholders")
  add_definitions(-DUSE_OLD_BOOST_PLACEHOLDERS=1)
endif()

catkin_package(
  CATKIN_DEPENDS
    roscpp

    sensor_msgs
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)

add_executable(urg_stamped src/urg_stamped.cpp)
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
