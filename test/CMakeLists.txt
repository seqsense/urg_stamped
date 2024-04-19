catkin_add_gtest(test_decode src/test_decode.cpp)
target_link_libraries(test_decode ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_device_time_origin src/test_device_time_origin.cpp)
target_link_libraries(test_device_time_origin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_first_order_filter src/test_first_order_filter.cpp)
catkin_add_gtest(test_param src/test_param.cpp)

catkin_add_gtest(test_timestamp_moving_average src/test_timestamp_moving_average.cpp)
target_link_libraries(test_timestamp_moving_average ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_timestamp_outlier_remover src/test_timestamp_outlier_remover.cpp)
target_link_libraries(test_timestamp_outlier_remover ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_scip2
  src/test_scip2.cpp
  ../src/scip2/logger.cpp
)
target_link_libraries(test_scip2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_walltime src/test_walltime.cpp ../src/scip2/logger.cpp)
target_link_libraries(test_walltime ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_rostest_gtest(e2e tests/e2e.test
  src/e2e.cpp
  ../src/urg_sim/encode.cpp
  ../src/urg_sim/urg_sim.cpp
)
target_link_libraries(e2e ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(e2e urg_stamped)
