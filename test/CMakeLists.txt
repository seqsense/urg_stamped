include_directories(include)

catkin_add_gtest(test_device_state_estimator
  src/test_device_state_estimator.cpp
  ../src/device_state_estimator.cpp
  ../src/scan_estimator_utm.cpp
  ../src/scan_estimator_ust.cpp
  ../src/clock_estimator_uust1.cpp
  ../src/clock_estimator_uust2.cpp
  ../src/scip2/logger.cpp
)
target_link_libraries(test_device_state_estimator ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_decode src/test_decode.cpp)
target_link_libraries(test_decode ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(test_strings src/test_strings.cpp)

catkin_add_gtest(test_param src/test_param.cpp)

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

add_rostest_gtest(sync_timing tests/sync_timing.test
  src/sync_timing.cpp
  ../src/urg_sim/encode.cpp
  ../src/urg_sim/urg_sim.cpp
)
target_link_libraries(sync_timing ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(sync_timing urg_stamped)
