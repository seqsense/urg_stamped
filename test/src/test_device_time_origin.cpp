/*
 * Copyright 2020 The urg_stamped Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <string>

#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <device_time_origin.h>

TEST(DeviceTimeOriginTest, testEstimateOriginByAverage)
{
  const boost::posix_time::time_duration delay = boost::posix_time::milliseconds(1);
  const boost::posix_time::ptime time_req = boost::posix_time::microsec_clock::universal_time();
  const boost::posix_time::ptime time_res = time_req + delay;
  const uint64_t device_timestamp = 12345 * 1e3;  // msec

  const ros::Time actual = device_time_origin::estimator::estimateOriginByAverage(
      time_req, time_res, device_timestamp);
  const ros::Time expected = ros::Time::fromBoost(
      time_req - boost::posix_time::milliseconds(device_timestamp) + delay / 2);

  ASSERT_EQ(actual, expected);
}

TEST(DeviceTimeOriginTest, testEstimateOrigin)
{
  const boost::posix_time::time_duration delay = boost::posix_time::milliseconds(1);
  const boost::posix_time::ptime time_req = boost::posix_time::microsec_clock::universal_time();
  const boost::posix_time::ptime time_res = time_req + delay;
  const uint64_t device_timestamp = 12345 * 1e3;  // msec

  ros::Time time_at_device_timestamp;
  const ros::Time actual = device_time_origin::estimator::estimateOrigin(
      time_res, device_timestamp, ros::Duration(ros::Time::fromBoost(delay).toSec()), time_at_device_timestamp);
  const ros::Time expected = ros::Time::fromBoost(
      time_req - boost::posix_time::milliseconds(device_timestamp) + delay / 2);

  ASSERT_EQ(time_at_device_timestamp, ros::Time::fromBoost(time_req + delay / 2));
  ASSERT_EQ(actual, expected);
}

TEST(DeviceTimeOriginTest, testDetectTimeJump)
{
  const ros::Time last_origin = ros::Time(100000);

  ASSERT_FALSE(
      device_time_origin::jump_detector::detectTimeJump(last_origin, last_origin + ros::Duration(1.00), 1.0));
  ASSERT_TRUE(
      device_time_origin::jump_detector::detectTimeJump(last_origin, last_origin + ros::Duration(1.01), 1.0));

  const ros::Time last_origin_not_init = ros::Time(0);

  ASSERT_FALSE(
      device_time_origin::jump_detector::detectTimeJump(
          last_origin_not_init, last_origin_not_init + ros::Duration(1.01), 1.0));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
