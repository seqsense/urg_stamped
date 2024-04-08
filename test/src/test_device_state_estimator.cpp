/*
 * Copyright 2024 The urg_stamped Authors
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

#include <ros/time.h>

#include <urg_stamped/device_state_estimator.h>

namespace urg_stamped
{
namespace device_state_estimator
{

TEST(DeviceStateEstimator, FindMinDelay)
{
  Estimator est;

  est.startSync();
  est.push(ros::Time(100.0001), ros::Time(100.00015), 100);
  est.push(ros::Time(101.0002), ros::Time(101.00023), 1100);
  est.push(ros::Time(102.0003), ros::Time(102.00031), 2100);  // Minimal delay
  est.push(ros::Time(103.0004), ros::Time(103.00046), 3100);
  est.push(ros::Time(104.0005), ros::Time(104.00054), 4100);
  est.push(ros::Time(105.0006), ros::Time(105.00062), 5100);  // Second minimal delay

  {
    SCOPED_TRACE("Without OriginFracPart");
    const auto it = est.findMinDelay(OriginFracPart());
    ASSERT_EQ(ros::Time(102.0003), it->t_req_);
    ASSERT_EQ(ros::Time(102.00031), it->t_res_);
    ASSERT_EQ(2100, it->device_wall_stamp_);
  }

  {
    SCOPED_TRACE("With OriginFracPart");
    const auto it = est.findMinDelay(OriginFracPart(0.00029, 0.00031));
    ASSERT_EQ(ros::Time(105.0006), it->t_req_);
    ASSERT_EQ(ros::Time(105.00062), it->t_res_);
    ASSERT_EQ(5100, it->device_wall_stamp_);
  }

  est.finishSync();
  est.startSync();

  {
    SCOPED_TRACE("No samples");
    ASSERT_EQ(est.samples_.end(), est.findMinDelay(OriginFracPart()))
        << "must return end iterator if no samples are pushed";
  }
}

TEST(DeviceStateEstimator, RawClockOrigin)
{
  Estimator est;
  for (double d = 0; d < 0.003; d += 0.00025)
  {
    SCOPED_TRACE("Offset " + std::to_string(d));
    est.startSync();
    for (double t = 0; t < 0.1; t += 0.0101)
    {
      const uint64_t ts = (t + d) * 1000;
      est.push(ros::Time(1 + t), ros::Time(1 + t + 0.0001), ts);
    }
    est.finishSync();
    ASSERT_NEAR(est.state_.raw_clock_origin_.toSec(), 1.000 - d, 0.0002);
  }
}

}  // namespace device_state_estimator
}  // namespace urg_stamped

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}