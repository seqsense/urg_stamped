/*
 * Copyright 2018-2021 The urg_stamped Authors
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

#include <urg_stamped/timestamp_moving_average.h>

namespace urg_stamped
{
TEST(TimestampMovingAverage, ResetAndGo)
{
  TimestampMovingAverage ma(3, ros::Duration(0.1));
  for (double t = 10.0; t < 11.0; t += 0.1)
  {
    ASSERT_EQ(ma.update(ros::Time(t)), ros::Time(t));
  }
  ma.reset();
  for (double t = 12.0; t < 13.0; t += 0.1)
  {
    ASSERT_EQ(ma.update(ros::Time(t)), ros::Time(t));
  }
}

TEST(TimestampMovingAverage, MovingAverage)
{
  TimestampMovingAverage ma(3, ros::Duration(0.1));
  ASSERT_EQ(ma.update(ros::Time(10.000)), ros::Time(10.000));
  ASSERT_EQ(ma.update(ros::Time(10.100)), ros::Time(10.100));
  ASSERT_EQ(ma.update(ros::Time(10.203)), ros::Time(10.201));
  ASSERT_EQ(ma.update(ros::Time(10.300)), ros::Time(10.301));
  ASSERT_EQ(ma.update(ros::Time(10.400)), ros::Time(10.401));
  ASSERT_EQ(ma.update(ros::Time(10.500)), ros::Time(10.500));
}

TEST(TimestampMovingAverage, SkippedInput)
{
  TimestampMovingAverage ma(3, ros::Duration(0.1));
  for (double t = 10.0; t < 11.0; t += 0.1)
  {
    ASSERT_EQ(ma.update(ros::Time(t)), ros::Time(t));
  }
  // skip 0.5 sec.
  for (double t = 11.5; t < 12.0; t += 0.1)
  {
    ASSERT_EQ(ma.update(ros::Time(t)), ros::Time(t));
  }
}
}  // namespace urg_stamped

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
