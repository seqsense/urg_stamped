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

#include <urg_stamped/timestamp_outlier_remover.h>

namespace urg_stamped
{
TEST(TimestampOutlierRemoval, RemoveOneOutlier)
{
  TimestampOutlierRemover remover(ros::Duration(0.01), ros::Duration(0.1));

  ASSERT_EQ(remover.update(ros::Time(10.000)), ros::Time(10.000));
  ASSERT_EQ(remover.update(ros::Time(10.105)), ros::Time(10.105));
  ASSERT_EQ(remover.update(ros::Time(10.200)), ros::Time(10.200));
  ASSERT_EQ(remover.update(ros::Time(10.320)), ros::Time(10.300));  // outlier
  ASSERT_EQ(remover.update(ros::Time(10.400)), ros::Time(10.400));
  ASSERT_EQ(remover.update(ros::Time(10.500)), ros::Time(10.500));
}

TEST(TimestampOutlierRemoval, MoreThanTwoOutlier)
{
  TimestampOutlierRemover remover(ros::Duration(0.01), ros::Duration(0.1));

  ASSERT_EQ(remover.update(ros::Time(10.000)), ros::Time(10.000));
  ASSERT_EQ(remover.update(ros::Time(10.105)), ros::Time(10.105));
  ASSERT_EQ(remover.update(ros::Time(10.200)), ros::Time(10.200));
  ASSERT_EQ(remover.update(ros::Time(10.320)), ros::Time(10.300));  // outlier
  ASSERT_EQ(remover.update(ros::Time(10.440)), ros::Time(10.440));  // fix only first outlier
}
}  // namespace urg_stamped

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
