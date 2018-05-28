/*
 * Copyright 2018 The urg_stamped Authors
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

#include <scip2/walltime.h>

TEST(WalltimeTest, testTimestampOverflow)
{
  for (uint64_t start_device_time = 0;
       start_device_time < (1 << 24);
       start_device_time += (1 << 23))
  {
    scip2::Walltime<24> walltime;

    for (uint64_t device_time = start_device_time;
         device_time < (1 << 25);
         device_time += 25)
    {
      const uint32_t device_timestamp = device_time & 0xFFFFFF;
      ASSERT_EQ(walltime.update(device_timestamp), device_time);
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
