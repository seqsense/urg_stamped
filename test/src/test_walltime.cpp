/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
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
