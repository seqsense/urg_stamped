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

#include <vector>

#include <ros/time.h>

#include <urg_stamped/device_state_estimator.h>

namespace urg_stamped
{
namespace device_state_estimator
{

TEST(ClockState, StampToTime)
{
  {
    SCOPED_TRACE("ClockGain 1.5");
    ClockState s;

    s.stamp_ = 1000;
    s.origin_ = ros::Time(100);
    s.gain_ = 1.0 / 1.5;

    ASSERT_EQ(ros::Time(99.5), s.stampToTime(0));
    ASSERT_EQ(ros::Time(101.0), s.stampToTime(1000));
    ASSERT_EQ(ros::Time(102.5), s.stampToTime(2000));
  }
  {
    SCOPED_TRACE("ClockGain 1.0");
    ClockState s;

    s.stamp_ = 1000;
    s.origin_ = ros::Time(100);
    s.gain_ = 1.0;

    ASSERT_EQ(ros::Time(100.0), s.stampToTime(0));
    ASSERT_EQ(ros::Time(101.0), s.stampToTime(1000));
    ASSERT_EQ(ros::Time(102.0), s.stampToTime(2000));
  }
  {
    SCOPED_TRACE("ClockGain 0.5");
    ClockState s;

    s.stamp_ = 1000;
    s.origin_ = ros::Time(100);
    s.gain_ = 1.0 / 0.5;

    ASSERT_EQ(ros::Time(100.5), s.stampToTime(0));
    ASSERT_EQ(ros::Time(101.0), s.stampToTime(1000));
    ASSERT_EQ(ros::Time(101.5), s.stampToTime(2000));
  }
}

TEST(DeviceStateEstimator, FindMinDelay)
{
  EstimatorUTM est(ros::Duration(0.025));

  est.startSync();
  est.pushSyncSample(ros::Time(100.0001), ros::Time(100.00015), 100);
  est.pushSyncSample(ros::Time(101.0002), ros::Time(101.00023), 1100);
  est.pushSyncSample(ros::Time(102.0003), ros::Time(102.00031), 2100);  // Minimal delay
  est.pushSyncSample(ros::Time(103.0004), ros::Time(103.00046), 3100);
  est.pushSyncSample(ros::Time(104.0005), ros::Time(104.00054), 4100);
  est.pushSyncSample(ros::Time(105.0006), ros::Time(105.00062), 5100);  // Second minimal delay

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
    ASSERT_EQ(est.sync_samples_.end(), est.findMinDelay(OriginFracPart()))
        << "must return end iterator if no samples are pushSyncSampleed";
  }
}

TEST(DeviceStateEstimator, RawClockOrigin)
{
  EstimatorUTM est(ros::Duration(0.025));
  for (double d = 0; d < 0.003; d += 0.00025)
  {
    SCOPED_TRACE("Offset " + std::to_string(d));
    est.startSync();
    for (double t = 0; t < 0.11; t += 0.0101)
    {
      const uint64_t ts = (t + d) * 1000;
      est.pushSyncSample(ros::Time(1 + t), ros::Time(1 + t + 0.0001), ts);
    }
    est.finishSync();
    ASSERT_NEAR(est.recent_clocks_.back().origin_.toSec(), 1.000 - d, 0.0002);
  }
}

TEST(DeviceStateEstimator, ClockGain)
{
  const std::vector<double> gains =
      {
          0.999,
          1.000,
          1.001,
      };

  for (const double gain : gains)
  {
    EstimatorUTM est(ros::Duration(0.025));
    SCOPED_TRACE("Gain " + std::to_string(gain));

    for (double t0 = 0; t0 < 50; t0 += 10)
    {
      SCOPED_TRACE("T " + std::to_string(t0));
      est.startSync();
      for (double t = t0; t < t0 + 0.11; t += 0.0101)
      {
        const uint64_t ts = t * gain * 1000;
        est.pushSyncSample(ros::Time(1 + t), ros::Time(1 + t + 0.0001), ts);
      }
      est.finishSync();

      if (t0 > 0)
      {
        ASSERT_NEAR(est.getClockState().gain_, gain, 0.0001);
      }
    }
  }
}

TEST(DeviceStateEstimatorUTM, PushScanSampleRaw)
{
  EstimatorUTM est(ros::Duration(0.1));
  const double t0 = 100;

  est.startSync();
  for (double t = 10; t < 10.2; t += 0.0101)
  {
    est.pushSyncSample(ros::Time(t0 + t), ros::Time(t0 + t + 0.00001), t * 1000);
  }
  est.finishSync();
  est.startSync();
  for (double t = 20; t < 20.2; t += 0.0101)
  {
    est.pushSyncSample(ros::Time(t0 + t), ros::Time(t0 + t + 0.00001), t * 1000);
  }
  est.finishSync();

  est.estimateScanTime(
      ros::Time(t0 + 30.0200), est.clock_.stampToTime(30000));  // minimal scan stamp to send delay: 20ms
  est.estimateScanTime(
      ros::Time(t0 + 30.1206), est.clock_.stampToTime(30100));
  est.estimateScanTime(
      ros::Time(t0 + 30.2203), est.clock_.stampToTime(30200));

  ASSERT_NEAR(
      t0 + 31.0000,
      est.estimateScanTime(ros::Time(t0 + 31.0200), est.clock_.stampToTime(31000)).first.toSec(),
      0.0001);
  ASSERT_NEAR(
      t0 + 32.0005,
      est.estimateScanTime(ros::Time(t0 + 32.0205), est.clock_.stampToTime(32000)).first.toSec(),
      0.0001);
}

}  // namespace device_state_estimator
}  // namespace urg_stamped

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
