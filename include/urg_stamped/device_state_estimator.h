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

#ifndef URG_STAMPED_DEVICE_CLOCK_ESTIMATOR_H
#define URG_STAMPED_DEVICE_CLOCK_ESTIMATOR_H

#include <deque>
#include <memory>
#include <vector>

#include <ros/time.h>

#include "gtest/gtest_prod.h"

namespace urg_stamped
{
namespace device_state_estimator
{

class ClockState
{
public:
  ros::Time t_estim_;
  uint64_t stamp_;
  ros::Time origin_;
  double gain_;

  bool initialized_;

  ros::Time stampToTime(const uint64_t stamp) const;
};

class SyncSample
{
public:
  ros::Time t_req_;
  ros::Time t_res_;
  uint64_t device_wall_stamp_;

  ros::Duration delay_;
  ros::Time t_process_;
  ros::Time t_origin_;

  inline SyncSample(const ros::Time& t_req, const ros::Time& t_res, const uint64_t device_wall_stamp)
    : t_req_(t_req)
    , t_res_(t_res)
    , device_wall_stamp_(device_wall_stamp)
    , delay_((t_res - t_req) * 0.5)
    , t_process_(t_res_ - delay_)
    , t_origin_(t_process_ - ros::Duration(device_wall_stamp_ * 0.001))
  {
  }
};
class OriginFracPart
{
public:
  const bool valid_;
  const double t_min_;
  const double t_max_;
  static constexpr double TOLERANCE = 1e-4;

  inline OriginFracPart()
    : valid_(false)
    , t_min_(-1)
    , t_max_(-1)
  {
  }

  inline OriginFracPart(const double t_min, const double t_max, const bool valid = true)
    : valid_(valid)
    , t_min_(t_min)
    , t_max_(t_max)
  {
  }

  inline operator bool() const
  {
    return valid_;
  }

  inline bool isOnOverflow(const ros::Time& t) const
  {
    const double r = std::fmod(t.toSec(), 0.001);
    double t0 = std::min(t_min_, t_max_);
    double t1 = std::max(t_min_, t_max_);
    return t0 - TOLERANCE < r && r < t1 + TOLERANCE;
  }

  inline ros::Time compensate(const ros::Time& t) const
  {
    const double frac = (t_min_ + t_max_) / 2;
    double t_integral = std::floor(t.toSec() * 1000) / 1000;
    if (std::fmod(t.toSec(), 0.001) < frac)
    {
      t_integral -= 0.001;
    }
    return ros::Time(t_integral + frac);
  }
};

class ScanSample
{
public:
  ros::Time t_;
  ros::Duration interval_;

  inline ScanSample(const ros::Time& t, const ros::Duration& interval)
    : t_(t)
    , interval_(interval)
  {
  }

  inline bool operator<(const ScanSample& b) const
  {
    return this->interval_ < b.interval_;
  }
};

class ScanState
{
public:
  ros::Time origin_;
  ros::Duration interval_;

  inline ScanState()
  {
  }

  ScanState(const std::vector<ScanSample>& samples);
  ros::Time fit(const ros::Time& t) const;
};

class Estimator
{
public:
  static constexpr int MIN_SYNC_SAMPLES = 10;
  static constexpr int MAX_SYNC_SAMPLES = 100;
  static constexpr int MIN_SCAN_SAMPLES = 5;
  static constexpr int MAX_SCAN_SAMPLES = 9;

  ClockState clock_;
  ros::Duration min_comm_delay_;
  ros::Duration min_stamp_to_send_;
  ScanState scan_;
  std::deque<ros::Time> recent_t_scans_;

  Estimator();
  void startSync();
  void pushSyncSample(const ros::Time& t_req, const ros::Time& t_res, const uint64_t device_wall_stamp);
  bool hasEnoughSyncSamples() const;
  void finishSync();

  ros::Time pushScanSample(const ros::Time& t_recv, const uint64_t device_wall_stamp);

private:
  std::vector<SyncSample> sync_samples_;

  std::vector<SyncSample>::const_iterator findMinDelay(const OriginFracPart& overflow_range) const;
  OriginFracPart originFracOverflow() const;
  ros::Time pushScanSampleRaw(const ros::Time& t_recv, const uint64_t device_wall_stamp);

  FRIEND_TEST(DeviceStateEstimator, FindMinDelay);
  FRIEND_TEST(DeviceStateEstimator, PushScanSampleRaw);
};

}  // namespace device_state_estimator
}  // namespace urg_stamped

#endif  // URG_STAMPED_DEVICE_CLOCK_ESTIMATOR_H
