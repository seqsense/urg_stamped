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

#ifndef URG_STAMPED_DEVICE_STATE_ESTIMATOR_H
#define URG_STAMPED_DEVICE_STATE_ESTIMATOR_H

#include <algorithm>
#include <deque>
#include <memory>
#include <utility>
#include <vector>

#include <ros/time.h>

#include "gtest/gtest_prod.h"

namespace urg_stamped
{
namespace device_state_estimator
{

class CommDelay
{
public:
  ros::Duration min_;
  ros::Duration sigma_;
};

class ClockState
{
public:
  ros::Time t_estim_;
  uint64_t stamp_;
  ros::Time origin_;
  double gain_;

  bool initialized_;

  inline ClockState()
    : stamp_(0)
    , gain_(1.0)
    , initialized_(false)
  {
  }

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

  bool isOnOverflow(const ros::Time& t) const;
  ros::Time compensate(const ros::Time& t) const;
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

  ros::Time fit(const ros::Time& t) const;
};

class Estimator
{
public:
  using Ptr = std::shared_ptr<Estimator>;

  void startSync();
  void pushSyncSample(
      const ros::Time& t_req,
      const ros::Time& t_res,
      const uint64_t device_wall_stamp);
  bool hasEnoughSyncSamples() const;
  void finishSync();

  inline ClockState getClockState() const
  {
    return clock_;
  }
  inline CommDelay getCommDelay() const
  {
    return comm_delay_;
  }
  inline ScanState getScanState() const
  {
    return scan_;
  }

  virtual std::pair<ros::Time, bool> pushScanSample(
      const ros::Time& t_recv, const uint64_t device_wall_stamp) = 0;

protected:
  ClockState clock_;
  CommDelay comm_delay_;
  ScanState scan_;

private:
  static constexpr int MIN_SYNC_SAMPLES = 10;
  static constexpr int MAX_SYNC_SAMPLES = 100;
  static constexpr double CLOCK_GAIN_ALPHA = 0.1;

  std::vector<SyncSample> sync_samples_;

  std::vector<SyncSample>::const_iterator findMinDelay(
      const OriginFracPart& overflow_range) const;
  OriginFracPart originFracOverflow() const;
  ros::Duration delaySigma() const;

  FRIEND_TEST(DeviceStateEstimatorUTM, FindMinDelay);
};

class EstimatorUTM : public Estimator
{
public:
  ros::Duration min_stamp_to_send_;
  std::deque<ros::Time> recent_t_scans_;

  std::pair<ros::Time, bool> pushScanSample(
      const ros::Time& t_recv,
      const uint64_t device_wall_stamp) final;

private:
  static constexpr int MIN_SCAN_SAMPLES = 5;
  static constexpr int MAX_SCAN_SAMPLES = 9;
  static constexpr double MIN_STAMP_TO_SEND_ALPHA = 0.05;

  std::pair<ros::Time, bool> pushScanSampleRaw(
      const ros::Time& t_recv, const uint64_t device_wall_stamp);

  FRIEND_TEST(DeviceStateEstimatorUTM, PushScanSampleRaw);
};

class EstimatorUST : public Estimator
{
public:
  std::pair<ros::Time, bool> pushScanSample(
      const ros::Time& t_recv,
      const uint64_t device_wall_stamp) final;
};

}  // namespace device_state_estimator
}  // namespace urg_stamped

#endif  // URG_STAMPED_DEVICE_STATE_ESTIMATOR_H
