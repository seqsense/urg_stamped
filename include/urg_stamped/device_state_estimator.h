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
#include <cstdint>
#include <deque>
#include <memory>
#include <utility>
#include <vector>

#include <string>
#include <fstream>

#include <ros/time.h>

#include "gtest/gtest_prod.h"

namespace urg_stamped
{
namespace device_state_estimator
{

static constexpr double DEVICE_TIMESTAMP_RESOLUTION = 0.001;

class CommDelay
{
public:
  ros::Duration min_;
  ros::Duration sigma_;
};

class ClockSample
{
public:
  ros::Time t_estim_;
  uint64_t stamp_;
  ros::Time origin_;
};

class ClockState
{
public:
  ros::Time origin_;
  double gain_;
  uint64_t stamp_;
  bool initialized_;

  inline ClockState()
    : gain_(1.0)
    , stamp_(0)
    , initialized_(false)
  {
  }

  inline ClockState(const ros::Time& origin, const double gain, const uint64_t stamp, const bool initialized = true)
    : origin_(origin)
    , gain_(gain)
    , stamp_(stamp)
    , initialized_(initialized)
  {
  }

  inline bool operator<(const ClockState& b) const
  {
    return this->gain_ < b.gain_;
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
    , t_origin_(t_process_ - ros::Duration(device_wall_stamp_ * DEVICE_TIMESTAMP_RESOLUTION))
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

class ScanSampleUTM
{
public:
  ros::Time t_;
  ros::Duration interval_;

  inline ScanSampleUTM(const ros::Time& t, const ros::Duration& interval)
    : t_(t)
    , interval_(interval)
  {
  }

  inline bool operator<(const ScanSampleUTM& b) const
  {
    return this->interval_ < b.interval_;
  }
};

class ScanSampleUST
{
public:
  uint64_t stamp_;
  int64_t interval_;

  inline ScanSampleUST(uint64_t stamp, int64_t interval)
    : stamp_(stamp)
    , interval_(interval)
  {
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

  inline explicit Estimator(const ros::Duration& ideal_scan_interval)
    : ideal_scan_interval_(ideal_scan_interval)
  {
  }

  void startSync();
  void pushSyncSample(
      const ros::Time& t_req,
      const ros::Time& t_res,
      const uint64_t device_wall_stamp);
  bool hasEnoughSyncSamples() const;
  bool finishSync();

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
  ros::Duration ideal_scan_interval_;

private:
  static constexpr int MIN_SYNC_SAMPLES = 10;
  static constexpr int MAX_SYNC_SAMPLES = 50;
  static constexpr int CLOCK_SAMPLES = 7;

  std::vector<SyncSample> sync_samples_;
  std::deque<ClockSample> recent_clocks_;

  std::vector<SyncSample>::const_iterator findMinDelay(
      const OriginFracPart& overflow_range) const;
  OriginFracPart originFracOverflow() const;
  ros::Duration delaySigma() const;

  FRIEND_TEST(DeviceStateEstimator, FindMinDelay);
  FRIEND_TEST(DeviceStateEstimator, RawClockOrigin);
  FRIEND_TEST(DeviceStateEstimator, ClockGain);
};

class DurationWithOffset
{
public:
  ros::Duration value_;
  ros::Duration offset_;

  inline DurationWithOffset(const ros::Duration& value, const ros::Duration& offset)
    : value_(value)
    , offset_(offset)
  {
  }

  inline bool operator<(const DurationWithOffset& b) const
  {
    return this->value_ + this->offset_ < b.value_ + b.offset_;
  }
};

class EstimatorUTM : public Estimator
{
public:
  inline explicit EstimatorUTM(const ros::Duration& ideal_scan_interval)
    : Estimator(ideal_scan_interval)
  {
  }

  std::pair<ros::Time, bool> pushScanSample(
      const ros::Time& t_recv,
      const uint64_t device_wall_stamp) final;

private:
  static constexpr size_t SCAN_SAMPLES = 16;
  static constexpr size_t STAMP_TO_SEND_SAMPLES = 32;

  ros::Duration min_stamp_to_send_;
  std::deque<ros::Time> recent_t_scans_;
  std::deque<ros::Duration> stamp_to_sends_;

  std::pair<ros::Time, bool> estimateScanTime(
      const ros::Time& t_recv, const ros::Time& t_stamp);

  FRIEND_TEST(DeviceStateEstimatorUTM, PushScanSampleRaw);
};

class EstimatorUST : public Estimator
{
public:
  inline explicit EstimatorUST(const ros::Duration& ideal_scan_interval)
    : Estimator(ideal_scan_interval)
    , primary_interval_(0)
  {
  }

  std::pair<ros::Time, bool> pushScanSample(
      const ros::Time& t_recv,
      const uint64_t device_wall_stamp) final;

private:
  static constexpr size_t STAMP_SAMPLES = 8;
  static constexpr size_t MAX_INTERVAL_SAMPLES = 2048;
  static constexpr size_t MIN_INTERVAL_SAMPLES = 10;
  std::deque<uint64_t> stamps_;
  std::deque<ScanSampleUST> scans_;
  int64_t primary_interval_;
};

}  // namespace device_state_estimator
}  // namespace urg_stamped

#endif  // URG_STAMPED_DEVICE_STATE_ESTIMATOR_H
