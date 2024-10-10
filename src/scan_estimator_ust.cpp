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

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include <ros/time.h>

#include <urg_stamped/device_state_estimator.h>
#include <scip2/logger.h>

namespace urg_stamped
{
namespace device_state_estimator
{

std::pair<ros::Time, bool> ScanEstimatorUST::pushScanSample(const ros::Time& t_recv, const uint64_t device_wall_stamp)
{
  const ClockState clock = clock_estim_->getClockState();
  const ros::Time t_stamp = clock.stampToTime(device_wall_stamp);
  if (!clock.initialized_)
  {
    return std::pair<ros::Time, bool>(t_stamp, true);
  }

  stamps_.push_back(device_wall_stamp);
  if (stamps_.size() < STAMP_SAMPLES)
  {
    return std::pair<ros::Time, bool>(t_stamp, true);
  }
  stamps_.pop_front();

  const int64_t interval = device_wall_stamp - stamps_[stamps_.size() - 2];

  std::vector<int64_t> intervals;
  for (size_t i = 1; i < stamps_.size(); ++i)
  {
    const int64_t interval = stamps_[i] - stamps_[i - 1];
    intervals.push_back(interval);
  }
  std::sort(intervals.begin(), intervals.end());
  primary_interval_ = intervals[intervals.size() / 2];

  const int64_t interval_diff = interval - primary_interval_;
  if (-1 <= interval_diff && interval_diff <= 1)
  {
    scans_.emplace_front(device_wall_stamp, interval);
    if (scans_.size() >= MAX_INTERVAL_SAMPLES)
    {
      scans_.pop_back();
    }

    // Find cycle of timestamp increment
    auto it = scans_.begin();
    auto it_change0 = scans_.end();
    auto it_change1 = scans_.end();
    for (; it != scans_.end(); it++)
    {
      if (it->interval_ != primary_interval_)
      {
        it_change0 = it;
        break;
      }
    }
    if (it_change0 != scans_.end())
    {
      auto it_prev = it;
      size_t num_samples = 0;
      for (it++; it != scans_.end(); it++)
      {
        num_samples++;
        if (it->interval_ != primary_interval_)
        {
          it_change1 = it_prev;
          if (num_samples >= MIN_INTERVAL_SAMPLES)
          {
            break;
          }
        }
        it_prev = it;
      }
    }
    if (it_change1 != scans_.end())
    {
      // Calculate scan interval
      if (it_change0 != scans_.end() && it_change1 != scans_.end())
      {
        const int64_t stamp_diff = it_change0->stamp_ - it_change1->stamp_;
        const double ideal_scan_interval_cnt =
            ideal_scan_interval_.toSec() * clock.gain_ / DEVICE_TIMESTAMP_RESOLUTION;
        const int num_scans = std::lround(static_cast<double>(stamp_diff) / ideal_scan_interval_cnt);
        const ros::Time new_origin = clock.stampToTime(it_change0->stamp_);
        if (new_origin != scan_.origin_)
        {
          scan_.origin_ = clock.stampToTime(it_change0->stamp_);
          scan_.interval_ = ros::Duration(stamp_diff * DEVICE_TIMESTAMP_RESOLUTION / (clock.gain_ * num_scans));
          scip2::logger::debug()
              << "scan_origin: " << scan_.origin_ << " interval: " << scan_.interval_ << std::endl;
        }
      }
    }
    else
    {
      if (scan_.origin_.isValid() && scan_.origin_ + ros::Duration(30) < t_recv)
      {
        scan_.origin_ = t_stamp;
        scan_.interval_ = ideal_scan_interval_ * (1.0 / clock.gain_);
        scip2::logger::debug()
            << "no-increment scan_origin: " << scan_.origin_ << " interval: " << scan_.interval_ << std::endl;
      }
    }
  }

  if (scan_.origin_.isZero())
  {
    return std::pair<ros::Time, bool>(t_stamp, true);
  }

  const ros::Time t_estimated = scan_.fit(t_stamp);
  const ros::Duration t_comp = t_estimated - t_stamp;
  const bool valid =
      ros::Duration(-DEVICE_TIMESTAMP_RESOLUTION) < t_comp &&
      t_comp < ros::Duration(DEVICE_TIMESTAMP_RESOLUTION);

  return std::pair<ros::Time, bool>(t_estimated, valid);
}

}  // namespace device_state_estimator
}  // namespace urg_stamped
