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
#include <memory>
#include <utility>
#include <vector>

#include <fstream>

#include <ros/time.h>

#include <urg_stamped/device_state_estimator.h>
#include <scip2/logger.h>

namespace urg_stamped
{
namespace device_state_estimator
{

static std::ofstream file_scan("/ws/src/urg_stamped/scan.dat");

std::pair<ros::Time, bool> EstimatorUST::pushScanSample(const ros::Time& t_recv, const uint64_t device_wall_stamp)
{
  const ros::Time t_stamp = clock_.stampToTime(device_wall_stamp);
  if (!clock_.initialized_)
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
    auto it_prev = it;
    int num_scans = 0;
    for (it++; it != scans_.end(); it++)
    {
      if (it->interval_ != primary_interval_)
      {
        it_change1 = it_prev;
        break;
      }
      it_prev = it;
      num_scans++;
    }
    if (it_change0 != scans_.end() && it_change1 != scans_.end())
    {
      const int64_t stamp_diff = it_change0->stamp_ - it_change1->stamp_;
      scan_.origin_ = clock_.stampToTime(it_change0->stamp_);
      scan_.interval_ = ros::Duration(stamp_diff * 0.001 / (clock_.gain_ * num_scans));

      std::cerr
          << "scan origin: " << scan_.origin_
          << " interval: " << scan_.interval_
          << " latest stamp: " << device_wall_stamp
          << std::endl;
    }
  }
  else
  {
    scans_.clear();
  }

  if (scan_.origin_.isZero())
  {
    return std::pair<ros::Time, bool>(t_stamp, true);
  }

  const ros::Time t_estimated = scan_.fit(t_stamp);

  file_scan
      << device_wall_stamp
      << " " << stamps_[stamps_.size() - 2]
      << " " << t_recv
      << " " << t_stamp
      << " " << interval
      << " " << primary_interval_
      << " " << t_estimated
      << std::endl;
  return std::pair<ros::Time, bool>(t_estimated, true);
}

}  // namespace device_state_estimator
}  // namespace urg_stamped
