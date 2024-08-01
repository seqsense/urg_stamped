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

std::pair<ros::Time, bool> EstimatorUTM::pushScanSample(const ros::Time& t_recv, const uint64_t device_wall_stamp)
{
  const ros::Time t_stamp = clock_.stampToTime(device_wall_stamp);
  if (!clock_.initialized_)
  {
    return std::pair<ros::Time, bool>(t_stamp, true);
  }

  const auto t_scan = estimateScanTime(t_recv, t_stamp);
  if (t_scan.second)
  {
    recent_t_scans_.emplace_back(t_scan.first);
    if (recent_t_scans_.size() < SCAN_SAMPLES)
    {
      return std::pair<ros::Time, bool>(t_scan.first, true);
    }
    recent_t_scans_.pop_front();

    std::vector<ScanSampleUTM> samples;
    for (size_t i = 1; i < recent_t_scans_.size(); ++i)
    {
      for (size_t j = 0; j < i; ++j)
      {
        const ros::Duration t_diff = recent_t_scans_[i] - recent_t_scans_[j];
        const int num_ideal_scans =
            std::max(1, static_cast<int>(std::lround(t_diff.toSec() / ideal_scan_interval_.toSec())));
        const ros::Duration interval = t_diff * (1.0 / num_ideal_scans);
        samples.emplace_back(recent_t_scans_[i], interval);
      }
    }

    std::sort(samples.begin(), samples.end());
    const ScanSampleUTM& med = samples[samples.size() / 2];

    scan_.interval_ = med.interval_;
    scan_.origin_ = med.t_;
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

std::pair<ros::Time, bool> EstimatorUTM::estimateScanTime(const ros::Time& t_recv, const ros::Time& t_stamp)
{
  if (!clock_.initialized_)
  {
    return std::pair<ros::Time, bool>(t_stamp, false);
  }

  const ros::Time t_sent = t_recv - comm_delay_.min_;
  const ros::Duration stamp_to_send_raw = t_sent - t_stamp;

  stamp_to_sends_.push_back(stamp_to_send_raw);
  if (stamp_to_sends_.size() > STAMP_TO_SEND_SAMPLES)
  {
    stamp_to_sends_.pop_front();
  }

  ros::Duration stamp_to_send = stamp_to_send_raw;
  for (const auto& s : stamp_to_sends_)
  {
    if (s < stamp_to_send)
    {
      stamp_to_send = s;
    }
  }

  if (min_stamp_to_send_.isZero() || stamp_to_send < min_stamp_to_send_)
  {
    min_stamp_to_send_ = stamp_to_send;
  }
  else if (stamp_to_send > min_stamp_to_send_ + ros::Duration(DEVICE_TIMESTAMP_RESOLUTION))
  {
    min_stamp_to_send_ = stamp_to_send - ros::Duration(DEVICE_TIMESTAMP_RESOLUTION);
  }

  const ros::Duration t_frac = stamp_to_send_raw - min_stamp_to_send_ - comm_delay_.sigma_;

  return std::pair<ros::Time, bool>(
      t_stamp + t_frac,
      ros::Duration(0) < t_frac && t_frac < ros::Duration(DEVICE_TIMESTAMP_RESOLUTION));
}

}  // namespace device_state_estimator
}  // namespace urg_stamped
