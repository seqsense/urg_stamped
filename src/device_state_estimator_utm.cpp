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

std::pair<ros::Time, bool> EstimatorUTM::pushScanSample(const ros::Time& t_recv, const uint64_t device_wall_stamp)
{
  const std::pair<ros::Time, bool> t_scan_raw = pushScanSampleRaw(t_recv, device_wall_stamp);
  if (!t_scan_raw.second)
  {
    return t_scan_raw;
  }
  if (!clock_.initialized_)
  {
    return t_scan_raw;
  }

  recent_t_scans_.emplace_back(t_scan_raw.first);
  if (recent_t_scans_.size() < MIN_SCAN_SAMPLES)
  {
    return t_scan_raw;
  }
  if (recent_t_scans_.size() >= MAX_SCAN_SAMPLES)
  {
    recent_t_scans_.pop_front();
  }

  std::vector<ScanSampleUTM> samples;
  for (size_t i = 1; i < recent_t_scans_.size(); ++i)
  {
    const ros::Duration interval = recent_t_scans_[i] - recent_t_scans_[i - 1];
    samples.emplace_back(recent_t_scans_[i], interval);
  }

  std::sort(samples.begin(), samples.end());
  const ScanSampleUTM& med = samples[samples.size() / 2];
  scan_.origin_ = med.t_;
  scan_.interval_ = med.interval_;

  return std::pair<ros::Time, bool>(scan_.fit(t_scan_raw.first), true);
}

std::pair<ros::Time, bool> EstimatorUTM::pushScanSampleRaw(const ros::Time& t_recv, const uint64_t device_wall_stamp)
{
  const ros::Time t_stamp = clock_.stampToTime(device_wall_stamp);
  if (!clock_.initialized_)
  {
    return std::pair<ros::Time, bool>(t_stamp, true);
  }

  const ros::Time t_sent = t_recv - comm_delay_.min_;
  const ros::Duration stamp_to_send = t_sent - t_stamp;
  ros::Duration new_min_stamp_to_send = min_stamp_to_send_;
  if (new_min_stamp_to_send.isZero() || stamp_to_send < new_min_stamp_to_send)
  {
    new_min_stamp_to_send = stamp_to_send;
  }
  if (min_stamp_to_send_.isZero())
  {
    min_stamp_to_send_ = new_min_stamp_to_send;
  }
  if (stamp_to_send - new_min_stamp_to_send < ros::Duration(0.001) && new_min_stamp_to_send < min_stamp_to_send_)
  {
    min_stamp_to_send_ =
        min_stamp_to_send_ * (1 - MIN_STAMP_TO_SEND_ALPHA) +
        new_min_stamp_to_send * MIN_STAMP_TO_SEND_ALPHA;
  }
  if (stamp_to_send - min_stamp_to_send_ > ros::Duration(0.001) + comm_delay_.sigma_ * 2 &&
      stamp_to_send - min_stamp_to_send_ < ros::Duration(0.002))
  {
    min_stamp_to_send_ =
        min_stamp_to_send_ * (1 - MIN_STAMP_TO_SEND_ALPHA) +
        (stamp_to_send - ros::Duration(0.001)) * MIN_STAMP_TO_SEND_ALPHA;
  }

  const ros::Duration t_frac = stamp_to_send - min_stamp_to_send_ - comm_delay_.sigma_;
  const ros::Time t_scan_raw = t_stamp + t_frac;
  const bool valid = t_frac < ros::Duration(0.0015);

  return std::pair<ros::Time, bool>(t_scan_raw, valid);
}

}  // namespace device_state_estimator
}  // namespace urg_stamped
