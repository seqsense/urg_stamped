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
  const ros::Time t_scan_raw = pushScanSampleRaw(t_recv, t_stamp);
  if (!clock_.initialized_)
  {
    return std::pair<ros::Time, bool>(t_scan_raw, true);
  }

  recent_t_scans_.emplace_back(t_scan_raw);
  if (recent_t_scans_.size() < MIN_SCAN_SAMPLES)
  {
    return std::pair<ros::Time, bool>(t_scan_raw, true);
  }
  if (recent_t_scans_.size() >= MAX_SCAN_SAMPLES)
  {
    recent_t_scans_.pop_front();
  }

  std::vector<ScanSampleUTM> samples;
  for (size_t i = 1; i < recent_t_scans_.size(); ++i)
  {
    const int dist = i < STAMP_DIFF_DIST ? i : STAMP_DIFF_DIST;
    const ros::Duration t_diff = recent_t_scans_[i] - recent_t_scans_[i - dist];
    const int num_ideal_scans = std::lround(t_diff.toSec() / ideal_scan_interval_.toSec());
    const ros::Duration interval = t_diff * (1.0 / num_ideal_scans);
    samples.emplace_back(recent_t_scans_[i], interval);
  }

  std::sort(samples.begin(), samples.end());
  const ScanSampleUTM& med = samples[samples.size() / 2];
  scan_.origin_ = med.t_;
  scan_.interval_ = med.interval_;

  const ros::Time t_estimated = scan_.fit(t_scan_raw);
  const ros::Duration t_comp = t_estimated - t_stamp;
  const bool valid = ros::Duration(-0.001) < t_comp && t_comp < ros::Duration(0.001);

  return std::pair<ros::Time, bool>(t_estimated, valid);
}

ros::Time EstimatorUTM::pushScanSampleRaw(const ros::Time& t_recv, const ros::Time& t_stamp)
{
  if (!clock_.initialized_)
  {
    return t_stamp;
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
  return t_stamp + t_frac;
}

}  // namespace device_state_estimator
}  // namespace urg_stamped
