/*
 * Copyright 2025 The urg_stamped Authors
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
#include <cmath>
#include <cstdint>
#include <vector>

#include <ros/time.h>

#include <urg_stamped/device_state_estimator.h>
#include <scip2/logger.h>

namespace urg_stamped
{
namespace device_state_estimator
{

void ClockEstimatorRaw::startSync()
{
  sync_samples_.clear();
  cnt_dropped_samples_ = 0;
}

void ClockEstimatorRaw::pushSyncSample(
    const ros::Time& t_req, const ros::Time& t_res, const uint64_t device_wall_stamp)
{
  const SyncSampleRaw s(t_req, t_res, device_wall_stamp);
  if (s.delay_ > ros::Duration(ACCEPTABLE_SAMPLE_DELAY))
  {
    cnt_dropped_samples_++;
    return;
  }
  sync_samples_.push_back(s);
  if (comm_delay_.min_.isZero() || comm_delay_.min_ > s.delay_)
  {
    comm_delay_.min_ = s.delay_;
  }
}

bool ClockEstimatorRaw::hasEnoughSyncSamples() const
{
  const size_t n = sync_samples_.size();
  if (cnt_dropped_samples_ >= MAX_DROPPED_SAMPLES)
  {
    return true;
  }
  return n >= MIN_SYNC_SAMPLES;
}

bool ClockEstimatorRaw::finishSync()
{
  if (cnt_dropped_samples_ >= MAX_DROPPED_SAMPLES)
  {
    scip2::logger::error()
        << "Communication delay is too large. Maybe unsupported sensor model"
        << std::endl;
    return false;
  }
  const auto min_delay = findMinDelay();
  if (min_delay == sync_samples_.cend())
  {
    scip2::logger::info() << "Failed to find minimal delay sample" << std::endl;
    return false;
  }
  comm_delay_.sigma_ = delaySigma();

  scip2::logger::debug()
      << "delay: " << min_delay->delay_
      << ", delay sigma: " << comm_delay_.sigma_
      << ", device timestamp: " << min_delay->device_wall_stamp_
      << std::endl;

  const ClockSample clock =
      {
          .t_estim_ = min_delay->t_process_,
          .stamp_ = min_delay->device_wall_stamp_,
          .origin_ = min_delay->t_origin_,
      };
  return pushClockSample(clock);
}

std::vector<ClockEstimatorRaw::SyncSampleRaw>::const_iterator
ClockEstimatorRaw::findMinDelay() const
{
  if (sync_samples_.size() == 0)
  {
    return sync_samples_.cend();
  }
  auto it_min_delay = sync_samples_.cbegin();
  for (auto it = sync_samples_.cbegin() + 1; it != sync_samples_.cend(); it++)
  {
    if (it->delay_ < it_min_delay->delay_)
    {
      it_min_delay = it;
    }
  }
  return it_min_delay;
}

ros::Duration ClockEstimatorRaw::delaySigma() const
{
  if (sync_samples_.size() == 0)
  {
    return ros::Duration();
  }
  double sum = 0;
  for (const auto& s : sync_samples_)
  {
    const double delay_diff = (s.delay_ - comm_delay_.min_).toSec();
    sum += delay_diff * delay_diff;
  }
  return ros::Duration(std::sqrt(sum / sync_samples_.size()));
}

}  // namespace device_state_estimator
}  // namespace urg_stamped

