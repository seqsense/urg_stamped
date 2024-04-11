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

#include <memory>
#include <vector>

#include <ros/time.h>

#include <urg_stamped/device_state_estimator.h>
#include <scip2/logger.h>

namespace urg_stamped
{
namespace device_state_estimator
{

ros::Time ClockState::stampToTime(const uint64_t stamp) const
{
  const double fromOrigin = (stamp_ + (int64_t)(stamp - stamp_) / clock_gain_) / 1000.0;
  return clock_origin_ + ros::Duration(fromOrigin);
}

Estimator::Estimator()
{
  state_.clock_gain_ = 1.0;
}

void Estimator::startSync()
{
  sync_samples_.clear();
}

void Estimator::pushSyncSample(const ros::Time& t_req, const ros::Time& t_res, const uint64_t device_wall_stamp)
{
  sync_samples_.emplace_back(t_req, t_res, device_wall_stamp);
}

bool Estimator::hasEnoughSyncSamples() const
{
  const size_t n = sync_samples_.size();
  if (n < MIN_SYNC_SAMPLES)
  {
    return false;
  }
  if (n >= MAX_SYNC_SAMPLES)
  {
    return true;
  }
  const OriginFracPart overflow_range = originFracOverflow();
  return overflow_range.t_max_ > overflow_range.t_min_;
}

void Estimator::finishSync()
{
  const OriginFracPart overflow_range = originFracOverflow();
  if (!overflow_range)
  {
    scip2::logger::warn()
        << "failed to find origin fractional part overflow: "
        << overflow_range.t_min_ << ", " << overflow_range.t_max_
        << ", samples=" << sync_samples_.size()
        << std::endl;
    return;
  }
  const auto min_delay = findMinDelay(overflow_range);
  if (min_delay == sync_samples_.cend())
  {
    scip2::logger::warn() << "failed to find minimal delay sample" << std::endl;
    return;
  }

  const ClockState last = state_;

  state_.clock_origin_ = overflow_range.compensate(min_delay->t_origin_);
  state_.stamp_ = min_delay->device_wall_stamp_;
  state_.t_estim_ = min_delay->t_process_;
  if (min_comm_delay_ > min_delay->delay_)
  {
    min_comm_delay_ = min_delay->delay_;
  }

  if (last.clock_origin_.isZero())
  {
    return;
  }

  const double t_diff = (state_.t_estim_ - last.t_estim_).toSec();
  const double origin_diff =
      (state_.clock_origin_ - last.clock_origin_).toSec();
  const double gain = (t_diff - origin_diff) / t_diff;
  state_.clock_gain_ = gain;

  scip2::logger::debug()
      << "origin: " << state_.clock_origin_
      << ", gain: " << gain
      << ", delay: " << min_delay->delay_
      << ", device timestamp: " << min_delay->device_wall_stamp_
      << std::endl;
}

std::vector<SyncSample>::const_iterator Estimator::findMinDelay(const OriginFracPart& overflow_range) const
{
  if (sync_samples_.size() == 0)
  {
    return sync_samples_.cend();
  }
  auto it_min_delay = sync_samples_.cbegin();
  for (auto it = sync_samples_.cbegin() + 1; it != sync_samples_.cend(); it++)
  {
    if (overflow_range.isOnOverflow(it->t_process_))
    {
      continue;
    }
    if (it->delay_ < it_min_delay->delay_)
    {
      it_min_delay = it;
    }
  }
  return it_min_delay;
}

OriginFracPart Estimator::originFracOverflow() const
{
  if (sync_samples_.size() == 0)
  {
    return OriginFracPart();
  }
  auto it_min_origin = sync_samples_.begin();
  auto it_max_origin = sync_samples_.begin();
  for (auto it = sync_samples_.begin() + 1; it != sync_samples_.end(); it++)
  {
    if (it->t_origin_ < it_min_origin->t_origin_)
    {
      it_min_origin = it;
    }
    if (it->t_origin_ > it_max_origin->t_origin_)
    {
      it_max_origin = it;
    }
  }
  double t_min = std::fmod(it_min_origin->t_process_.toSec(), 0.001);
  double t_max = std::fmod(it_max_origin->t_process_.toSec(), 0.001);
  if (t_min > t_max + 0.0005)
  {
    t_max += 0.001;
  }
  else if (t_max > t_min + 0.0005)
  {
    t_min += 0.001;
  }
  if (std::abs(t_max - t_min) > 0.00025)
  {
    return OriginFracPart(t_min, t_max, false);
  }
  return OriginFracPart(t_min, t_max);
}

void Estimator::pushScanSample(const ros::Time& t_recv, const uint64_t device_wall_stamp)
{
}

}  // namespace device_state_estimator
}  // namespace urg_stamped
