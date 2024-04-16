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

#include <ros/time.h>

#include <urg_stamped/device_state_estimator.h>
#include <scip2/logger.h>

namespace urg_stamped
{
namespace device_state_estimator
{

ros::Time ClockState::stampToTime(const uint64_t stamp) const
{
  const double from_origin = (stamp_ + (int64_t)(stamp - stamp_) / gain_) / 1000.0;
  return origin_ + ros::Duration(from_origin);
}

ros::Time ScanState::fit(const ros::Time& t) const
{
  const double from_origin = (t - origin_).toSec();
  const double interval = interval_.toSec();
  const int n = std::lround(from_origin / interval);
  return origin_ + ros::Duration(interval * n);
}

bool OriginFracPart::isOnOverflow(const ros::Time& t) const
{
  const double r = std::fmod(t.toSec(), 0.001);
  double t0 = std::min(t_min_, t_max_);
  double t1 = std::max(t_min_, t_max_);
  return t0 - TOLERANCE < r && r < t1 + TOLERANCE;
}

ros::Time OriginFracPart::compensate(const ros::Time& t) const
{
  const double frac = (t_min_ + t_max_) / 2;
  double t_integral = std::floor(t.toSec() * 1000) / 1000;
  if (std::fmod(t.toSec(), 0.001) < frac)
  {
    t_integral -= 0.001;
  }
  return ros::Time(t_integral + frac);
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
  comm_delay_.sigma_ = delaySigma();

  const ClockState last = clock_;

  clock_.origin_ = overflow_range.compensate(min_delay->t_origin_);
  clock_.stamp_ = min_delay->device_wall_stamp_;
  clock_.t_estim_ = min_delay->t_process_;
  if (comm_delay_.min_.isZero() || comm_delay_.min_ > min_delay->delay_)
  {
    comm_delay_.min_ = min_delay->delay_;
  }

  if (last.origin_.isZero())
  {
    return;
  }

  const double t_diff = (clock_.t_estim_ - last.t_estim_).toSec();
  const double origin_diff =
      (clock_.origin_ - last.origin_).toSec();
  const double gain = (t_diff - origin_diff) / t_diff;

  if (!clock_.initialized_)
  {
    clock_.gain_ = gain;
    clock_.initialized_ = true;
  }
  else
  {
    clock_.gain_ =
        clock_.gain_ * (1 - CLOCK_GAIN_ALPHA) +
        gain * CLOCK_GAIN_ALPHA;
  }

  scip2::logger::debug()
      << "origin: " << clock_.origin_
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

ros::Duration Estimator::delaySigma() const
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
