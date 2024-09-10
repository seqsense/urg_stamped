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
  const double from_origin = (stamp_ + static_cast<int64_t>(stamp - stamp_) / gain_) * DEVICE_TIMESTAMP_RESOLUTION;
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
  const double r = std::fmod(t.toSec(), DEVICE_TIMESTAMP_RESOLUTION);
  const double t0 = std::min(t_min_, t_max_);
  const double t1 = std::max(t_min_, t_max_);
  return t0 - TOLERANCE < r && r < t1 + TOLERANCE;
}

ros::Time OriginFracPart::compensate(const ros::Time& t) const
{
  const double frac = (t_min_ + t_max_) / 2;
  double t_integral = std::floor(t.toSec() / DEVICE_TIMESTAMP_RESOLUTION) * DEVICE_TIMESTAMP_RESOLUTION;
  if (std::fmod(t.toSec(), DEVICE_TIMESTAMP_RESOLUTION) < frac)
  {
    t_integral -= DEVICE_TIMESTAMP_RESOLUTION;
  }
  return ros::Time(t_integral + frac);
}

void Estimator::startSync()
{
  sync_samples_.clear();
}

void Estimator::pushSyncSample(const ros::Time& t_req, const ros::Time& t_res, const uint64_t device_wall_stamp)
{
  const SyncSample s(t_req, t_res, device_wall_stamp);
  sync_samples_.push_back(s);
  if (comm_delay_.min_.isZero() || comm_delay_.min_ > s.delay_)
  {
    comm_delay_.min_ = s.delay_;
  }
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

bool Estimator::finishSync()
{
  const OriginFracPart overflow_range = originFracOverflow();
  if (!overflow_range)
  {
    scip2::logger::info()
        << "Failed to find origin fractional part overflow: "
        << overflow_range.t_min_ << ", " << overflow_range.t_max_
        << ", samples=" << sync_samples_.size()
        << std::endl;
    return false;
  }
  const auto min_delay = findMinDelay(overflow_range);
  if (min_delay == sync_samples_.cend())
  {
    scip2::logger::info() << "Failed to find minimal delay sample" << std::endl;
    return false;
  }
  comm_delay_.sigma_ = delaySigma();

  const ClockSample clock =
      {
          .t_estim_ = min_delay->t_process_,
          .stamp_ = min_delay->device_wall_stamp_,
          .origin_ = overflow_range.compensate(min_delay->t_origin_),
      };
  recent_clocks_.push_back(clock);
  if (recent_clocks_.size() >= CLOCK_SAMPLES)
  {
    recent_clocks_.pop_front();
  }

  if (recent_clocks_.size() <= 1)
  {
    // Initialized=false since clock gain is not yet estimated
    clock_ = ClockState(clock.origin_, 1, min_delay->device_wall_stamp_, false);

    scip2::logger::debug()
        << "initial origin: " << clock.origin_
        << ", delay: " << min_delay->delay_
        << ", delay sigma: " << comm_delay_.sigma_
        << ", device timestamp: " << min_delay->device_wall_stamp_
        << std::endl;
    return true;
  }

  std::vector<ClockState> clocks;
  for (size_t i = 1; i < recent_clocks_.size(); ++i)
  {
    for (size_t j = 0; j < i; ++j)
    {
      const double t_diff =
          (recent_clocks_[i].t_estim_ - recent_clocks_[j].t_estim_).toSec();
      const double origin_diff =
          (recent_clocks_[i].origin_ - recent_clocks_[j].origin_).toSec();
      clocks.emplace_back(
          recent_clocks_[i].origin_,
          (t_diff - origin_diff) / t_diff,
          recent_clocks_[i].stamp_);
    }
  }
  std::sort(clocks.begin(), clocks.end());
  clock_ = clocks[clocks.size() / 2];

  scip2::logger::debug()
      << "origin: " << clock_.origin_
      << ", gain: " << clock_.gain_
      << ", delay: " << min_delay->delay_
      << ", delay sigma: " << comm_delay_.sigma_
      << ", device timestamp: " << min_delay->device_wall_stamp_
      << std::endl;

  return true;
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

  const ros::Duration max_delay = comm_delay_.min_ + delaySigma();
  int valid_samples = 0;

  auto it_min_origin = sync_samples_.begin();
  auto it_max_origin = sync_samples_.begin();
  for (auto it = sync_samples_.begin() + 1; it != sync_samples_.end(); it++)
  {
    if (it->delay_ > max_delay)
    {
      continue;
    }
    valid_samples++;
    if (it->t_origin_ < it_min_origin->t_origin_)
    {
      it_min_origin = it;
    }
    if (it->t_origin_ > it_max_origin->t_origin_)
    {
      it_max_origin = it;
    }
  }

  if (valid_samples < MIN_SYNC_SAMPLES ||
      it_min_origin->delay_ > max_delay ||
      it_max_origin->delay_ > max_delay)
  {
    return OriginFracPart();
  }

  double t_min = std::fmod(it_min_origin->t_process_.toSec(), DEVICE_TIMESTAMP_RESOLUTION);
  double t_max = std::fmod(it_max_origin->t_process_.toSec(), DEVICE_TIMESTAMP_RESOLUTION);
  if (t_min > t_max + DEVICE_TIMESTAMP_RESOLUTION / 2)
  {
    t_max += DEVICE_TIMESTAMP_RESOLUTION;
  }
  else if (t_max > t_min + DEVICE_TIMESTAMP_RESOLUTION / 2)
  {
    t_min += DEVICE_TIMESTAMP_RESOLUTION;
  }
  if (std::abs(t_max - t_min) > DEVICE_TIMESTAMP_RESOLUTION / 4)
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
