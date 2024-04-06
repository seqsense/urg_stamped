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

void Estimator::startSync()
{
  samples_.clear();
}

void Estimator::push(const ros::Time& t_req, const ros::Time& t_res, const uint64_t device_wall_stamp)
{
  samples_.emplace_back(t_req, t_res, device_wall_stamp);
}

void Estimator::finishSync()
{
  const OriginFracPart overflow_range = originFracOverflow();
  if (!overflow_range)
  {
    scip2::logger::warn()
        << "failed to find origin fractional part overflow: "
        << overflow_range.t0_ << ", " << overflow_range.t1_
        << std::endl;
    return;
  }
  const auto min_delay = findMinDelay(overflow_range);
  if (min_delay == samples_.cend())
  {
    scip2::logger::warn() << "failed to find minimal delay sample" << std::endl;
    return;
  }

  /*
  for (const auto& s : samples_)
  {
    std::cout << std::fmod(s.t_process_.toSec(), 0.001) << " " << s.t_origin_ << std::endl;
  }
  */
  const ros::Time t_origin = overflow_range.compensate(min_delay->t_origin_);
  scip2::logger::info()
      << "origin: " << t_origin
      << ", delay: " << min_delay->delay_
      << ", device timestamp: " << min_delay->device_wall_stamp_
      << std::endl;
  state_.raw_clock_origin_ = t_origin;
}

std::vector<TMSample>::const_iterator Estimator::findMinDelay(const OriginFracPart& overflow_range) const
{
  if (samples_.size() == 0)
  {
    return samples_.cend();
  }
  auto it_min_delay = samples_.cbegin();
  for (auto it = samples_.cbegin() + 1; it != samples_.cend(); it++)
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
  if (samples_.size() == 0)
  {
    return OriginFracPart();
  }
  auto it_min_origin = samples_.begin();
  auto it_max_origin = samples_.begin();
  for (auto it = samples_.begin() + 1; it != samples_.end(); it++)
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
  const double t0 = std::fmod(it_min_origin->t_process_.toSec(), 0.001);
  const double t1 = std::fmod(it_max_origin->t_process_.toSec(), 0.001);
  double t_min = std::min(t0, t1);
  double t_max = std::max(t0, t1);
  if (t_max - t_min > 0.0005)
  {
    t_min = std::max(t0, t1);
    t_max = std::min(t0, t1) + 0.001;
  }
  const double diff = t_max - t_min;
  if (diff > 0.00025)
  {
    return OriginFracPart(t_min, t_max, false);
  }
  return OriginFracPart(t_min, t_max);
}

}  // namespace device_state_estimator
}  // namespace urg_stamped
