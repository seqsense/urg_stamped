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
#include <cmath>
#include <cstdint>

#include <ros/time.h>

#include <urg_stamped/device_state_estimator.h>

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

}  // namespace device_state_estimator
}  // namespace urg_stamped
