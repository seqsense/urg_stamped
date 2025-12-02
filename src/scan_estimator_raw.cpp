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
#include <utility>

#include <ros/time.h>

#include <urg_stamped/device_state_estimator.h>

namespace urg_stamped
{
namespace device_state_estimator
{

std::pair<ros::Time, bool> ScanEstimatorRaw::pushScanSample(const ros::Time& t_recv, const uint64_t device_wall_stamp)
{
  const ClockState clock = clock_estim_->getClockState();
  const ros::Time t_stamp = clock.stampToTime(device_wall_stamp);

  scan_.origin_ = t_stamp;
  if (!last_stamp_.isZero())
  {
    const ros::Duration stamp_diff = t_stamp - last_stamp_;
    if (std::abs((stamp_diff - ideal_scan_interval_).toSec()) <
        ideal_scan_interval_.toSec() * INTERVAL_UPDATE_TOLERANCE)
    {
      scan_.interval_ = stamp_diff;
    }
  }

  last_stamp_ = t_stamp;

  return std::pair<ros::Time, bool>(t_stamp, true);
}

}  // namespace device_state_estimator
}  // namespace urg_stamped
