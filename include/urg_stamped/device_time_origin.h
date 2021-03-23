/*
 * Copyright 2020-2021 The urg_stamped Authors
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

#ifndef URG_STAMPED_DEVICE_TIME_ORIGIN_H
#define URG_STAMPED_DEVICE_TIME_ORIGIN_H

#include <ros/ros.h>

#include <cmath>

namespace urg_stamped
{
namespace device_time_origin
{
class DriftedTime
{
public:
  ros::Time origin_;
  double gain_;

  inline DriftedTime()
    : gain_(1.0)
  {
  }
  inline DriftedTime(const ros::Time origin, const float gain)
    : origin_(origin)
    , gain_(gain)
  {
  }
};

namespace estimator
{
inline ros::Time estimateOriginByAverage(
    const boost::posix_time::ptime& time_request,
    const boost::posix_time::ptime& time_response,
    const uint64_t& device_timestamp)
{
  const ros::Duration delay =
      ros::Time::fromBoost(time_response) -
      ros::Time::fromBoost(time_request);
  const ros::Time time_at_device_timestamp = ros::Time::fromBoost(time_request) + delay * 0.5;

  return time_at_device_timestamp - ros::Duration().fromNSec(device_timestamp * 1e6);
}
inline ros::Time estimateOrigin(
    const boost::posix_time::ptime& time_response,
    const uint64_t& device_timestamp,
    const ros::Duration& communication_delay,
    ros::Time& time_at_device_timestamp)
{
  time_at_device_timestamp = ros::Time::fromBoost(time_response) - communication_delay * 0.5;

  return time_at_device_timestamp - ros::Duration().fromNSec(device_timestamp * 1e6);
}
}  // namespace estimator

namespace jump_detector
{
inline bool detectTimeJump(
    const ros::Time& last_device_time_origin,
    const ros::Time& current_device_time_origin,
    const double allowed_device_time_origin_diff)
{
  if (last_device_time_origin == ros::Time(0))
    return false;

  const ros::Duration origin_diff = last_device_time_origin - current_device_time_origin;
  return std::abs(origin_diff.toSec()) > allowed_device_time_origin_diff;
}
}  // namespace jump_detector
}  // namespace device_time_origin
}  // namespace urg_stamped

#endif  // URG_STAMPED_DEVICE_TIME_ORIGIN_H
