/*
 * Copyright 2018-2021 The urg_stamped Authors
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

#ifndef URG_STAMPED_TIMESTAMP_MOVING_AVERAGE_H
#define URG_STAMPED_TIMESTAMP_MOVING_AVERAGE_H

#include <ros/ros.h>

#include <cmath>
#include <vector>

namespace urg_stamped
{
class TimestampMovingAverage
{
protected:
  size_t window_size_;
  ros::Duration interval_;
  std::vector<ros::Time> buffer_;
  size_t pos_;

public:
  TimestampMovingAverage(
      const size_t window_size,
      const ros::Duration& interval)
    : window_size_(window_size)
    , interval_(interval)
    , pos_(0)
  {
    buffer_.resize(window_size);
  }
  void setInterval(const ros::Duration& interval)
  {
    interval_ = interval;
  }
  ros::Time update(const ros::Time& stamp)
  {
    buffer_[pos_ % window_size_] = stamp;
    pos_++;
    if (pos_ < window_size_)
      return stamp;

    ros::Duration sum(0);
    for (const ros::Time& b : buffer_)
    {
      sum += ros::Duration(remainder((b - stamp).toSec(), interval_.toSec()));
    }
    return stamp + sum * (1.0 / window_size_);
  }
  void reset()
  {
    pos_ = 0;
  }
};
}  // namespace urg_stamped

#endif  // URG_STAMPED_TIMESTAMP_MOVING_AVERAGE_H
