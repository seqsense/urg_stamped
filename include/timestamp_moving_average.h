/*
 * Copyright 2018 The urg_stamped Authors
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

#ifndef TIMESTAMP_MOVING_AVERAGE_H
#define TIMESTAMP_MOVING_AVERAGE_H

#include <ros/ros.h>

#include <cmath>
#include <vector>

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
      const ros::Duration interval)
    : window_size_(window_size)
    , interval_(interval)
    , pos_(0)
  {
    buffer_.resize(window_size);
  }
  void setInterval(const ros::Duration interval)
  {
    interval_ = interval;
  }
  ros::Time update(const ros::Time stamp)
  {
    buffer_[pos_ % window_size_] = stamp;
    pos_++;
    if (pos_ < window_size_)
      return stamp;

    ros::Duration sum(0);
    for (const auto &b : buffer_)
    {
      sum += b - stamp;
    }
    const ros::Time average = stamp + sum * (1.0 / window_size_);
    return average + interval_ * ((window_size_ - 1) / 2.0);
  }
  void reset()
  {
    pos_ = 0;
  }
};

#endif  // TIMESTAMP_MOVING_AVERAGE_H
