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

#ifndef TIMESTAMP_OUTLIER_REMOVER_H
#define TIMESTAMP_OUTLIER_REMOVER_H

#include <ros/ros.h>

#include <cmath>

class TimestampOutlierRemover
{
protected:
  ros::Time stamp_;
  ros::Duration diff_max_;
  ros::Duration interval_;
  size_t outlier_cnt_;

public:
  TimestampOutlierRemover(
      const ros::Duration &diff_max,
      const ros::Duration &interval)
    : diff_max_(diff_max)
    , interval_(interval)
    , outlier_cnt_(0)
  {
  }
  void setInterval(const ros::Duration &interval)
  {
    interval_ = interval;
  }
  ros::Time update(const ros::Time &stamp)
  {
    if (stamp_ == ros::Time())
      stamp_ = stamp - interval_;

    const auto interval = stamp - stamp_;
    const int scan_num = lround(interval.toSec() / interval_.toSec());
    const double interval_remainder = remainder(interval.toSec(), interval_.toSec());

    if (fabs(interval_remainder) > diff_max_.toSec())
    {
      stamp_ += interval_ * scan_num;
      if (outlier_cnt_ >= 1)
        stamp_ = stamp;

      outlier_cnt_++;
    }
    else
    {
      stamp_ = stamp;
      outlier_cnt_ = 0;
    }

    return stamp_;
  }
  void reset()
  {
    stamp_ = ros::Time();
  }
};

#endif  // TIMESTAMP_OUTLIER_REMOVER_H
