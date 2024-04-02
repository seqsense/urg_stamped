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

#ifndef URG_STAMPED_DEVICE_CLOCK_ESTIMATOR_H
#define URG_STAMPED_DEVICE_CLOCK_ESTIMATOR_H

#include <memory>
#include <vector>

#include <ros/time.h>

#include <urg_stamped/ros_logger.h>

namespace urg_stamped
{
namespace device_state_estimator
{

class State
{
public:
  using Ptr = std::shared_ptr<State>;

  ros::Time clock_origin_;
  double clock_gain_;
  ros::Duration min_comm_delay_;

  // Scan time (timestamp) should be scan_origin_ * n + scan_interval_
  ros::Time scan_origin_;
  ros::Duration scan_interval_;
};

class TMSample
{
public:
  ros::Time t_req_;
  ros::Time t_res_;
  uint64_t device_wall_stamp_;

  ros::Duration delay_;
  ros::Time t_process_;
  ros::Time t_origin_;

  inline TMSample(const ros::Time& t_req, const ros::Time& t_res, const uint64_t device_wall_stamp)
    : t_req_(t_req)
    , t_res_(t_res)
    , device_wall_stamp_(device_wall_stamp)
    , delay_((t_res - t_req) * 0.5)
    , t_process_(t_res_ - delay_)
    , t_origin_(t_process_ - ros::Duration(device_wall_stamp_ * 0.001))
  {
  }
};

class Estimator
{
public:
  State::Ptr state_;

  inline void startSync()
  {
    samples_.clear();
  }

  inline void push(const ros::Time& t_req, const ros::Time& t_res, const uint64_t device_wall_stamp)
  {
    samples_.emplace_back(t_req, t_res, device_wall_stamp);
  }

  inline void finishSync()
  {
    const auto min_delay = findMinDelay();
    if (min_delay == samples_.cend())
    {
      scip2::logger::warn() << "no TM samples" << std::endl;
      return;
    }

    /*
    for (const auto& s : samples_)
    {
      std::cout << std::fmod(s.t_process_.toSec(), 0.001) << " " << s.t_origin_ << std::endl;
    }
    */
    const ros::Duration smd = subMillisecondOrigin();
    double t = std::floor(min_delay->t_origin_.toSec() * 1000) / 1000;
    const double t_remainder = std::fmod(min_delay->t_origin_.toSec(), 0.001);
    if (t_remainder > smd.toSec())
    {
      t += 0.001;
    }
    scip2::logger::info()
        << "origin: " << ros::Time(t)
        << " " << min_delay->t_origin_
        << ", delay: " << min_delay->delay_
        << ", device timestamp: " << min_delay->device_wall_stamp_
        << std::endl;
  }

private:
  std::vector<TMSample> samples_;

  inline std::vector<TMSample>::const_iterator findMinDelay() const
  {
    if (samples_.size() == 0)
    {
      return samples_.cend();
    }
    auto min_it = samples_.cbegin();
    for (auto it = samples_.cbegin() + 1; it != samples_.cend(); it++)
    {
      if (it->delay_ < min_it->delay_)
      {
        min_it = it;
      }
    }
    return min_it;
  }

  inline ros::Duration subMillisecondOrigin() const
  {
    if (samples_.size() == 0)
    {
      return ros::Duration(0);
    }
    auto min_it = samples_.begin();
    auto max_it = samples_.begin();
    for (auto it = samples_.begin() + 1; it != samples_.end(); it++)
    {
      if (it->t_origin_ < min_it->t_origin_)
      {
        min_it = it;
      }
      if (it->t_origin_ > max_it->t_origin_)
      {
        max_it = it;
      }
    }
    const double t0 = std::fmod(min_it->t_process_.toSec(), 0.001);
    const double t1 = std::fmod(max_it->t_process_.toSec(), 0.001);
    double t_min = std::min(t0, t1);
    double t_max = std::max(t0, t1);
    if (t_max - t_min > 0.0005)
    {
      t_min = std::max(t0, t1);
      t_max = std::min(t0, t1) + 0.001;
    }
    scip2::logger::info()
        << "min: " << t_min
        << ", max: " << t_max
        << std::endl;
    return ros::Duration((t_min + t_max) / 2);
  }
};

}  // namespace device_state_estimator
}  // namespace urg_stamped

#endif  // URG_STAMPED_DEVICE_CLOCK_ESTIMATOR_H
