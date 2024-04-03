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

class OriginFracPart
{
public:
  const bool valid_;
  const double t0_;
  const double t1_;

  inline OriginFracPart()
    : valid_(false)
    , t0_(0)
    , t1_(0)
  {
  }

  inline OriginFracPart(const double t0, const double t1)
    : valid_(true)
    , t0_(t0)
    , t1_(t1)
  {
  }

  inline operator bool() const
  {
    return valid_;
  }

  inline bool isOnOverflow(const ros::Time& t) const
  {
    const double r = std::fmod(t.toSec(), 0.001);
    return t0_ < r && r < t1_;
  }

  inline ros::Time compensate(const ros::Time& t) const
  {
    const double frac = (t0_ + t1_) / 2;
    double t_integral = std::floor(t.toSec() * 1000) / 1000;
    if (std::fmod(t.toSec(), 0.001) > frac)
    {
      t_integral += 0.001;
    }
    return ros::Time(t_integral + frac);
  }
};

class Estimator
{
public:
  State::Ptr state_;

  void startSync();
  void push(const ros::Time& t_req, const ros::Time& t_res, const uint64_t device_wall_stamp);
  void finishSync();

private:
  std::vector<TMSample> samples_;

  std::vector<TMSample>::const_iterator findMinDelay(const OriginFracPart& overflow_range) const;
  OriginFracPart originFracOverflow() const;
};

}  // namespace device_state_estimator
}  // namespace urg_stamped

#endif  // URG_STAMPED_DEVICE_CLOCK_ESTIMATOR_H
