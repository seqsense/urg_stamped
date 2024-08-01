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

#ifndef URG_STAMPED_URG_STAMPED_H
#define URG_STAMPED_URG_STAMPED_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/bind/bind.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <algorithm>
#include <cstdint>
#include <list>
#include <map>
#include <random>
#include <string>
#include <vector>

#include <scip2/scip2.h>
#include <scip2/walltime.h>

#include <urg_stamped/device_state_estimator.h>
#include <urg_stamped/ros_logger.h>

namespace urg_stamped
{
class DeviceOriginAt
{
public:
  ros::Time origin_;
  ros::Time at_;

  inline DeviceOriginAt(const ros::Time origin, const ros::Time at)
    : origin_(origin)
    , at_(at)
  {
  }

  inline bool operator<(const DeviceOriginAt& b) const
  {
    return origin_ < b.origin_;
  }
};

class UrgStampedNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_scan_;
  ros::Publisher pub_status_;
  ros::Timer timer_delay_estim_;
  ros::Timer timer_retry_tm_;

  sensor_msgs::LaserScan msg_base_;
  uint32_t step_min_;
  uint32_t step_max_;
  ros::Duration ideal_scan_interval_;

  scip2::Connection::Ptr device_;
  scip2::Protocol::Ptr scip_;

  bool publish_intensity_;
  bool failed_;

  enum class DelayEstimState
  {
    IDLE,
    STOPPING_SCAN,
    STATE_CHECKING,
    ESTIMATION_STARTING,
    ESTIMATING,
    EXITING,
  };
  DelayEstimState delay_estim_state_;
  boost::posix_time::ptime time_tm_request;
  ros::Time tm_start_time_;

  scip2::Walltime<24> walltime_;

  std::default_random_engine random_engine_;
  ros::Time last_sync_time_;

  struct ResponseErrorCount
  {
    inline ResponseErrorCount()
      : abnormal_error(0)
      , error(0)
    {
    }
    int abnormal_error;
    int error;
  };
  ResponseErrorCount error_count_;
  bool tm_success_;
  int error_count_max_;
  int scan_drop_count_;
  int scan_drop_continuous_;
  int fallback_on_continuous_scan_drop_;

  ros::Duration tm_command_interval_;
  std::string last_measurement_state_;
  int tm_try_max_;
  int tm_try_count_;

  bool cmd_resetting_;

  device_state_estimator::Estimator::Ptr est_;

  void cbM(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status,
      const scip2::ScanData& scan,
      const bool has_intensity);
  void cbTMSend(const boost::posix_time::ptime& time_send);
  void cbTM(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status,
      const scip2::Timestamp& time_device);
  void cbPP(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status,
      const std::map<std::string, std::string>& params);
  void cbVV(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status,
      const std::map<std::string, std::string>& params);
  void cbIISend(const boost::posix_time::ptime& time_send);
  void cbII(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status,
      const std::map<std::string, std::string>& params);
  void cbQT(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status);
  void cbRB(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status);
  void cbRS(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status);
  void cbConnect();
  void cbClose();

  void sendII();
  void timeSync(const ros::TimerEvent& event = ros::TimerEvent());
  void estimateSensorClock(const ros::TimerEvent& event = ros::TimerEvent());
  void retryTM(const ros::TimerEvent& event = ros::TimerEvent());
  void publishStatus();

  void errorCountIncrement(const std::string& status = "");

  void softReset();
  void hardReset();
  void sleepRandom(const double min, const double max);

public:
  UrgStampedNode();
  void spin();
};
}  // namespace urg_stamped

#endif  // URG_STAMPED_URG_STAMPED_H
