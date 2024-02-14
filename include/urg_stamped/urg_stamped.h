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
#include <list>
#include <map>
#include <random>
#include <string>
#include <vector>

#include <scip2/scip2.h>
#include <scip2/walltime.h>

#include <urg_stamped/device_time_origin.h>
#include <urg_stamped/first_order_filter.h>
#include <urg_stamped/timestamp_moving_average.h>
#include <urg_stamped/timestamp_outlier_remover.h>
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
  ros::Timer timer_sync_;
  ros::Timer timer_delay_estim_;
  ros::Timer timer_retry_tm_;

  sensor_msgs::LaserScan msg_base_;
  uint32_t step_min_;
  uint32_t step_max_;

  scip2::Connection::Ptr device_;
  scip2::Protocol::Ptr scip_;

  bool publish_intensity_;
  bool failed_;
  bool disable_on_scan_sync_;

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
  std::list<ros::Duration> communication_delays_;
  std::list<DeviceOriginAt> device_time_origins_;
  ros::Duration estimated_communication_delay_;
  size_t tm_iter_num_;
  size_t tm_median_window_;
  bool estimated_communication_delay_init_;
  bool device_time_origin_init_;
  double communication_delay_filter_alpha_;
  ros::Time tm_start_time_;

  boost::posix_time::ptime time_ii_request;
  std::vector<ros::Duration> on_scan_communication_delays_;

  device_time_origin::DriftedTime device_time_origin_;
  double allowed_device_time_origin_diff_;

  scip2::Walltime<24> walltime_;

  std::default_random_engine random_engine_;
  std::uniform_real_distribution<double> sync_interval_;
  ros::Time last_sync_time_;

  ros::Time t0_;
  FirstOrderLPF<double> timestamp_lpf_;
  FirstOrderHPF<double> timestamp_hpf_;
  TimestampOutlierRemover timestamp_outlier_removal_;
  TimestampMovingAverage timestamp_moving_average_;

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

  ros::Duration tm_command_interval_;
  std::string last_measurement_state_;
  int tm_try_max_;
  int tm_try_count_;

  bool cmd_resetting_;

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
  void delayEstimation(const ros::TimerEvent& event = ros::TimerEvent());
  void retryTM(const ros::TimerEvent& event = ros::TimerEvent());
  void updateOrigin(const ros::Time& now, const ros::Time& origin, const ros::Time& time_at_device_timestamp);

  void errorCountIncrement(const std::string& status = "");

  bool detectDeviceTimeJump(
      const boost::posix_time::ptime& time_response,
      const uint64_t& device_timestamp);

  void softReset();
  void hardReset();
  void sleepRandom(const double min, const double max);

public:
  UrgStampedNode();
  void spin();
};
}  // namespace urg_stamped

#endif  // URG_STAMPED_URG_STAMPED_H
