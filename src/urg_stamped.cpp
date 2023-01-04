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

#include <ros/console.h>
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
#include <scip2/logger.h>

#include <urg_stamped/device_time_origin.h>
#include <urg_stamped/first_order_filter.h>
#include <urg_stamped/timestamp_moving_average.h>
#include <urg_stamped/timestamp_outlier_remover.h>
#include <urg_stamped/ros_logger.h>

#include <urg_stamped/urg_stamped.h>

namespace urg_stamped
{
void UrgStampedNode::cbM(
    const boost::posix_time::ptime& time_read,
    const std::string& echo_back,
    const std::string& status,
    const scip2::ScanData& scan,
    const bool has_intensity)
{
  if (status != "99")
  {
    if (status != "00")
    {
      scip2::logger::error() << echo_back << " errored with " << status << std::endl;
      errorCountIncrement(status);
    }
    return;
  }

  const uint64_t walltime_device = walltime_.update(scan.timestamp_);
  if (detectDeviceTimeJump(time_read, walltime_device))
  {
    errorCountIncrement(status);
    return;
  }

  const auto estimated_timestamp_lf =
      device_time_origin_.origin_ +
      ros::Duration().fromNSec(walltime_device * 1e6 * device_time_origin_.gain_) +
      ros::Duration(msg_base_.time_increment * step_min_);

  if (t0_ == ros::Time(0))
    t0_ = estimated_timestamp_lf;

  const ros::Time time_read_ros = ros::Time::fromBoost(time_read);
  const auto receive_time =
      timestamp_outlier_removal_.update(
          time_read_ros -
          estimated_communication_delay_ * 0.5 -
          ros::Duration(msg_base_.scan_time));

  sensor_msgs::LaserScan msg(msg_base_);
  msg.header.stamp =
      timestamp_moving_average_.update(
          t0_ +
          ros::Duration(
              timestamp_lpf_.update((estimated_timestamp_lf - t0_).toSec()) +
              timestamp_hpf_.update((receive_time - t0_).toSec())));

  if (msg.header.stamp > time_read_ros)
  {
    scip2::logger::error()
        << std::setprecision(6) << std::fixed
        << "estimated future timestamp (read: " << time_read_ros.toSec()
        << ", estimated: " << msg.header.stamp.toSec() << std::endl;
    errorCountIncrement();
    return;
  }

  if (scan.ranges_.size() != step_max_ - step_min_ + 1)
  {
    scip2::logger::debug()
        << "Size of the received scan data is wrong "
           "(expected: "
        << step_max_ - step_min_ + 1
        << ", received: " << scan.ranges_.size() << "); refreshing" << std::endl;
    scip_->sendCommand(
        (has_intensity ? "ME" : "MD") +
        (boost::format("%04d%04d") % step_min_ % step_max_).str() +
        "00000");
    return;
  }

  msg.ranges.reserve(scan.ranges_.size());
  for (auto& r : scan.ranges_)
    msg.ranges.push_back(r * 1e-3);
  if (has_intensity)
  {
    msg.intensities.reserve(scan.intensities_.size());
    for (auto& r : scan.intensities_)
      msg.intensities.push_back(r);
  }

  pub_scan_.publish(msg);
  error_count_ = ResponseErrorCount();
}

void UrgStampedNode::cbTMSend(const boost::posix_time::ptime& time_send)
{
  time_tm_request = time_send;
}

void UrgStampedNode::cbTM(
    const boost::posix_time::ptime& time_read,
    const std::string& echo_back,
    const std::string& status,
    const scip2::Timestamp& time_device)
{
  if (status != "00")
  {
    scip2::logger::error() << echo_back << " errored with " << status << std::endl;
    errorCountIncrement(status);

    if (echo_back[2] == '0' && delay_estim_state_ == DelayEstimState::ESTIMATION_STARTING)
    {
      scip2::logger::info()
          << "Failed to enter the time synchronization mode, "
             "even after receiving successful QT command response. "
             "QT command may be ignored by the sensor firmware"
          << std::endl;
      delay_estim_state_ = DelayEstimState::STOPPING_SCAN;
    }
    return;
  }

  timer_retry_tm_.stop();
  switch (echo_back[2])
  {
    case '0':
    {
      scip2::logger::debug() << "Entered the time synchronization mode" << std::endl;
      delay_estim_state_ = DelayEstimState::ESTIMATING;
      scip_->sendCommand(
          "TM1",
          boost::bind(&UrgStampedNode::cbTMSend, this, boost::arg<1>()));
      break;
    }
    case '1':
    {
      const uint64_t walltime_device = walltime_.update(time_device.timestamp_);
      if (detectDeviceTimeJump(time_read, walltime_device))
      {
        errorCountIncrement(status);
        break;
      }

      const auto delay =
          ros::Time::fromBoost(time_read) -
          ros::Time::fromBoost(time_tm_request);
      communication_delays_.push_back(delay);
      if (communication_delays_.size() > tm_median_window_)
        communication_delays_.pop_front();

      const auto origin = device_time_origin::estimator::estimateOriginByAverage(
          time_tm_request, time_read, walltime_device);
      device_time_origins_.push_back(origin);
      if (device_time_origins_.size() > tm_median_window_)
        device_time_origins_.pop_front();

      if (communication_delays_.size() >= tm_iter_num_)
      {
        std::vector<ros::Duration> delays(communication_delays_.begin(), communication_delays_.end());
        std::vector<ros::Time> origins(device_time_origins_.begin(), device_time_origins_.end());
        sort(delays.begin(), delays.end());
        sort(origins.begin(), origins.end());

        if (!estimated_communication_delay_init_)
        {
          estimated_communication_delay_ = delays[tm_iter_num_ / 2];
          device_time_origin_ = device_time_origin::DriftedTime(origins[tm_iter_num_ / 2], 1.0);
        }
        else
        {
          estimated_communication_delay_ =
              estimated_communication_delay_ * (1.0 - communication_delay_filter_alpha_) +
              delays[tm_iter_num_ / 2] * communication_delay_filter_alpha_;
        }
        estimated_communication_delay_init_ = true;
        scip2::logger::debug()
            << "delay: "
            << std::setprecision(6) << std::fixed << estimated_communication_delay_.toSec()
            << ", device timestamp: " << walltime_device
            << ", device time origin: " << origins[tm_iter_num_ / 2].toSec()
            << std::endl;
        scip_->sendCommand("TM2");
      }
      else
      {
        ros::Duration(0.005).sleep();
        scip_->sendCommand(
            "TM1",
            boost::bind(&UrgStampedNode::cbTMSend, this, boost::arg<1>()));
      }
      break;
    }
    case '2':
    {
      delay_estim_state_ = DelayEstimState::IDLE;
      scip_->sendCommand(
          (publish_intensity_ ? "ME" : "MD") +
          (boost::format("%04d%04d") % step_min_ % step_max_).str() +
          "00000");
      timeSync();
      timestamp_outlier_removal_.reset();
      timestamp_moving_average_.reset();
      t0_ = ros::Time();
      scip2::logger::debug() << "Leaving the time synchronization mode" << std::endl;
      tm_success_ = true;
      break;
    }
  }
}

void UrgStampedNode::cbPP(
    const boost::posix_time::ptime& time_read,
    const std::string& echo_back,
    const std::string& status,
    const std::map<std::string, std::string>& params)
{
  if (status != "00")
  {
    scip2::logger::error() << echo_back << " errored with " << status << std::endl;
    errorCountIncrement(status);
    return;
  }

  const auto amin = params.find("AMIN");
  const auto amax = params.find("AMAX");
  const auto dmin = params.find("DMIN");
  const auto dmax = params.find("DMAX");
  const auto ares = params.find("ARES");
  const auto afrt = params.find("AFRT");
  const auto scan = params.find("SCAN");
  if (amin == params.end() || amax == params.end() ||
      dmin == params.end() || dmax == params.end() ||
      ares == params.end() || afrt == params.end() ||
      scan == params.end())
  {
    scip2::logger::error() << "PP doesn't have required parameters" << std::endl;
    return;
  }
  step_min_ = std::stoi(amin->second);
  step_max_ = std::stoi(amax->second);
  msg_base_.scan_time = 60.0 / std::stoi(scan->second);
  msg_base_.angle_increment = 2.0 * M_PI / std::stoi(ares->second);
  msg_base_.time_increment = msg_base_.scan_time / std::stoi(ares->second);
  msg_base_.range_min = std::stoi(dmin->second) * 1e-3;
  msg_base_.range_max = std::stoi(dmax->second) * 1e-3;
  msg_base_.angle_min =
      (std::stoi(amin->second) - std::stoi(afrt->second)) * msg_base_.angle_increment;
  msg_base_.angle_max =
      (std::stoi(amax->second) - std::stoi(afrt->second)) * msg_base_.angle_increment;

  timestamp_outlier_removal_.setInterval(ros::Duration(msg_base_.scan_time));
  timestamp_moving_average_.setInterval(ros::Duration(msg_base_.scan_time));
  delayEstimation();
}

void UrgStampedNode::cbVV(
    const boost::posix_time::ptime& time_read,
    const std::string& echo_back,
    const std::string& status,
    const std::map<std::string, std::string>& params)
{
  if (status != "00")
  {
    scip2::logger::error() << echo_back << " errored with " << status << std::endl;
    errorCountIncrement(status);
    return;
  }

  const char* keys[] =
      {
          "VEND",
          "PROD",
          "FIRM",
          "PROT",
          "SERI",
      };
  for (const char* key : keys)
  {
    const auto kv = params.find(key);
    if (kv == params.end())
    {
      scip2::logger::error() << "VV doesn't have key " << key << std::endl;
      continue;
    }
    scip2::logger::info() << key << ": " << kv->second << std::endl;
  }
}

void UrgStampedNode::cbIISend(const boost::posix_time::ptime& time_send)
{
  time_ii_request = time_send;
}

void UrgStampedNode::cbII(
    const boost::posix_time::ptime& time_read,
    const std::string& echo_back,
    const std::string& status,
    const std::map<std::string, std::string>& params)
{
  if (status != "00")
  {
    scip2::logger::error() << echo_back << " errored with " << status << std::endl;
    errorCountIncrement(status);
    return;
  }

  const auto stat = params.find("STAT");
  if (stat != params.end())
  {
    scip2::logger::debug() << "sensor status: " << stat->second << std::endl;
  }
  const auto mesm = params.find("MESM");
  if (mesm != params.end())
  {
    scip2::logger::debug() << "measurement status: " << mesm->second << std::endl;
  }

  if (delay_estim_state_ == DelayEstimState::STATE_CHECKING)
  {
    if (mesm == params.end())
    {
      scip2::logger::error() << "II doesn't have measurement state" << std::endl;
      errorCountIncrement();
    }
    else
    {
      // MESM (measurement state) value depends on the sensor model.
      //   UTM-30LX-EW: "Idle"
      //   UST-**LX: "Measuring"
      std::string state(mesm->second);
      const auto tolower = [](unsigned char c)
      {
        return std::tolower(c);
      };
      std::transform(state.begin(), state.end(), state.begin(), tolower);
      if (state.find("idle") != std::string::npos || state.find("measuring") != std::string::npos)
      {
        if (last_measurement_state_ != state && last_measurement_state_ != "")
        {
          scip2::logger::info() << "Sensor is idle, entering time synchronization mode" << std::endl;
        }
        delay_estim_state_ = DelayEstimState::ESTIMATION_STARTING;
        retryTM();
      }
      else
      {
        if (last_measurement_state_ != state)
        {
          scip2::logger::info() << "Sensor is not idle (" << state << ")" << std::endl;
        }
        delay_estim_state_ = DelayEstimState::STOPPING_SCAN;
      }
      last_measurement_state_ = state;
    }
    return;
  }

  const auto delay =
      ros::Time::fromBoost(time_read) -
      ros::Time::fromBoost(time_ii_request);

  if (delay.toSec() < 0.002)
  {
    const auto time = params.find("TIME");
    if (time == params.end())
    {
      scip2::logger::debug() << "II doesn't have timestamp" << std::endl;
      return;
    }
    if (time->second.size() != 6 && time->second.size() != 4)
    {
      scip2::logger::debug() << "Timestamp in II is ill-formatted (" << time->second << ")" << std::endl;
      return;
    }
    const uint32_t time_device =
        time->second.size() == 6 ?
            std::stoi(time->second, nullptr, 16) :
            *(scip2::Decoder<4>(time->second).begin());

    const uint64_t walltime_device = walltime_.update(time_device);
    if (detectDeviceTimeJump(time_read, walltime_device))
    {
      errorCountIncrement(status);
      return;
    }

    ros::Time time_at_device_timestamp;
    const auto origin = device_time_origin::estimator::estimateOrigin(
        time_read, walltime_device, estimated_communication_delay_, time_at_device_timestamp);

    const auto now = ros::Time::fromBoost(time_read);
    if (last_sync_time_ == ros::Time(0))
      last_sync_time_ = now;
    const double dt = std::min((now - last_sync_time_).toSec(), 10.0);
    last_sync_time_ = now;

    const double gain =
        (time_at_device_timestamp - device_time_origin_.origin_).toSec() /
        (time_at_device_timestamp - origin).toSec();
    const double exp_lpf_alpha =
        dt * (1.0 / 30.0);  // 30 seconds exponential LPF
    const double updated_gain =
        (1.0 - exp_lpf_alpha) * device_time_origin_.gain_ + exp_lpf_alpha * gain;
    device_time_origin_.gain_ = updated_gain;
    device_time_origin_.origin_ +=
        ros::Duration(exp_lpf_alpha * (origin - device_time_origin_.origin_).toSec());

    scip2::logger::debug()
        << "on scan delay: " << std::setprecision(6) << std::fixed << delay
        << ", device timestamp: " << walltime_device
        << ", device time origin: " << origin
        << ", gain: " << updated_gain << std::endl;
  }
  else
  {
    scip2::logger::debug()
        << "on scan delay (" << std::setprecision(6) << std::fixed << delay
        << ") is larger than expected; skipping" << std::endl;
  }
}

void UrgStampedNode::cbQT(
    const boost::posix_time::ptime& time_read,
    const std::string& echo_back,
    const std::string& status)
{
  if (status != "00")
  {
    scip2::logger::error() << echo_back << " errored with " << status << std::endl;
    errorCountIncrement(status);
    return;
  }

  scip2::logger::debug() << "Scan data stopped" << std::endl;

  if (delay_estim_state_ == DelayEstimState::STOPPING_SCAN)
  {
    delay_estim_state_ = DelayEstimState::STATE_CHECKING;
    retryTM();
  }
}

void UrgStampedNode::cbRB(
    const boost::posix_time::ptime& time_read,
    const std::string& echo_back,
    const std::string& status)
{
  if (status == "01")
  {
    scip2::logger::info() << "Sensor reboot in-progress" << std::endl;
    scip_->sendCommand("RB");  // Sending it 2 times in 1 sec. is needed
    return;
  }
  else if (status == "00")
  {
    scip2::logger::info() << "Sensor reboot succeeded" << std::endl;
    device_->stop();
    sleepRandom(1.0, 2.0);
    ros::shutdown();
    return;
  }
  scip2::logger::error()
      << echo_back << " errored with " << status << std::endl
      << "Failed to reboot. Please power-off the sensor." << std::endl;
}

void UrgStampedNode::cbRS(
    const boost::posix_time::ptime& time_read,
    const std::string& echo_back,
    const std::string& status)
{
  if (status != "00")
  {
    scip2::logger::error()
        << echo_back << " errored with " << status << std::endl
        << "Failed to reset. Rebooting the sensor and exiting." << std::endl;
    hardReset();
    return;
  }
  scip2::logger::info() << "Sensor reset succeeded" << std::endl;
  if (failed_)
  {
    scip2::logger::info() << "Restarting urg_stamped" << std::endl;
    device_->stop();
    sleepRandom(0.5, 1.0);
    ros::shutdown();
    return;
  }
  scip_->sendCommand("VV");
  scip_->sendCommand("PP");
  delay_estim_state_ = DelayEstimState::IDLE;
  tm_try_count_ = 0;
}

void UrgStampedNode::cbConnect()
{
  scip_->sendCommand("RS");
  device_->startWatchdog(boost::posix_time::seconds(1));
}

void UrgStampedNode::cbClose()
{
  scip2::logger::info()
      << "internal state on failure: "
      << "delay_estim_state_=" << static_cast<int>(delay_estim_state_) << " "
      << "tm_try_count_=" << tm_try_count_ << " "
      << "error_count_.error=" << error_count_.error << " "
      << "error_count_.abnormal_error=" << error_count_.abnormal_error << " "
      << std::endl;
  delay_estim_state_ = DelayEstimState::EXITING;
  device_->stop();
  ros::shutdown();
}

void UrgStampedNode::sendII()
{
  scip_->sendCommand(
      "II",
      boost::bind(&UrgStampedNode::cbIISend, this, boost::arg<1>()));
}

void UrgStampedNode::timeSync(const ros::TimerEvent& event)
{
  if (delay_estim_state_ == DelayEstimState::IDLE)
  {
    sendII();
  }
  timer_sync_ = nh_.createTimer(
      ros::Duration(sync_interval_(random_engine_)),
      &UrgStampedNode::timeSync, this, true);
}

void UrgStampedNode::delayEstimation(const ros::TimerEvent& event)
{
  tm_try_count_ = 0;
  timer_sync_.stop();  // Stop timer for sync using II command.
  scip2::logger::debug() << "Starting communication delay estimation" << std::endl;
  delay_estim_state_ = DelayEstimState::STOPPING_SCAN;
  timer_retry_tm_.stop();
  timer_retry_tm_ = nh_.createTimer(
      tm_command_interval_,
      &UrgStampedNode::retryTM, this);
  retryTM();
}

void UrgStampedNode::retryTM(const ros::TimerEvent& event)
{
  switch (delay_estim_state_)
  {
    case DelayEstimState::STOPPING_SCAN:
      scip2::logger::debug() << "Stopping scan" << std::endl;
      scip_->sendCommand("QT");
      tm_try_count_++;
      if (tm_try_count_ > tm_try_max_)
      {
        scip2::logger::error() << "Failed to enter time synchronization mode" << std::endl;
        errorCountIncrement();
        softReset();
      }
      break;
    case DelayEstimState::STATE_CHECKING:
      scip2::logger::debug() << "Checking sensor state" << std::endl;
      sendII();
      break;
    case DelayEstimState::ESTIMATION_STARTING:
      scip2::logger::debug() << "Entering the time synchronization mode" << std::endl;
      scip_->sendCommand("TM0");
      break;
    case DelayEstimState::ESTIMATING:
      scip2::logger::warn() << "Timeout occured during the time synchronization" << std::endl;
      scip_->sendCommand("TM2");
      break;
    default:
      break;
  }
}

void UrgStampedNode::errorCountIncrement(const std::string& status)
{
  if (delay_estim_state_ == DelayEstimState::EXITING)
  {
    // Already resetting or rebooting.
    return;
  }

  if (status == "00")
  {
    return;
  }

  if (status == "0L")
  {
    ++error_count_.abnormal_error;
    if (error_count_.abnormal_error > error_count_max_)
    {
      failed_ = true;
      delay_estim_state_ = DelayEstimState::EXITING;
      scip2::logger::error()
          << "Error count exceeded limit, rebooting the sensor and exiting."
          << std::endl;
      hardReset();
    }
  }
  else
  {
    ++error_count_.error;
    if (error_count_.error > error_count_max_)
    {
      failed_ = true;
      delay_estim_state_ = DelayEstimState::EXITING;
      if (tm_success_)
      {
        scip2::logger::error()
            << "Error count exceeded limit, resetting the sensor and exiting." << std::endl;
        softReset();
      }
      else
      {
        scip2::logger::error()
            << "Error count exceeded limit without successful time sync, "
               "rebooting the sensor and exiting."
            << std::endl;
        hardReset();
      }
    }
  }
}

void UrgStampedNode::softReset()
{
  scip_->sendCommand("RS");
}

void UrgStampedNode::hardReset()
{
  scip2::logger::error() << "Rebooting the sensor" << std::endl;
  scip_->sendCommand("RB");
}

void UrgStampedNode::sleepRandom(const double min, const double max)
{
  std::uniform_real_distribution<double> rnd(min, max);
  ros::Duration(rnd(random_engine_)).sleep();
}

bool UrgStampedNode::detectDeviceTimeJump(
    const boost::posix_time::ptime& time_response,
    const uint64_t& device_timestamp)
{
  ros::Time time_at_device_timestamp;
  const ros::Time current_origin =
      device_time_origin::estimator::estimateOrigin(
          time_response, device_timestamp, estimated_communication_delay_, time_at_device_timestamp);

  const bool jumped = device_time_origin::jump_detector::detectTimeJump(
      device_time_origin_.origin_, current_origin, allowed_device_time_origin_diff_);

  if (jumped)
  {
    scip2::logger::error()
        << "Device time origin jumped\n"
           "last origin: "
        << std::setprecision(3) << std::fixed << device_time_origin_.origin_.toSec()
        << ", current origin: " << current_origin.toSec()
        << ", allowed_device_time_origin_diff: " << allowed_device_time_origin_diff_
        << ", device_timestamp: " << device_timestamp << std::endl;
  }
  return jumped;
}

UrgStampedNode::UrgStampedNode()
  : nh_("")
  , pnh_("~")
  , failed_(false)
  , delay_estim_state_(DelayEstimState::IDLE)
  , tm_iter_num_(5)
  , tm_median_window_(35)
  , estimated_communication_delay_init_(false)
  , communication_delay_filter_alpha_(0.3)
  , last_sync_time_(0)
  , timestamp_lpf_(20)
  , timestamp_hpf_(20)
  , timestamp_outlier_removal_(ros::Duration(0.001), ros::Duration())
  , timestamp_moving_average_(5, ros::Duration())
  , tm_success_(false)
{
  std::random_device rd;
  random_engine_.seed(rd());

  std::string ip;
  int port;
  double sync_interval_min;
  double sync_interval_max;
  double delay_estim_interval;

  pnh_.param("ip_address", ip, std::string("192.168.0.10"));
  pnh_.param("ip_port", port, 10940);
  pnh_.param("frame_id", msg_base_.header.frame_id, std::string("laser"));
  pnh_.param("publish_intensity", publish_intensity_, true);
  pnh_.param("sync_interval_min", sync_interval_min, 1.0);
  pnh_.param("sync_interval_max", sync_interval_max, 1.5);
  sync_interval_ = std::uniform_real_distribution<double>(sync_interval_min, sync_interval_max);
  pnh_.param("delay_estim_interval", delay_estim_interval, 20.0);
  pnh_.param("error_limit", error_count_max_, 4);
  pnh_.param("allowed_device_time_origin_diff", allowed_device_time_origin_diff_, 1.0);

  double tm_interval, tm_timeout;
  pnh_.param("tm_interval", tm_interval, 0.06);
  tm_command_interval_ = ros::Duration(tm_interval);
  pnh_.param("tm_timeout", tm_timeout, 10.0);
  tm_try_max_ = static_cast<int>(tm_timeout / tm_interval);

  urg_stamped::setROSLogger(msg_base_.header.frame_id + ": ");

  bool debug;
  pnh_.param("debug", debug, false);
  if (debug)
  {
    // Enable debug level log at the beginning of the node to show initialization related logs.
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }
  }

  pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);

  device_.reset(new scip2::ConnectionTcp(ip, port));
  device_->registerCloseCallback(
      boost::bind(&UrgStampedNode::cbClose, this));
  device_->registerConnectCallback(
      boost::bind(&UrgStampedNode::cbConnect, this));

  scip_.reset(new scip2::Protocol(device_));
  scip_->registerCallback<scip2::ResponsePP>(
      boost::bind(&UrgStampedNode::cbPP, this,
                  boost::arg<1>(),
                  boost::arg<2>(),
                  boost::arg<3>(),
                  boost::arg<4>()));
  scip_->registerCallback<scip2::ResponseVV>(
      boost::bind(&UrgStampedNode::cbVV, this,
                  boost::arg<1>(),
                  boost::arg<2>(),
                  boost::arg<3>(),
                  boost::arg<4>()));
  scip_->registerCallback<scip2::ResponseII>(
      boost::bind(&UrgStampedNode::cbII, this,
                  boost::arg<1>(),
                  boost::arg<2>(),
                  boost::arg<3>(),
                  boost::arg<4>()));
  scip_->registerCallback<scip2::ResponseMD>(
      boost::bind(&UrgStampedNode::cbM, this,
                  boost::arg<1>(),
                  boost::arg<2>(),
                  boost::arg<3>(),
                  boost::arg<4>(),
                  false));
  scip_->registerCallback<scip2::ResponseME>(
      boost::bind(&UrgStampedNode::cbM, this,
                  boost::arg<1>(),
                  boost::arg<2>(),
                  boost::arg<3>(),
                  boost::arg<4>(),
                  true));
  scip_->registerCallback<scip2::ResponseTM>(
      boost::bind(&UrgStampedNode::cbTM, this,
                  boost::arg<1>(),
                  boost::arg<2>(),
                  boost::arg<3>(),
                  boost::arg<4>()));
  scip_->registerCallback<scip2::ResponseQT>(
      boost::bind(&UrgStampedNode::cbQT, this,
                  boost::arg<1>(),
                  boost::arg<2>(),
                  boost::arg<3>()));
  scip_->registerCallback<scip2::ResponseRB>(
      boost::bind(&UrgStampedNode::cbRB, this,
                  boost::arg<1>(),
                  boost::arg<2>(),
                  boost::arg<3>()));
  scip_->registerCallback<scip2::ResponseRS>(
      boost::bind(&UrgStampedNode::cbRS, this,
                  boost::arg<1>(),
                  boost::arg<2>(),
                  boost::arg<3>()));

  if (delay_estim_interval > 0.0)
  {
    timer_delay_estim_ = nh_.createTimer(
        ros::Duration(delay_estim_interval), &UrgStampedNode::delayEstimation, this);
  }
}

void UrgStampedNode::spin()
{
  boost::thread thread(
      boost::bind(&scip2::Connection::spin, device_.get()));
  ros::spin();
  timer_sync_.stop();
  delay_estim_state_ = DelayEstimState::EXITING;
  scip_->sendCommand("QT");
  device_->stop();
  thread.join();
}
}  // namespace urg_stamped
