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
#include <urg_stamped/Status.h>
#include <std_msgs/Header.h>

#include <boost/bind/bind.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <algorithm>
#include <cstdint>
#include <list>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <scip2/scip2.h>
#include <scip2/walltime.h>
#include <scip2/logger.h>

#include <urg_stamped/ros_logger.h>
#include <urg_stamped/strings.h>

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

  if (!est_)
  {
    scip2::logger::error()
        << "Received scan data before timestamp estimator initialization"
        << std::endl;
    return;
  }

  const uint64_t walltime_device = walltime_.update(scan.timestamp_);
  const ros::Time time_read_ros = ros::Time::fromBoost(time_read);

  if (!next_sync_.isZero() && time_read_ros > next_sync_)
  {
    estimateSensorClock();
    next_sync_ += clock_estim_interval_;
  }

  std::pair<ros::Time, bool> t_scan = est_->scan_->pushScanSample(time_read_ros, walltime_device);
  sensor_msgs::LaserScan msg(msg_base_);
  msg.header.stamp = t_scan.first;
  if (is_uust2_)
  {
    msg.header.stamp += uust2_stamp_offset_;
  }
  if (!t_scan.second)
  {
    scan_drop_count_++;
    scan_drop_continuous_++;
    if (scan_drop_continuous_ > fallback_on_continuous_scan_drop_)
    {
      // Fallback to naive sensor timestamp
      msg.header.stamp = est_->clock_->getClockState().stampToTime(walltime_device);
    }
    else
    {
      return;
    }
  }
  else
  {
    scan_drop_continuous_ = 0;
  }

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

void UrgStampedNode::cbTMSend(
    const boost::posix_time::ptime& time_send, const std::string& cmd)
{
  time_tm_request_ = std::make_pair(cmd, time_send);
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

  if (!est_)
  {
    scip2::logger::error()
        << "Received time sync response before timestamp estimator initialization"
        << std::endl;
    return;
  }

  timer_retry_tm_.stop();
  switch (echo_back[2])
  {
    case '0':
    {
      scip2::logger::debug() << "Entered the time synchronization mode" << std::endl;
      delay_estim_state_ = DelayEstimState::ESTIMATING;

      tm_start_time_ = ros::Time::now();
      est_->clock_->startSync();

      sendTM1();
      break;
    }
    case '1':
    {
      if (time_tm_request_.first != echo_back)
      {
        scip2::logger::info()
            << "Ignoring unmatched TM1 response. Expected " << time_tm_request_.first
            << ", received " << echo_back
            << std::endl;
        break;
      }

      const uint64_t walltime_device = walltime_.update(time_device.timestamp_);

      est_->clock_->pushSyncSample(
          ros::Time::fromBoost(time_tm_request_.second),
          ros::Time::fromBoost(time_read),
          walltime_device);

      if (est_->clock_->hasEnoughSyncSamples())
      {
        scip_->sendCommand("TM2");
        break;
      }

      const std::pair<ros::Duration, ros::Duration> wait = est_->clock_->syncWaitDuration();
      sleepRandom(wait.first.toSec(), wait.second.toSec());

      sendTM1();
      break;
    }
    case '2':
    {
      if (!est_->clock_->finishSync() && !est_->clock_->getClockState().initialized_)
      {
        // Immediately retry if initial clock sync is failed
        scip2::logger::debug()
            << std::setprecision(6) << std::fixed
            << "Retrying initial clock state estimation (took "
            << (ros::Time::now() - tm_start_time_).toSec()
            << ")"
            << std::endl;
        estimateSensorClock();
        break;
      }

      delay_estim_state_ = DelayEstimState::IDLE;
      scip_->sendCommand(
          (publish_intensity_ ? "ME" : "MD") +
          (boost::format("%04d%04d") % step_min_ % step_max_).str() +
          "00000");
      scip2::logger::debug()
          << std::setprecision(6) << std::fixed
          << "Leaving the time synchronization mode (took "
          << (ros::Time::now() - tm_start_time_).toSec()
          << ")"
          << std::endl;
      tm_success_ = true;
      publishStatus();
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
  ideal_scan_interval_ = ros::Duration(60.0 / std::stoi(scan->second));
  msg_base_.scan_time = ideal_scan_interval_.toSec();
  msg_base_.angle_increment = 2.0 * M_PI / std::stoi(ares->second);
  msg_base_.time_increment = msg_base_.scan_time / std::stoi(ares->second);
  msg_base_.range_min = std::stoi(dmin->second) * 1e-3;
  msg_base_.range_max = std::stoi(dmax->second) * 1e-3;
  msg_base_.angle_min =
      (std::stoi(amin->second) - std::stoi(afrt->second)) * msg_base_.angle_increment;
  msg_base_.angle_max =
      (std::stoi(amax->second) - std::stoi(afrt->second)) * msg_base_.angle_increment;

  estimateSensorClock();
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
  if (!est_)
  {
    int firm_major = 1;
    int firm_minor = 0;
    int firm_patch = 0;
    {
      const auto firm_it = params.find("FIRM");
      if (firm_it == params.end())
      {
        scip2::logger::error()
            << "Could not detect sensor hardware revision. Fallback to UUST1 mode"
            << std::endl;
      }
      else
      {
        const std::vector<std::string> version = urg_stamped::strings::split(firm_it->second, '.');
        if (version.size() < 3)
        {
          scip2::logger::error()
              << "Invalid firmware version (" << firm_it->second << "). Fallback to UUST1 mode"
              << std::endl;
        }
        else
        {
          firm_major = std::stoi(version[0]);
          firm_minor = std::stoi(version[1]);
          firm_patch = std::stoi(version[2]);
        }
      }
    }
    std::string prod;
    {
      const auto prod_it = params.find("PROD");
      if (prod_it == params.end())
      {
        scip2::logger::error()
            << "Could not detect sensor model. Fallback to UTM mode"
            << std::endl;
        prod = "UTM";
      }
      else
      {
        prod = prod_it->second.substr(0, 3);
      }
    }
    device_state_estimator::ClockEstimator::Ptr clock;
    device_state_estimator::ScanEstimator::Ptr scan;
    std::string model;
    is_uust2_ = false;
    if (prod == "UTM")
    {
      clock.reset(new device_state_estimator::ClockEstimatorUUST1());
      scan.reset(new device_state_estimator::ScanEstimatorUTM(clock, ideal_scan_interval_));
    }
    else
    {
      if (prod != "UST")
      {
        scip2::logger::info()
            << "Unknown sensor model. Fallback to UST mode"
            << std::endl;
      }
      if (firm_major == 4 && firm_minor == 0 && firm_patch < 3)
      {
        model = "UST (UUST2Unfixed)";
        clock.reset(new device_state_estimator::ClockEstimatorUUST2());
        is_uust2_ = true;
      }
      else
      {
        model = "UST (UUST1, UUST2Fixed)";
        clock.reset(new device_state_estimator::ClockEstimatorUUST1());
      }
      scan.reset(new device_state_estimator::ScanEstimatorUST(clock, ideal_scan_interval_));
    }
    est_.reset(new device_state_estimator::Estimator(clock, scan));
    scip2::logger::info() << "Initialized timestamp estimator for " << model << std::endl;
  }
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
  const bool requested = cmd_resetting_;
  cmd_resetting_ = false;

  if (status != "00")
  {
    scip2::logger::error() << echo_back << " errored with " << status << std::endl;
    if (!requested)
    {
      errorCountIncrement(status);
      return;
    }
    scip2::logger::error() << "Failed to reset. Rebooting the sensor and exiting." << std::endl;
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
  scip_->sendCommand("PP");
  scip_->sendCommand("VV");
  delay_estim_state_ = DelayEstimState::IDLE;
  tm_try_count_ = 0;
}

void UrgStampedNode::cbConnect()
{
  softReset();
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
  scip_->sendCommand("II");
}

void UrgStampedNode::estimateSensorClock(const ros::TimerEvent& event)
{
  if (scan_drop_count_ > 0)
  {
    auto& logger =
        scan_drop_count_ > log_scan_drop_more_than_ ?
            scip2::logger::info() :
            scip2::logger::debug();
    logger
        << "Dropped "
        << scan_drop_count_
        << " scans with large time estimation error" << std::endl;
    scan_drop_count_ = 0;
  }

  tm_try_count_ = 0;
  scip2::logger::debug() << "Starting communication delay estimation" << std::endl;
  delay_estim_state_ = DelayEstimState::STOPPING_SCAN;
  retryTM();
}

void UrgStampedNode::retryTM(const ros::TimerEvent& event)
{
  bool retry = true;
  timer_retry_tm_.stop();
  switch (delay_estim_state_)
  {
    case DelayEstimState::STOPPING_SCAN:
      scip2::logger::debug() << "Stopping scan" << std::endl;
      publishSyncStart();
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
      retry = false;
      break;
    default:
      retry = false;
      break;
  }
  if (retry)
  {
    timer_retry_tm_ = nh_.createTimer(tm_command_interval_, &UrgStampedNode::retryTM, this, true);
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
  cmd_resetting_ = true;
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

void UrgStampedNode::sendTM1()
{
  std::stringstream cmd;
  tm_key_++;
  cmd << "TM1;" << std::hex << tm_key_;
  scip_->sendCommand(
      cmd.str(),
      boost::bind(&UrgStampedNode::cbTMSend, this, boost::arg<1>(), cmd.str()));
}

void UrgStampedNode::publishStatus()
{
  if (!est_)
  {
    return;
  }
  const device_state_estimator::ClockState clock = est_->clock_->getClockState();
  const device_state_estimator::ScanState scan = est_->scan_->getScanState();
  const device_state_estimator::CommDelay comm_delay = est_->clock_->getCommDelay();

  urg_stamped::Status msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = msg_base_.header.frame_id;
  msg.sensor_clock_origin = clock.origin_;
  msg.sensor_clock_gain = clock.gain_;
  msg.communication_delay = comm_delay.min_;
  msg.communication_delay_sigma = comm_delay.sigma_;
  msg.scan_time_origin = scan.origin_;
  msg.scan_interval = scan.interval_;
  pub_status_.publish(msg);
}

void UrgStampedNode::cbSyncStart(const std_msgs::Header::ConstPtr& msg)
{
  if (msg->frame_id == msg_base_.header.frame_id)
  {
    return;
  }
  switch (delay_estim_state_)
  {
    case DelayEstimState::IDLE:
    case DelayEstimState::EXITING:
      return;
      break;
    default:
      break;
  }
  std::uniform_real_distribution<double> rnd(0, clock_estim_interval_.toSec());
  ros::Duration offset(rnd(random_engine_));
  next_sync_ += offset;
  scip2::logger::info()
      << std::fixed << std::setprecision(3)
      << "Concurrent time synchronization detected. Delaying the next time sync by "
      << offset.toSec() << "s"
      << std::endl;
}

void UrgStampedNode::publishSyncStart()
{
  std_msgs::Header msg;
  msg.frame_id = msg_base_.header.frame_id;
  msg.stamp = ros::Time::now();
  pub_sync_start_.publish(msg);
}

UrgStampedNode::UrgStampedNode()
  : nh_("")
  , pnh_("~")
  , failed_(false)
  , delay_estim_state_(DelayEstimState::IDLE)
  , tm_key_(0)
  , last_sync_time_(0)
  , tm_success_(false)
  , scan_drop_count_(0)
  , scan_drop_continuous_(0)
  , cmd_resetting_(false)
  , is_uust2_(false)
{
  std::random_device rd;
  random_engine_.seed(rd());

  std::string ip;
  int port;
  double clock_estim_interval;

  pnh_.param("ip_address", ip, std::string("192.168.0.10"));
  pnh_.param("ip_port", port, 10940);
  pnh_.param("frame_id", msg_base_.header.frame_id, std::string("laser"));
  pnh_.param("publish_intensity", publish_intensity_, true);
  pnh_.param("clock_estim_interval", clock_estim_interval, 30.0);
  pnh_.param("error_limit", error_count_max_, 4);
  pnh_.param("fallback_on_continuous_scan_drop", fallback_on_continuous_scan_drop_, 5);
  clock_estim_interval_ = ros::Duration(clock_estim_interval);

  // 30s * 40Hz = 1200scans total, output info level log if dropped 90/1200scans or more
  pnh_.param("log_scan_drop_more_than", log_scan_drop_more_than_, static_cast<int>(clock_estim_interval * 3));

  double tm_interval, tm_timeout;
  pnh_.param("tm_interval", tm_interval, 0.06);
  tm_command_interval_ = ros::Duration(tm_interval);
  pnh_.param("tm_timeout", tm_timeout, 10.0);
  tm_try_max_ = static_cast<int>(tm_timeout / tm_interval);

  // UUST2 doesn't support estimating communication delay. Static offset can be specified to compensate the delay
  double uust2_stamp_offset;
  pnh_.param("uust2_stamp_offset", uust2_stamp_offset, 0.0);
  uust2_stamp_offset_ = ros::Duration(uust2_stamp_offset);

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
  pub_status_ = pnh_.advertise<urg_stamped::Status>("status", 1, true);

  pub_sync_start_ = nh_.advertise<std_msgs::Header>("urg_stamped_sync_start", 1, true);
  sub_sync_start_ = nh_.subscribe("urg_stamped_sync_start", 1, &UrgStampedNode::cbSyncStart, this);

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

  if (clock_estim_interval_ > ros::Duration(0))
  {
    next_sync_ = ros::Time::now() + clock_estim_interval_;
  }
}

void UrgStampedNode::spin()
{
  boost::thread thread(
      boost::bind(&scip2::Connection::spin, device_.get()));
  ros::spin();
  delay_estim_state_ = DelayEstimState::EXITING;
  scip_->sendCommand("QT");
  device_->stop();
  thread.join();
}
}  // namespace urg_stamped
