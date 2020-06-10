/*
 * Copyright 2018-2019 The urg_stamped Authors
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

#include <device_time_origin.h>
#include <first_order_filter.h>
#include <timestamp_moving_average.h>
#include <timestamp_outlier_remover.h>
#include <ros_logger.h>

class UrgStampedNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_scan_;
  ros::Timer timer_sync_;
  ros::Timer timer_delay_estim_;
  ros::Timer timer_try_tm_;

  int error_count_;
  int error_count_max_;

  sensor_msgs::LaserScan msg_base_;
  uint32_t step_min_;
  uint32_t step_max_;

  scip2::Connection::Ptr device_;
  scip2::Protocol::Ptr scip_;

  bool publish_intensity_;

  boost::posix_time::ptime time_tm_request;
  std::list<ros::Duration> communication_delays_;
  std::list<ros::Time> device_time_origins_;
  ros::Duration estimated_communication_delay_;
  size_t tm_iter_num_;
  size_t tm_median_window_;
  bool estimated_communication_delay_init_;
  double communication_delay_filter_alpha_;

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

  void cbM(
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
        ROS_ERROR("%s errored with %s", echo_back.c_str(), status.c_str());
        errorCountIncrement();
      }
      return;
    }

    const uint64_t walltime_device = walltime_.update(scan.timestamp_);
    if (detectDeviceTimeJump(time_read, walltime_device))
    {
      errorCountIncrement();
      return;
    }
    error_count_ = 0;

    const auto estimated_timestamp_lf =
        device_time_origin_.origin_ +
        ros::Duration().fromNSec(walltime_device * 1e6 * device_time_origin_.gain_) +
        ros::Duration(msg_base_.time_increment * step_min_);

    if (t0_ == ros::Time(0))
      t0_ = estimated_timestamp_lf;

    const auto receive_time =
        timestamp_outlier_removal_.update(
            ros::Time::fromBoost(time_read) -
            estimated_communication_delay_ * 0.5 -
            ros::Duration(msg_base_.scan_time));

    sensor_msgs::LaserScan msg(msg_base_);
    msg.header.stamp =
        timestamp_moving_average_.update(
            t0_ +
            ros::Duration(
                timestamp_lpf_.update((estimated_timestamp_lf - t0_).toSec()) +
                timestamp_hpf_.update((receive_time - t0_).toSec())));

    if (scan.ranges_.size() != step_max_ - step_min_ + 1)
    {
      ROS_DEBUG("Size of the received scan data is wrong (expected: %d, received: %lu); refreshing",
                step_max_ - step_min_ + 1, scan.ranges_.size());
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
  }
  void cbTMSend(const boost::posix_time::ptime& time_send)
  {
    time_tm_request = time_send;
  }
  void cbTM(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status,
      const scip2::Timestamp& time_device)
  {
    if (status != "00")
    {
      ROS_ERROR("%s errored with %s", echo_back.c_str(), status.c_str());
      errorCountIncrement();
      return;
    }

    timer_try_tm_.stop();
    switch (echo_back[2])
    {
      case '0':
      {
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
          errorCountIncrement();
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
          ROS_DEBUG("delay: %0.6f, device timestamp: %ld, device time origin: %0.6f",
                    estimated_communication_delay_.toSec(),
                    walltime_device,
                    origins[tm_iter_num_ / 2].toSec());
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
        scip_->sendCommand(
            (publish_intensity_ ? "ME" : "MD") +
            (boost::format("%04d%04d") % step_min_ % step_max_).str() +
            "00000");
        timeSync();
        timestamp_outlier_removal_.reset();
        timestamp_moving_average_.reset();
        t0_ = ros::Time();
        break;
      }
    }
  }
  void cbPP(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status,
      const std::map<std::string, std::string>& params)
  {
    if (status != "00")
    {
      ROS_ERROR("%s errored with %s", echo_back.c_str(), status.c_str());
      errorCountIncrement();
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
      ROS_ERROR("PP doesn't have required parameters");
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
  void cbVV(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status,
      const std::map<std::string, std::string>& params)
  {
    if (status != "00")
    {
      ROS_ERROR("%s errored with %s", echo_back.c_str(), status.c_str());
      errorCountIncrement();
      return;
    }
  }
  void cbIISend(const boost::posix_time::ptime& time_send)
  {
    time_ii_request = time_send;
  }
  void cbII(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status,
      const std::map<std::string, std::string>& params)
  {
    if (status != "00")
    {
      ROS_ERROR("%s errored with %s", echo_back.c_str(), status.c_str());
      errorCountIncrement();
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
        ROS_DEBUG("II doesn't have timestamp");
        return;
      }
      if (time->second.size() != 6 && time->second.size() != 4)
      {
        ROS_DEBUG("Timestamp in II is ill-formatted (%s)", time->second.c_str());
        return;
      }
      const uint32_t time_device =
          time->second.size() == 6 ?
              std::stoi(time->second, nullptr, 16) :
              *(scip2::Decoder<4>(time->second).begin());

      const uint64_t walltime_device = walltime_.update(time_device);
      if (detectDeviceTimeJump(time_read, walltime_device))
      {
        errorCountIncrement();
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

      ROS_DEBUG("on scan delay: %0.6f, device timestamp: %ld, device time origin: %0.6f, gain: %0.6f",
                delay.toSec(),
                walltime_device,
                origin.toSec(),
                updated_gain);
    }
    else
    {
      ROS_DEBUG("on scan delay (%0.6f) is larger than expected; skipping",
                delay.toSec());
    }
  }
  void cbQT(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status)
  {
    if (status != "00")
    {
      ROS_ERROR("%s errored with %s", echo_back.c_str(), status.c_str());
      errorCountIncrement();
      return;
    }

    ROS_DEBUG("Scan data stopped");
  }
  void cbConnect()
  {
    scip_->sendCommand("PP");
    device_->startWatchdog(boost::posix_time::seconds(1));
  }

  void timeSync(const ros::TimerEvent& event = ros::TimerEvent())
  {
    scip_->sendCommand(
        "II",
        boost::bind(&UrgStampedNode::cbIISend, this, boost::arg<1>()));
    timer_sync_ = nh_.createTimer(
        ros::Duration(sync_interval_(random_engine_)),
        &UrgStampedNode::timeSync, this, true);
  }
  void delayEstimation(const ros::TimerEvent& event = ros::TimerEvent())
  {
    timer_sync_.stop();
    ROS_DEBUG("Starting communication delay estimation");
    scip_->sendCommand("QT");
    timer_try_tm_ = nh_.createTimer(
        ros::Duration(0.05),
        &UrgStampedNode::tryTM, this);
  }
  void tryTM(const ros::TimerEvent& event = ros::TimerEvent())
  {
    scip_->sendCommand("QT");
    scip_->sendCommand("TM0");
  }

  void errorCountIncrement()
  {
    ++error_count_;
    if (error_count_ > error_count_max_)
    {
      ROS_ERROR("Error count exceeded limit, resetting the sensor and exiting.");
      scip_->sendCommand("RS");
      ros::Duration(0.05).sleep();
      ros::shutdown();
    }
  }

  bool detectDeviceTimeJump(
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
      ROS_ERROR(
          "Device time origin jumped.\n"
          "last origin: %0.3f, current origin: %0.3f, "
          "allowed_device_time_origin_diff: %0.3f, device_timestamp: %ld",
          device_time_origin_.origin_.toSec(), current_origin.toSec(),
          allowed_device_time_origin_diff_, device_timestamp);
    }
    return jumped;
  }

public:
  UrgStampedNode()
    : nh_("")
    , pnh_("~")
    , error_count_(0)
    , tm_iter_num_(5)
    , tm_median_window_(35)
    , estimated_communication_delay_init_(false)
    , communication_delay_filter_alpha_(0.3)
    , last_sync_time_(0)
    , timestamp_lpf_(20)
    , timestamp_hpf_(20)
    , timestamp_outlier_removal_(ros::Duration(0.001), ros::Duration())
    , timestamp_moving_average_(5, ros::Duration())
  {
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

    pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);

    device_.reset(new scip2::ConnectionTcp(ip, port));
    device_->registerCloseCallback(ros::shutdown);
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

    if (delay_estim_interval > 0.0)
    {
      timer_delay_estim_ = nh_.createTimer(
          ros::Duration(delay_estim_interval), &UrgStampedNode::delayEstimation, this);
    }
  }
  void spin()
  {
    boost::thread thread(
        boost::bind(&scip2::Connection::spin, device_.get()));
    ros::spin();
    timer_sync_.stop();
    scip_->sendCommand("QT");
    device_->stop();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "urg_stamped");
  urg_stamped::setROSLogger();

  UrgStampedNode node;
  node.spin();

  return 1;
}
