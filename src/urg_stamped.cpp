/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

#include <string>

#include <scip2/scip2.h>

class UrgStampedNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_scan_;

  sensor_msgs::LaserScan msg_base_;
  uint32_t step_min_;
  uint32_t step_max_;

  scip2::Connection::Ptr device_;
  scip2::Protocol::Ptr scip_;

  void cbMD(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      const scip2::ScanData &scan)
  {
    sensor_msgs::LaserScan msg(msg_base_);
    msg.header.stamp = ros::Time::fromBoost(time_read);

    msg.ranges.reserve(scan.ranges_.size());
    for (auto &r : scan.ranges_)
      msg.ranges.push_back(r * 1e-3);

    pub_scan_.publish(msg);
  }
  void cbME(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      const scip2::ScanData &scan)
  {
    sensor_msgs::LaserScan msg(msg_base_);
    msg.header.stamp = ros::Time::fromBoost(time_read);

    msg.ranges.reserve(scan.ranges_.size());
    for (auto &r : scan.ranges_)
      msg.ranges.push_back(r * 1e-3);
    msg.intensities.reserve(scan.intensities_.size());
    for (auto &r : scan.intensities_)
      msg.intensities.push_back(r);

    pub_scan_.publish(msg);
  }
  boost::posix_time::ptime time_tm_request;
  std::vector<ros::Duration> communication_delays_;
  void cbTMSend(const boost::posix_time::ptime &time_send)
  {
    time_tm_request = time_send;
  }
  void cbTM(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      const scip2::Timestamp &time_device)
  {
    switch (echo_back[2])
    {
      case '0':
      {
        scip_->sendCommand(
            "TM1",
            boost::bind(&UrgStampedNode::cbTMSend, this, boost::placeholders::_1));
        break;
      }
      case '1':
      {
        const auto delay =
            ros::Time::fromBoost(time_read) -
            ros::Time::fromBoost(time_tm_request);
        communication_delays_.push_back(delay);
        if (communication_delays_.size() > 40)
        {
          sort(communication_delays_.begin(), communication_delays_.end());
          ROS_INFO("communication delay: %0.6f", communication_delays_[20].toSec());
          scip_->sendCommand("TM2");
        }
        else
        {
          scip_->sendCommand(
              "TM1",
              boost::bind(&UrgStampedNode::cbTMSend, this, boost::placeholders::_1));
        }
        break;
      }
      case '2':
      {
        scip_->sendCommand(
            "ME" +
            (boost::format("%04d%04d") % step_min_ % step_max_).str() +
            "00000");
        break;
      }
    }
  }
  void cbPP(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      const std::map<std::string, std::string> &params)
  {
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
    msg_base_.range_min = std::stoi(dmin->second) * 1e-3;
    msg_base_.range_max = std::stoi(dmax->second) * 1e-3;
    msg_base_.angle_min =
        (std::stoi(amin->second) - std::stoi(afrt->second)) * msg_base_.angle_increment;
    msg_base_.angle_max =
        (std::stoi(amax->second) - std::stoi(afrt->second)) * msg_base_.angle_increment;

    scip_->sendCommand("TM0");
  }
  void cbVV(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      const std::map<std::string, std::string> &params)
  {
  }
  void cbII(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      const std::map<std::string, std::string> &params)
  {
  }
  void cbConnect()
  {
    scip_->sendCommand("PP");
  }

public:
  UrgStampedNode()
    : nh_("")
    , pnh_("~")
  {
    std::string ip;
    int port;
    pnh_.param("ip", ip, std::string("192.168.0.10"));
    pnh_.param("port", port, 10940);
    pnh_.param("frame_id", msg_base_.header.frame_id, std::string("laser"));

    pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10);

    device_.reset(new scip2::ConnectionTcp(ip, port));
    device_->registerCloseCallback(ros::shutdown);
    device_->registerConnectCallback(
        boost::bind(&UrgStampedNode::cbConnect, this));

    scip_.reset(new scip2::Protocol(device_));
    scip_->registerCallback<scip2::ResponsePP>(
        boost::bind(&UrgStampedNode::cbPP, this,
                    boost::placeholders::_1,
                    boost::placeholders::_2,
                    boost::placeholders::_3,
                    boost::placeholders::_4));
    scip_->registerCallback<scip2::ResponseVV>(
        boost::bind(&UrgStampedNode::cbVV, this,
                    boost::placeholders::_1,
                    boost::placeholders::_2,
                    boost::placeholders::_3,
                    boost::placeholders::_4));
    scip_->registerCallback<scip2::ResponseII>(
        boost::bind(&UrgStampedNode::cbII, this,
                    boost::placeholders::_1,
                    boost::placeholders::_2,
                    boost::placeholders::_3,
                    boost::placeholders::_4));
    scip_->registerCallback<scip2::ResponseMD>(
        boost::bind(&UrgStampedNode::cbMD, this,
                    boost::placeholders::_1,
                    boost::placeholders::_2,
                    boost::placeholders::_3,
                    boost::placeholders::_4));
    scip_->registerCallback<scip2::ResponseME>(
        boost::bind(&UrgStampedNode::cbME, this,
                    boost::placeholders::_1,
                    boost::placeholders::_2,
                    boost::placeholders::_3,
                    boost::placeholders::_4));
    scip_->registerCallback<scip2::ResponseTM>(
        boost::bind(&UrgStampedNode::cbTM, this,
                    boost::placeholders::_1,
                    boost::placeholders::_2,
                    boost::placeholders::_3,
                    boost::placeholders::_4));
  }
  void spin()
  {
    boost::thread thread(
        boost::bind(&scip2::Connection::spin, device_.get()));
    ros::spin();
    scip_->sendCommand("QT");
    device_->stop();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "urg_stamped");
  UrgStampedNode node;
  node.spin();

  return 1;
}
