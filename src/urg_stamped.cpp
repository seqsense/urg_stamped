/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

#include <string>

#include <scip/scip.h>

class UrgStampedNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_scan_;

  sensor_msgs::LaserScan msg_base_;

  scip::Connection::Ptr device_;
  scip::Protocol::Ptr scip_;

  void cbMD(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      const scip::ScanData &scan)
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
      const scip::ScanData &scan)
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
    msg_base_.scan_time = 60.0 / std::stoi(scan->second);
    msg_base_.angle_increment = 2.0 * M_PI / std::stoi(ares->second);
    msg_base_.range_min = std::stoi(dmin->second) * 1e-3;
    msg_base_.range_max = std::stoi(dmax->second) * 1e-3;
    msg_base_.angle_min =
        (std::stoi(amin->second) - std::stoi(afrt->second)) * msg_base_.angle_increment;
    msg_base_.angle_max =
        (std::stoi(amax->second) - std::stoi(afrt->second)) * msg_base_.angle_increment;

    scip_->sendCommand(
        "ME" +
        (boost::format("%04d%04d") % std::stoi(amin->second) % std::stoi(amax->second)).str() +
        "00000");
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

    device_.reset(new scip::ConnectionTcp(ip, port));
    device_->registerCloseCallback(ros::shutdown);
    device_->registerConnectCallback(
        boost::bind(&UrgStampedNode::cbConnect, this));

    scip_.reset(new scip::Protocol(device_));
    scip_->registerCallback<scip::ResponsePP>(
        boost::bind(&UrgStampedNode::cbPP, this,
                    boost::placeholders::_1,
                    boost::placeholders::_2,
                    boost::placeholders::_3,
                    boost::placeholders::_4));
    scip_->registerCallback<scip::ResponseVV>(
        boost::bind(&UrgStampedNode::cbVV, this,
                    boost::placeholders::_1,
                    boost::placeholders::_2,
                    boost::placeholders::_3,
                    boost::placeholders::_4));
    scip_->registerCallback<scip::ResponseII>(
        boost::bind(&UrgStampedNode::cbII, this,
                    boost::placeholders::_1,
                    boost::placeholders::_2,
                    boost::placeholders::_3,
                    boost::placeholders::_4));
    scip_->registerCallback<scip::ResponseMD>(
        boost::bind(&UrgStampedNode::cbMD, this,
                    boost::placeholders::_1,
                    boost::placeholders::_2,
                    boost::placeholders::_3,
                    boost::placeholders::_4));
    scip_->registerCallback<scip::ResponseME>(
        boost::bind(&UrgStampedNode::cbME, this,
                    boost::placeholders::_1,
                    boost::placeholders::_2,
                    boost::placeholders::_3,
                    boost::placeholders::_4));
  }
  void spin()
  {
    boost::thread thread(
        boost::bind(&scip::Connection::spin, device_.get()));
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
