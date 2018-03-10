/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

#include <string>

#include <scip/scip.h>

class UrgStampedNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  scip::Connection::Ptr device_;
  scip::Protocol::Ptr scip_;

  void cbMD(
      const std::string &echo_back,
      const std::string &status,
      const scip::ScanData &scan)
  {
    ROS_ERROR("MD");
    for (auto &r : scan.ranges_)
    {
      std::cout << r << " ";
    }
    std::cout << std::endl;
  }
  void cbME(
      const std::string &echo_back,
      const std::string &status,
      const scip::ScanData &scan)
  {
    ROS_ERROR("ME");
    size_t i = 0;
    for (auto &r : scan.ranges_)
    {
      std::cout << r << "(" << scan.intensities_[i] << ") ";
    }
    std::cout << std::endl;
  }
  void cbPP(
      const std::string &echo_back,
      const std::string &status,
      const std::map<std::string, std::string> &params)
  {
    ROS_ERROR("PP");
    const auto amin = params.find("AMIN");
    const auto amax = params.find("AMAX");
    if (amin == params.end() || amax == params.end())
    {
      ROS_ERROR("PP doesn't have AMIN and AMAX");
      return;
    }
    scip_->sendCommand(
        "ME" +
        (boost::format("%04d%04d") % std::stoi(amin->second) % std::stoi(amax->second)).str() +
        "00030");
  }
  void cbVV(
      const std::string &echo_back,
      const std::string &status,
      const std::map<std::string, std::string> &params)
  {
    ROS_ERROR("VV");
  }
  void cbII(
      const std::string &echo_back,
      const std::string &status,
      const std::map<std::string, std::string> &params)
  {
    ROS_ERROR("II");
  }
  void cbConnect()
  {
    scip_->sendCommand("PP");
    scip_->sendCommand("VV");
    scip_->sendCommand("II");
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

    device_.reset(new scip::ConnectionTcp(ip, port));
    device_->registerCloseCallback(ros::shutdown);
    device_->registerConnectCallback(
        boost::bind(&UrgStampedNode::cbConnect, this));

    scip_.reset(new scip::Protocol(device_));
    scip_->registerCallback<scip::ResponsePP>(
        boost::bind(&UrgStampedNode::cbPP, this,
                    boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
    scip_->registerCallback<scip::ResponseVV>(
        boost::bind(&UrgStampedNode::cbVV, this,
                    boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
    scip_->registerCallback<scip::ResponseII>(
        boost::bind(&UrgStampedNode::cbII, this,
                    boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
    scip_->registerCallback<scip::ResponseMD>(
        boost::bind(&UrgStampedNode::cbMD, this,
                    boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
    scip_->registerCallback<scip::ResponseME>(
        boost::bind(&UrgStampedNode::cbME, this,
                    boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
  }
  void spin()
  {
    boost::thread thread(
        boost::bind(&scip::Connection::spin, device_.get()));
    ros::spin();
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
