/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#include <ros/ros.h>

#include <boost/thread.hpp>

#include <string>

#include <scip/scip.h>

class UrgStampedNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  scip::Connection::Ptr device_;
  scip::Protocol::Ptr scip_;

  void cbConnect()
  {
    scip_->sendCommand("PP");
    scip_->sendCommand("VV");
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
