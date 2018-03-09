/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#include <ros/ros.h>

#include <boost/thread.hpp>

#include <string>

#include <scip/connection.h>

class UrgStampedNode
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  scip::Connection::Ptr scip_;

  void cbClose()
  {
    ros::shutdown();
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

    scip_.reset(new scip::ConnectionTcp(ip, port));
    scip_->registerCloseCallback(
        boost::bind(&UrgStampedNode::cbClose, this));
  }
  void spin()
  {
    boost::thread thread(
        boost::bind(&scip::Connection::spin, scip_.get()));
    ros::spin();
    scip_->stop();
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "urg_stamped");
  UrgStampedNode node;
  node.spin();

  return 1;
}
