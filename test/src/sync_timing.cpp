/*
 * Copyright 2024 The urg_stamped Authors
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

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio/ip/tcp.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include <urg_stamped/Status.h>

#include <ros/network.h>
#include <ros/xmlrpc_manager.h>
#include <xmlrpcpp/XmlRpc.h>

#include <urg_sim/urg_sim.h>
#include <e2e_utils.h>

#include <gtest/gtest.h>

class SyncTiming : public ::testing::Test
{
public:
  SyncTiming()
    : nh_("")
  {
    sub_sync_start_ = nh_.subscribe("urg_stamped_sync_start", 100, &SyncTiming::cbSyncStart, this);
  }

  void TearDown()
  {
    for (int i = 0; i < 2; i++)
    {
      if (sim_[i])
      {
        sim_[i]->kill();
        th_sim_[i].join();
      }
    }
  }

protected:
  ros::NodeHandle nh_;
  ros::Subscriber sub_sync_start_;

  urg_sim::URGSimulator* sim_[2];
  std::thread th_sim_[2];
  std::map<std::string, ros::Time> sync_start_time_;

  void cbRawScanData(const urg_sim::RawScanData::Ptr data)
  {
  }

  void cbSyncStart(const std_msgs::Header::ConstPtr& msg)
  {
    sync_start_time_[msg->frame_id] = msg->stamp;
  }

  void spinFor(const ros::Duration& timeout)
  {
    const ros::Time deadline = ros::Time::now() + timeout;
    ros::Rate wait(10);
    while (ros::Time::now() < deadline)
    {
      wait.sleep();
      ros::spinOnce();
      ASSERT_TRUE(ros::ok());
    }
  }

  void startSimulator(const int id, const urg_sim::URGSimulator::Params& params)
  {
    sim_[id] = new urg_sim::URGSimulator(
        boost::asio::ip::tcp::endpoint(
            boost::asio::ip::tcp::v4(),
            0),
        params,
        std::bind(&SyncTiming::cbRawScanData, this, std::placeholders::_1));
    th_sim_[id] = std::thread(std::bind(&urg_sim::URGSimulator::spin, sim_[id]));
    ros::Duration(0.1).sleep();  // Wait boot
  }

  void startUrgStamped()
  {
    nh_.setParam("/urg_stamped0/ip_port", sim_[0]->getLocalEndpoint().port());
    nh_.setParam("/urg_stamped1/ip_port", sim_[1]->getLocalEndpoint().port());

    // Shutdown urg_stamped to initialize internal state and reload parameters
    if (!shutdownNode("urg_stamped0") || !shutdownNode("urg_stamped1"))
    {
      ros::Duration(1).sleep();  // Retry
      ASSERT_TRUE(shutdownNode("urg_stamped0"));
      ASSERT_TRUE(shutdownNode("urg_stamped1"));
    }
  }
};

TEST_F(SyncTiming, NoConcurrentSync)
{
  const urg_sim::URGSimulator::Params params =
      {
          .model = urg_sim::URGSimulator::Model::UTM,
          .boot_duration = 0.01,
          .comm_delay_base = 0.0005,
          .comm_delay_sigma = 0.0005,
          .scan_interval = 0.025,
          .clock_rate = 1.0,
          .hex_ii_timestamp = false,
          .angle_resolution = 1440,
          .angle_min = 0,
          .angle_max = 1080,
          .angle_front = 540,
      };

  ASSERT_NO_FATAL_FAILURE(startSimulator(0, params));
  ASSERT_NO_FATAL_FAILURE(startSimulator(1, params));
  ros::Duration(1).sleep();  // Wait startup

  ASSERT_NO_FATAL_FAILURE(startUrgStamped());
  ros::Duration(1).sleep();  // Wait node respawn

  ASSERT_NO_FATAL_FAILURE(spinFor(ros::Duration(5)));

  ASSERT_TRUE(sync_start_time_["laser0"].isValid());
  ASSERT_TRUE(sync_start_time_["laser1"].isValid());

  // Base interval is 1s
  const double diff = std::remainder(
      (sync_start_time_["laser0"] - sync_start_time_["laser1"]).toSec(), 1.0);

  // Sync takes more than 0.1s on this configuration,
  // so the timings must have difference larger than 0.1s
  ASSERT_GT(std::abs(diff), 0.1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "urg_stamped_e2e");
  ros::NodeHandle nh;  // workaround to keep the test node during the process life time

  return RUN_ALL_TESTS();
}

