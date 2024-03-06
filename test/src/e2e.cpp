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

#include <iostream>
#include <vector>

#include <boost/asio/ip/tcp.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <urg_stamped/urg_stamped.h>
#include <urg_sim/urg_sim.h>

#include <gtest/gtest.h>

class E2E : public ::testing::Test
{
public:
  E2E()
    : nh_()
    , pnh_("~")
    , sub_scan_(nh_.subscribe("scan", 100, &E2E::cbScan, this))
    , cnt_(0)
  {
  }

  void TearDown()
  {
    if (sim_)
    {
      sim_->kill();
      th_sim_.join();
      th_node_.join();
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_scan_;

  urg_sim::URGSimulator* sim_;
  urg_stamped::UrgStampedNode* node_;
  std::thread th_sim_;
  std::thread th_node_;

  size_t cnt_;

  void cbScan(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    scans_.push_back(msg);
  }

  void cbRawScanData(const urg_sim::RawScanData::Ptr data)
  {
    data->ranges[0] = cnt_;
    cnt_++;
    raw_scans_.push_back(data);
  }

protected:
  std::vector<sensor_msgs::LaserScan::ConstPtr> scans_;
  std::vector<urg_sim::RawScanData::Ptr> raw_scans_;

  void waitScans(const size_t num, const ros::Duration& timeout)
  {
    const ros::Time deadline = ros::Time::now() + timeout;
    ros::Rate wait(10);
    while (scans_.size() <= num)
    {
      wait.sleep();
      ros::spinOnce();
      ASSERT_TRUE(ros::ok());
      ASSERT_LT(ros::Time::now(), deadline) << "Timeout";
    }
  }

  void startSimulator(const urg_sim::URGSimulator::Params& params)
  {
    sim_ = new urg_sim::URGSimulator(
        boost::asio::ip::tcp::endpoint(
            boost::asio::ip::tcp::v4(),
            0),
        params,
        std::bind(&E2E::cbRawScanData, this, std::placeholders::_1));
    th_sim_ = std::thread(std::bind(&urg_sim::URGSimulator::spin, sim_));
    ros::Duration(0.1).sleep();  // Wait boot

    pnh_.setParam("ip_address", "127.0.0.1");
    pnh_.setParam("ip_port", sim_->getLocalEndpoint().port());

    node_ = new urg_stamped::UrgStampedNode();
    th_node_ = std::thread(std::bind(&urg_stamped::UrgStampedNode::spin, node_));

    waitScans(1, ros::Duration(5));
  }
};

TEST_F(E2E, Simple)
{
  const urg_sim::URGSimulator::Params params =
      {
          .model = urg_sim::URGSimulator::Model::UTM,
          .boot_duration = 0.01,
          .comm_delay_base = 0.002,
          .comm_delay_sigma = 0.0002,
          .scan_interval = 0.025,
          .clock_rate = 1.0,
          .hex_ii_timestamp = false,
          .angle_resolution = 1440,
          .angle_min = 0,
          .angle_max = 1080,
          .angle_front = 540,
      };
  startSimulator(params);

  waitScans(100, ros::Duration(10));

  std::map<int, ros::Time> expected_stamps_;
  for (const urg_sim::RawScanData::Ptr raw : raw_scans_)
  {
    const int index = raw->ranges[0];
    expected_stamps_[index] = ros::Time::fromBoost(raw->full_time);
  }

  int cnt = 0;
  for (const sensor_msgs::LaserScan::ConstPtr scan : scans_)
  {
    cnt++;
    if (cnt > 50)
    {
      const int index = std::lround(scan->ranges[0] * 1000);
      ASSERT_NE(expected_stamps_.find(index), expected_stamps_.end()) << "Can not find corresponding ground truth timestamp";
      std::cerr << (expected_stamps_[index] - scan->header.stamp).toSec() << std::endl;
      ASSERT_LT(expected_stamps_[index] - scan->header.stamp, ros::Duration(0.0015));
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "urg_stamped_e2e");
  ros::NodeHandle nh;  // workaround to keep the test node during the process life time

  return RUN_ALL_TESTS();
}

