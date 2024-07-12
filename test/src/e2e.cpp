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
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio/ip/tcp.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <urg_stamped/Status.h>

#include <ros/network.h>
#include <ros/xmlrpc_manager.h>
#include <xmlrpcpp/XmlRpc.h>

#include <urg_sim/urg_sim.h>

#include <gtest/gtest.h>

namespace
{

bool shutdownUrgStamped()
{
  XmlRpc::XmlRpcValue req, res, payload;
  req[0] = ros::this_node::getName();
  req[1] = "urg_stamped";
  if (!ros::master::execute("lookupNode", req, res, payload, true))
  {
    return false;
  }

  std::string host;
  uint32_t port;
  if (!ros::network::splitURI(static_cast<std::string>(res[2]), host, port))
  {
    return false;
  }

  XmlRpc::XmlRpcClient cli(host.c_str(), port, "/");
  XmlRpc::XmlRpcValue req2, res2;
  return cli.execute("shutdown", req2, res2);
}

}  // namespace

class E2E : public ::testing::Test
{
public:
  E2E()
    : nh_("")
    , pnh_("~")
    , sim_killed_(false)
    , cnt_(0)
  {
  }

  void TearDown()
  {
    if (sim_)
    {
      sim_->kill();
      th_sim_.join();
    }
  }

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_scan_;
  ros::Subscriber sub_status_;

  urg_sim::URGSimulator* sim_;
  std::thread th_sim_;
  bool sim_killed_;
  urg_stamped::Status::ConstPtr status_msg_;

  int cnt_;

  std::vector<sensor_msgs::LaserScan::ConstPtr> scans_;
  std::vector<urg_sim::RawScanData::Ptr> raw_scans_;
  std::map<int, ros::Time> true_stamps_;

  void cbScan(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
    scans_.push_back(msg);
  }

  void cbStatus(const urg_stamped::Status ::ConstPtr& msg)
  {
    status_msg_ = msg;
  }

  void cbRawScanData(const urg_sim::RawScanData::Ptr data)
  {
    data->ranges[0] = cnt_;
    raw_scans_.push_back(data);
    true_stamps_[cnt_] = ros::Time::fromBoost(data->full_time);
    cnt_++;
  }

  void waitScans(const size_t num, const ros::Duration& timeout)
  {
    const ros::Time deadline = ros::Time::now() + timeout;
    ros::Rate wait(10);
    while (scans_.size() <= num)
    {
      wait.sleep();
      ros::spinOnce();
      ASSERT_TRUE(ros::ok());
      ASSERT_LT(ros::Time::now(), deadline) << "Timeout: received " << scans_.size() << "/" << num;
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
  }

  void startUrgStamped()
  {
    nh_.setParam("/urg_stamped/ip_address", "127.0.0.1");
    nh_.setParam("/urg_stamped/ip_port", sim_->getLocalEndpoint().port());

    // Shutdown urg_stamped to initialize internal state and reload parameters
    if (!shutdownUrgStamped())
    {
      ros::Duration(1).sleep();  // Retry
      ASSERT_TRUE(shutdownUrgStamped());
    }
    ros::Duration(1).sleep();  // Wait node respawn

    sub_scan_ = nh_.subscribe("scan", 100, &E2E::cbScan, this);
    sub_status_ = nh_.subscribe("/urg_stamped/status", 10, &E2E::cbStatus, this);
  }
};

class E2EWithParam : public E2E,
                     public ::testing::WithParamInterface<urg_sim::URGSimulator::Params>
{
};

std::vector<urg_sim::URGSimulator::Params> params(
    {
        {
            .model = urg_sim::URGSimulator::Model::UTM,
            .boot_duration = 0.01,
            .comm_delay_base = 0.0002,
            .comm_delay_sigma = 0.0004,
            .scan_interval = 0.02505,
            .clock_rate = 1.0,
            .hex_ii_timestamp = false,
            .angle_resolution = 1440,
            .angle_min = 0,
            .angle_max = 1080,
            .angle_front = 540,
        },  // NOLINT(whitespace/braces)
        {
            .model = urg_sim::URGSimulator::Model::UTM,
            .boot_duration = 0.01,
            .comm_delay_base = 0.0002,
            .comm_delay_sigma = 0.0004,
            .scan_interval = 0.02505,
            .clock_rate = 1.001,
            .hex_ii_timestamp = false,
            .angle_resolution = 1440,
            .angle_min = 0,
            .angle_max = 1080,
            .angle_front = 540,
        },  // NOLINT(whitespace/braces)
        {
            .model = urg_sim::URGSimulator::Model::UTM,
            .boot_duration = 0.01,
            .comm_delay_base = 0.0002,
            .comm_delay_sigma = 0.0004,
            .scan_interval = 0.02505,
            .clock_rate = 0.999,
            .hex_ii_timestamp = false,
            .angle_resolution = 1440,
            .angle_min = 0,
            .angle_max = 1080,
            .angle_front = 540,
        },  // NOLINT(whitespace/braces)
        {
            .model = urg_sim::URGSimulator::Model::UST,
            .boot_duration = 0.01,
            .comm_delay_base = 0.0005,
            .comm_delay_sigma = 0.0005,
            .scan_interval = 0.02505,
            .clock_rate = 1.0,
            .hex_ii_timestamp = true,
            .angle_resolution = 1440,
            .angle_min = 0,
            .angle_max = 1080,
            .angle_front = 540,
        },  // NOLINT(whitespace/braces)
        {
            .model = urg_sim::URGSimulator::Model::UST,
            .boot_duration = 0.01,
            .comm_delay_base = 0.0005,
            .comm_delay_sigma = 0.0005,
            .scan_interval = 0.02505,
            .clock_rate = 1.001,
            .hex_ii_timestamp = true,
            .angle_resolution = 1440,
            .angle_min = 0,
            .angle_max = 1080,
            .angle_front = 540,
        },  // NOLINT(whitespace/braces)
        {
            .model = urg_sim::URGSimulator::Model::UST,
            .boot_duration = 0.01,
            .comm_delay_base = 0.0005,
            .comm_delay_sigma = 0.0005,
            .scan_interval = 0.02505,
            .clock_rate = 0.999,
            .hex_ii_timestamp = true,
            .angle_resolution = 1440,
            .angle_min = 0,
            .angle_max = 1080,
            .angle_front = 540,
        },  // NOLINT(whitespace/braces)
    });     // NOLINT(whitespace/braces)

INSTANTIATE_TEST_CASE_P(
    Simple, E2EWithParam,
    ::testing::ValuesIn(params));

TEST_P(E2EWithParam, Simple)
{
  const auto param = GetParam();
  ASSERT_NO_FATAL_FAILURE(startSimulator(param));

  // Make time sync happens frequently
  pnh_.setParam("/urg_stamped/clock_estim_interval", 2.5);
  pnh_.setParam("/urg_stamped/error_limit", 4);
  pnh_.setParam("/urg_stamped/debug", true);
  ASSERT_NO_FATAL_FAILURE(startUrgStamped());

  ASSERT_NO_FATAL_FAILURE(waitScans(300, ros::Duration(10)));

  int err_rms = 0;
  for (size_t i = 250; i < 300; ++i)
  {
    const int index = std::lround(scans_[i]->ranges[0] * 1000);
    ASSERT_NE(true_stamps_.find(index), true_stamps_.end()) << "Can not find corresponding ground truth timestamp";
    const double err = (true_stamps_[index] - scans_[i]->header.stamp).toSec();
    EXPECT_LT(std::abs(err), 0.001)
        << std::setprecision(9) << std::fixed
        << "scan " << i << " gain " << status_msg_->sensor_clock_gain
        << " stamp " << scans_[i]->header.stamp;
    err_rms += err * err;
  }
  err_rms = std::sqrt(err_rms / 50);
  EXPECT_LT(err_rms, 0.0002);

  ASSERT_TRUE(static_cast<bool>(status_msg_));
  ASSERT_NEAR(status_msg_->sensor_clock_gain, param.clock_rate, 2.5e-4);
  ASSERT_GE(status_msg_->communication_delay.toSec(), param.comm_delay_base);
  ASSERT_LT(status_msg_->communication_delay.toSec(), param.comm_delay_base + param.comm_delay_sigma * 3);
  ASSERT_NEAR(status_msg_->scan_interval.toSec(), param.scan_interval, 5e-5);
}

TEST_F(E2E, RebootOnError)
{
  const urg_sim::URGSimulator::Params params =
      {
          .model = urg_sim::URGSimulator::Model::UTM,
          .boot_duration = 0.01,
          .comm_delay_base = 0.00025,
          .comm_delay_sigma = 0.00005,
          .scan_interval = 0.025,
          .clock_rate = 1.0,
          .hex_ii_timestamp = false,
          .angle_resolution = 1440,
          .angle_min = 0,
          .angle_max = 1080,
          .angle_front = 540,
      };

  ASSERT_NO_FATAL_FAILURE(startSimulator(params));
  sim_->setState(urg_sim::URGSimulator::SensorState::ERROR_DETECTED);

  pnh_.setParam("/urg_stamped/error_limit", 0);
  ASSERT_NO_FATAL_FAILURE(startUrgStamped());

  ros::Duration(2).sleep();
  ASSERT_GE(sim_->getBootCnt(), 2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "urg_stamped_e2e");
  ros::NodeHandle nh;  // workaround to keep the test node during the process life time

  return RUN_ALL_TESTS();
}

