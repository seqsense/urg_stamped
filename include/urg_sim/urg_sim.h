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

#ifndef URG_SIM_URG_SIM_H
#define URG_SIM_URG_SIM_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/streambuf.hpp>

namespace urg_sim
{

struct RawScanData
{
  uint32_t timestamp;
  boost::posix_time::ptime full_time;
  std::vector<uint32_t> ranges;
  std::vector<uint32_t> intensities;

  using Ptr = std::shared_ptr<RawScanData>;
};

using RawScanDataCallback = std::function<void(const RawScanData::Ptr)>;

namespace
{
void nopRawScanDataCallback(const RawScanData::Ptr)
{
}
}  // namespace

class URGSimulator
{
public:
  enum class Model
  {
    UTM,
    UST,
  };
  struct Params
  {
    Model model;
    double boot_duration;
    double comm_delay_base;
    double comm_delay_sigma;
    double scan_interval;
    double clock_rate;
    bool hex_ii_timestamp;
    int angle_resolution;
    int angle_min;
    int angle_max;
    int angle_front;
  };
  enum class SensorState
  {
    BOOTING,
    IDLE,
    SINGLE_SCAN,
    MULTI_SCAN,
    TIME_ADJUSTMENT,
    ERROR_DETECTED,
  };
  enum class MeasurementMode
  {
    RANGE,
    RANGE_INTENSITY,
  };

  inline URGSimulator(
      const boost::asio::ip::tcp::endpoint& endpoint,
      const URGSimulator::Params& params,
      const RawScanDataCallback raw_scan_data_cb = nopRawScanDataCallback)
    : params_(params)
    , acceptor_(io_service_, endpoint)
    , socket_(io_service_)
    , boot_timer_(io_service_)
    , scan_timer_(io_service_)
    , raw_scan_data_cb_(raw_scan_data_cb)
    , rand_engine_(std::random_device()())
    , comm_delay_distribution_(0, params.comm_delay_sigma)
    , killed_(false)
    , handlers_(
          {
              {"II", std::bind(&URGSimulator::handleII, this, std::placeholders::_1)},
              {"VV", std::bind(&URGSimulator::handleVV, this, std::placeholders::_1)},
              {"PP", std::bind(&URGSimulator::handlePP, this, std::placeholders::_1)},
              {"TM", std::bind(&URGSimulator::handleTM, this, std::placeholders::_1)},
              {"BM", std::bind(&URGSimulator::handleBM, this, std::placeholders::_1)},
              {"QT", std::bind(&URGSimulator::handleQT, this, std::placeholders::_1)},
              {"RS", std::bind(&URGSimulator::handleRS, this, std::placeholders::_1)},
              {"RT", std::bind(&URGSimulator::handleRS, this, std::placeholders::_1)},
              {"RB", std::bind(&URGSimulator::handleRB, this, std::placeholders::_1)},
              {"MD", std::bind(&URGSimulator::handleMX, this, std::placeholders::_1)},
              {"ME", std::bind(&URGSimulator::handleMX, this, std::placeholders::_1)},
          })  // NOLINT(whitespace/braces)
    , sensor_state_(SensorState::IDLE)
    , boot_cnt_(0)
  {
    switch (params_.model)
    {
      case Model::UTM:
        model_name_ = "UTM-30LX-EW";
        break;
      case Model::UST:
        model_name_ = "UST-30LC";
        break;
    }
  }

  inline boost::asio::ip::tcp::endpoint getLocalEndpoint() const
  {
    return acceptor_.local_endpoint();
  }

  void spin();
  void kill();
  void setState(const SensorState s);
  int getBootCnt();

private:
  using KeyValue = std::pair<std::string, std::string>;
  using KeyValues = std::vector<KeyValue>;

  const URGSimulator::Params params_;
  std::string model_name_;

  boost::asio::io_service io_service_;
  boost::asio::io_service input_fifo_;
  boost::asio::io_service output_fifo_;
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::streambuf input_buf_;
  boost::asio::deadline_timer boot_timer_;
  boost::asio::deadline_timer scan_timer_;
  std::mutex mu_;  // Mutex for sensor_state_ and boot_cnt_ for access from CI thread

  RawScanDataCallback raw_scan_data_cb_;

  std::default_random_engine rand_engine_;
  std::normal_distribution<double> comm_delay_distribution_;

  std::atomic<bool> killed_;
  boost::posix_time::ptime timestamp_epoch_;
  std::map<std::string, std::function<void(const std::string)>> handlers_;
  SensorState sensor_state_;
  boost::posix_time::ptime last_rb_;
  RawScanData::Ptr last_raw_scan_;
  boost::posix_time::ptime next_scan_;
  MeasurementMode measurement_mode_;
  int measurement_start_step_;
  int measurement_end_step_;
  int measurement_grouping_step_;
  int measurement_skips_;
  int measurement_scans_;
  int measurement_cnt_;
  int measurement_sent_;
  std::string measurement_cmd_;
  std::string measurement_extra_string_;
  int boot_cnt_;

  void onRead(const boost::system::error_code& ec);
  void processInput(
      const std::string cmd,
      const boost::posix_time::ptime& when);
  void asyncRead();
  void reset();
  void reboot();
  void booted();
  void response(
      const std::string echo,
      const std::string status,
      const std::string data = "");
  void responseKeyValues(
      const std::string echo,
      const std::string status,
      const KeyValues kv);
  void send(
      const std::string data,
      const boost::posix_time::ptime& when);
  void accept();
  void accepted(
      const boost::system::error_code& ec);
  void nextScan();
  void scan();
  void sendScan();
  void fifo(boost::asio::io_service& fifo);

  void handleII(const std::string cmd);
  void handleVV(const std::string cmd);
  void handlePP(const std::string cmd);
  void handleTM(const std::string cmd);
  void handleBM(const std::string cmd);
  void handleQT(const std::string cmd);
  void handleRS(const std::string cmd);
  void handleRB(const std::string cmd);
  void handleMX(const std::string cmd);
  void handleUnknown(const std::string cmd);
  void handleDisconnect();

  uint32_t timestamp(
      const boost::posix_time::ptime& now = boost::posix_time::microsec_clock::universal_time());
  bool validateExtraString(
      const std::string& cmd,
      const size_t expected_size);
  double randomCommDelay();
};

}  // namespace urg_sim

#endif  // URG_SIM_URG_SIM_H
