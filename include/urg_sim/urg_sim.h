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

#include <functional>
#include <list>
#include <map>
#include <random>

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/streambuf.hpp>

namespace urg_sim
{

class URGSimulator
{
public:
  struct Params
  {
    double comm_delay_base;
    double comm_delay_sigma;
    double scan_interval;
    double clock_rate;
    bool hex_ii_timestamp;
  };
  enum SensorState
  {
    BOOTING,
    IDLE,
    SINGLE_SCAN,
    MULTI_SCAN,
    TIME_ADJUSTMENT,
    ERROR_DETECTED,
  };

  inline URGSimulator(
      const boost::asio::ip::tcp::endpoint& endpoint,
      const URGSimulator::Params& params)
    : params_(params)
    , io_service_()
    , acceptor_(io_service_, endpoint)
    , socket_(io_service_)
    , input_process_timer_(io_service_)
    , output_process_timer_(io_service_)
    , rand_engine_(std::random_device()())
    , comm_delay_distribution_(
          params.comm_delay_base, params.comm_delay_sigma)
    , handlers_({
          {"II", std::bind(&URGSimulator::handleII, this, std::placeholders::_1)},
          {"VV", std::bind(&URGSimulator::handleVV, this, std::placeholders::_1)},
          {"PP", std::bind(&URGSimulator::handlePP, this, std::placeholders::_1)},
          {"TM", std::bind(&URGSimulator::handleTM, this, std::placeholders::_1)},
          {"BM", std::bind(&URGSimulator::handleBM, this, std::placeholders::_1)},
          {"QT", std::bind(&URGSimulator::handleQT, this, std::placeholders::_1)},
          {"RS", std::bind(&URGSimulator::handleRS, this, std::placeholders::_1)},
          {"RT", std::bind(&URGSimulator::handleRS, this, std::placeholders::_1)},
      })
    , laser_(false)
    , sensor_state_(SensorState::IDLE)
  {
    reset();
  }

  inline boost::asio::ip::tcp::endpoint getLocalEndpoint() const
  {
    return acceptor_.local_endpoint();
  }

  void spin();

private:
  using KeyValue = std::pair<std::string, std::string>;
  using KeyValues = std::vector<KeyValue>;

  const URGSimulator::Params params_;

  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::streambuf input_buf_;
  boost::asio::deadline_timer input_process_timer_;
  boost::asio::deadline_timer output_process_timer_;

  std::default_random_engine rand_engine_;
  std::normal_distribution<double> comm_delay_distribution_;

  boost::posix_time::ptime timestamp_epoch_;
  std::map<std::string, std::function<void(const std::string)>> handlers_;
  bool laser_;
  SensorState sensor_state_;

  void onRead(const boost::system::error_code& ec);
  void processInput(
      const std::string cmd,
      const boost::system::error_code& ec);
  void asyncRead();
  void reset();
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
      const boost::system::error_code& ec);

  void handleII(const std::string cmd);
  void handleVV(const std::string cmd);
  void handlePP(const std::string cmd);
  void handleTM(const std::string cmd);
  void handleBM(const std::string cmd);
  void handleQT(const std::string cmd);
  void handleRS(const std::string cmd);
  void handleUnknown(const std::string cmd);

  uint32_t timestamp();
};

}  // namespace urg_sim

#endif  // URG_SIM_URG_SIM_H
