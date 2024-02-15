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

#include <list>
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
    double response_delay_base;
    double response_delay_sigma;
    double scan_interval;
    double clock_rate;
  };

  inline URGSimulator(
      const boost::asio::ip::tcp::endpoint& endpoint,
      const URGSimulator::Params& params)
    : params_(params)
    , io_service_()
    , acceptor_(io_service_, endpoint)
    , socket_(io_service_)
    , input_process_timer_(io_service_)
    , rand_engine_(std::random_device()())
    , comm_delay_distribution_(
          params.comm_delay_base, params.comm_delay_sigma)
  {
  }

  inline boost::asio::ip::tcp::endpoint getLocalEndpoint() const
  {
    return acceptor_.local_endpoint();
  }

  void spin();

private:
  const URGSimulator::Params params_;

  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::acceptor acceptor_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::streambuf input_buf_;
  boost::asio::deadline_timer input_process_timer_;

  std::default_random_engine rand_engine_;
  std::normal_distribution<double> comm_delay_distribution_;

  void onRead(const boost::system::error_code& error);
  void processInput(
      const std::string line,
      const boost::system::error_code& error);
  void asyncRead();
};

}  // namespace urg_sim

#endif  // URG_SIM_URG_SIM_H
