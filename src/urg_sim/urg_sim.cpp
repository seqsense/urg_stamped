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
#include <string>

#include <boost/asio/error.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/system/error_code.hpp>

#include <urg_sim/urg_sim.h>
#include <urg_sim/encode.h>

namespace urg_sim
{

namespace
{
const char* status_accepted = "00";
const char* status_error_command_not_defined = "0E";
}  // namespace

void URGSimulator::asyncRead()
{
  boost::asio::async_read_until(
      socket_, input_buf_, "\n",
      boost::bind(
          &URGSimulator::onRead, this, boost::asio::placeholders::error));
}

void URGSimulator::onRead(const boost::system::error_code& ec)
{
  if (ec == boost::asio::error::eof)
  {
    std::cerr << "Connection closed" << std::endl;
    io_service_.stop();
    return;
  }
  const auto now = boost::posix_time::microsec_clock::universal_time();
  const double delay_sec = comm_delay_distribution_(rand_engine_);
  const auto delay = boost::posix_time::microseconds(
      static_cast<long>(delay_sec * 1e6));
  const auto when = now + delay;

  std::istream stream(&input_buf_);
  std::string line;
  while (std::getline(stream, line))
  {
    std::cerr << "onRead " << line << std::endl;

    input_process_timer_.expires_at(when);
    input_process_timer_.async_wait(
        boost::bind(
            &URGSimulator::processInput,
            this,
            line,
            boost::asio::placeholders::error));
  }

  asyncRead();
}

void URGSimulator::processInput(
    const std::string cmd,
    const boost::system::error_code& error)
{
  std::cerr << "processInput " << cmd << std::endl;

  const std::string op = cmd.substr(0, 2);
  const auto it_h = handlers_.find(op);
  const auto h =
      (it_h != handlers_.end()) ?
          it_h->second :
          std::bind(&URGSimulator::handleUnknown, this, std::placeholders::_1);

  h(cmd);
}

void URGSimulator::handleII(const std::string cmd)
{
  send(cmd, status_accepted, "data\n");
}

void URGSimulator::handleVV(const std::string cmd)
{
  send(cmd, status_accepted, "data\n");
}

void URGSimulator::handlePP(const std::string cmd)
{
  send(cmd, status_accepted, "data\n");
}

void URGSimulator::handleTM(const std::string cmd)
{
  send(cmd, status_accepted, "data\n");
}

void URGSimulator::handleUnknown(const std::string cmd)
{
  send(cmd, status_error_command_not_defined, "");
}

void URGSimulator::reset()
{
  timestamp_origin_ = boost::posix_time::microsec_clock::universal_time();
}

void URGSimulator::send(
    const std::string echo,
    const std::string status,
    const std::string data)
{
  boost::asio::write(
      socket_,
      boost::asio::buffer(
          echo + "\n" +
          encode::withChecksum(status) + "\n" +
          data + "\n"));
}

void URGSimulator::spin()
{
  acceptor_.accept(socket_);
  asyncRead();
  io_service_.run();
}

}  // namespace urg_sim
