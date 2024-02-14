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

#include <chrono>
#include <iostream>

#include <boost/asio/error.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/system/error_code.hpp>

#include <urg_sim/urg_sim.h>

namespace urg_sim
{

namespace
{
const char* status_accepted = "00";
const char* status_error_command_not_defined = "0E";
}  // namespace

void URGSimulator::spin()
{
  acceptor_.accept(socket_);
  while (true)
  {
    boost::asio::streambuf buf;
    boost::system::error_code ec;
    boost::asio::read_until(socket_, buf, "\n\n", ec);
    if (ec == boost::asio::error::eof)
    {
      std::cerr << "Connection closed" << std::endl;
      break;
    }

    const auto now = std::chrono::steady_clock::now();
    const double delay_sec = comm_delay_distribution_(rand_engine_);
    const auto delay =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::ratio<1>>(delay_sec));

    const std::string line(
        boost::asio::buffer_cast<const char*>(buf.data()));
    input_queue_.emplace_back(line, now + delay);
  }
}

void URGSimulator::parseCommand(const std::string& line)
{
}

}  // namespace urg_sim
