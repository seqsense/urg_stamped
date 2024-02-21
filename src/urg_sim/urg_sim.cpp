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
#include <sstream>
#include <string>
#include <utility>
#include <vector>

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
const char* status_ok = "00";
const char* status_already = "02";
const char* status_error_command_not_defined = "0E";
const char* status_error_abnormal = "0L";
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
  if (ec)
  {
    std::cerr << "Connection closed: " << ec.message() << std::endl;
    handleDisconnect();
    return;
  }
  const auto now = boost::posix_time::microsec_clock::universal_time();
  const double delay_sec = comm_delay_distribution_(rand_engine_);
  const auto delay = boost::posix_time::microseconds(
      static_cast<int64_t>(delay_sec * 1e6));
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
    const boost::system::error_code& ec)
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
  const uint32_t stamp = timestamp();
  std::string time;
  if (params_.hex_ii_timestamp)
  {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << std::hex << stamp;
    time = ss.str();
  }
  else
  {
    time = encode::encode(
        std::vector<uint32_t>(1, stamp), encode::EncodeType::CED4);
  }
  const int32_t rpm =
      static_cast<int32_t>(60.0 / params_.scan_interval);

  std::string mesm;
  std::string stat;
  switch (params_.model)
  {
    case Model::UTM:
      switch (sensor_state_)
      {
        case SensorState::BOOTING:
          mesm = "001 Booting";
          break;
        case SensorState::IDLE:
          mesm = "000 Idle";
          break;
      }
      stat = "Stable 000 no error.";
      break;
    case Model::UST:
      mesm = "Measuring by Sensitive Mode";
      stat = "sensor is working normally";
      break;
  }

  const KeyValues kvs =
      {
          {"MODL", "UTM-30LX-EW"},
          {"LASR", laser_ ? "ON" : "OFF"},
          {"SCSP", std::to_string(rpm)},
          {"MESM", mesm},
          {"SBPS", "Ethernet 100 [Mbps]"},
          {"TIME", time},
          {"STAT", stat},
      };
  responseKeyValues(cmd, status_ok, kvs);
}  // namespace urg_sim

void URGSimulator::handleVV(const std::string cmd)
{
  const KeyValues kvs =
      {
          {"VEND", "Hokuyo Automatic Co., Ltd."},
          {"PROD", "UTM-30LX-EW"},
          {"FIRM", "1.1.0 (2011-09-30)"},
          {"PROT", "SCIP 2.2"},
          {"SERI", "H0123456"},
      };
  responseKeyValues(cmd, status_ok, kvs);
}

void URGSimulator::handlePP(const std::string cmd)
{
  const int32_t rpm =
      static_cast<int32_t>(60.0 / params_.scan_interval);
  const KeyValues kvs =
      {
          {"MODL", "UTM-30LX-EW"},
          {"DMIN", "23"},
          {"DMAX", "60000"},
          {"PROT", "SCIP 2.2"},
          {"ARES", "1440"},
          {"AMIN", "0"},
          {"AMAX", "1080"},
          {"AFRT", "540"},
          {"SCAN", std::to_string(rpm)},
      };
  responseKeyValues(cmd, status_ok, kvs);
}

void URGSimulator::handleTM(const std::string cmd)
{
  if (sensor_state_ == SensorState::ERROR_DETECTED)
  {
    response(cmd, status_error_abnormal);
    return;
  }
  response(cmd, status_ok, "data\n");
}

void URGSimulator::handleBM(const std::string cmd)
{
  if (sensor_state_ == SensorState::ERROR_DETECTED)
  {
    response(cmd, status_error_abnormal);
    return;
  }
  if (laser_)
  {
    response(cmd, status_already);
    return;
  }
  laser_ = true;
  sensor_state_ == SensorState::SINGLE_SCAN;
  response(cmd, status_ok);
}

void URGSimulator::handleQT(const std::string cmd)
{
  if (sensor_state_ == SensorState::ERROR_DETECTED)
  {
    response(cmd, status_error_abnormal);
    return;
  }
  if (!laser_)
  {
    response(cmd, status_already);
    return;
  }
  laser_ = false;
  sensor_state_ == SensorState::IDLE;
  response(cmd, status_ok);
}

void URGSimulator::handleRS(const std::string cmd)
{
  if (sensor_state_ == SensorState::ERROR_DETECTED)
  {
    response(cmd, status_error_abnormal);
    return;
  }
  reset();
  response(cmd, status_ok);
}

void URGSimulator::handleUnknown(const std::string cmd)
{
  if (cmd == "")
  {
    return;
  }
  response(cmd, status_error_command_not_defined);
}

void URGSimulator::handleDisconnect()
{
  if (sensor_state_ != SensorState::BOOTING &&
      sensor_state_ != SensorState::ERROR_DETECTED)
  {
    sensor_state_ = SensorState::IDLE;
  }
  socket_.close();
}

void URGSimulator::reset()
{
  timestamp_epoch_ = boost::posix_time::microsec_clock::universal_time();
  laser_ = false;
  sensor_state_ == SensorState::IDLE;
}

void URGSimulator::reboot()
{
  if (socket_.is_open())
  {
    socket_.close();
  }
  sensor_state_ = SensorState::BOOTING;

  const auto delay = boost::posix_time::microseconds(
      static_cast<int64_t>(params_.boot_duration * 1e6));
  boot_timer_.expires_from_now(delay);
  boot_timer_.async_wait(
      boost::bind(
          &URGSimulator::booted,
          this));
  reset();
}

void URGSimulator::booted()
{
  sensor_state_ = SensorState::IDLE;
  if (params_.model == Model::UST)
  {
    asyncRead();
  }
}

void URGSimulator::response(
    const std::string echo,
    const std::string status,
    const std::string data)
{
  const double delay_sec = comm_delay_distribution_(rand_engine_);
  const auto delay = boost::posix_time::microseconds(
      static_cast<int64_t>(delay_sec * 1e6));

  const std::string text =
      echo + "\n" +
      encode::withChecksum(status) + "\n" +
      data + "\n";

  input_process_timer_.expires_from_now(delay);
  input_process_timer_.async_wait(
      boost::bind(
          &URGSimulator::send,
          this,
          text,
          boost::asio::placeholders::error));
}

void URGSimulator::responseKeyValues(
    const std::string echo,
    const std::string status,
    const std::vector<std::pair<std::string, std::string>> kvs)
{
  std::stringstream ss;
  for (const auto& kv : kvs)
  {
    const std::string line = kv.first + ":" + kv.second;
    ss << line << ";" << encode::checksum(line) << "\n";
  }
  response(echo, status, ss.str());
}

void URGSimulator::send(
    const std::string data,
    const boost::system::error_code& ec)
{
  boost::asio::write(socket_, boost::asio::buffer(data));
}

uint32_t URGSimulator::timestamp()
{
  const auto now = boost::posix_time::microsec_clock::universal_time();
  const uint32_t diff = (now - timestamp_epoch_).total_milliseconds();
  return diff & 0xFFFFFF;
}

void URGSimulator::spin()
{
  while (true)
  {
    acceptor_.accept(socket_);
    if (params_.model == Model::UTM)
    {
      asyncRead();
    }
    io_service_.run();
    io_service_.reset();
  }
}

}  // namespace urg_sim
