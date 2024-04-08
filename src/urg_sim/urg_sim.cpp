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
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/asio/error.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/move/move.hpp>
#include <boost/system/error_code.hpp>

#include <urg_sim/urg_sim.h>
#include <urg_sim/encode.h>

namespace urg_sim
{

namespace
{
// Common status code across the commands
const char* status_ok = "00";
const char* status_already = "02";
const char* status_error_command_not_defined = "0E";
const char* status_error_abnormal = "0L";
const char* status_error_denied = "10";
const char* status_error_command_short = "0C";
const char* status_error_command_long = "0D";
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
    input_fifo_.post(boost::bind(&URGSimulator::processInput, this, line, when));
  }

  asyncRead();
}

void URGSimulator::processInput(
    const std::string cmd,
    const boost::posix_time::ptime& when)
{
  // Find handler from command string
  const std::string op = cmd.substr(0, 2);
  const auto it_h = handlers_.find(op);
  const auto h =
      (it_h != handlers_.end()) ?
          it_h->second :
          std::bind(&URGSimulator::handleUnknown, this, std::placeholders::_1);

  boost::asio::deadline_timer wait(io_service_);
  wait.expires_at(when);
  wait.wait();

  h(cmd);
}

void URGSimulator::handleII(const std::string cmd)
{
  if (!validateExtraString(cmd, 2))
  {
    return;
  }
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
  std::string lasr;
  {
    std::lock_guard<std::mutex> lock(mu_);

    switch (params_.model)
    {
      case Model::UTM:
        switch (sensor_state_)
        {
          case SensorState::BOOTING:
            mesm = "001 Booting";
            break;
          case SensorState::SINGLE_SCAN:
            mesm = "003 Single_scan";
            break;
          case SensorState::MULTI_SCAN:
            mesm = "004 Multi_scan";
            break;
          default:
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
    switch (sensor_state_)
    {
      case SensorState::SINGLE_SCAN:
      case SensorState::MULTI_SCAN:
        lasr = "ON";
        break;
      default:
        lasr = "OFF";
        break;
    }
  }

  const KeyValues kvs =
      {
          {"MODL", model_name_},
          {"LASR", lasr},
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
  if (!validateExtraString(cmd, 2))
  {
    return;
  }
  const KeyValues kvs =
      {
          {"VEND", "Hokuyo Automatic Co., Ltd."},
          {"PROD", model_name_},
          {"FIRM", "1.1.0 (2011-09-30)"},
          {"PROT", "SCIP 2.2"},
          {"SERI", "H0123456"},
      };
  responseKeyValues(cmd, status_ok, kvs);
}

void URGSimulator::handlePP(const std::string cmd)
{
  if (!validateExtraString(cmd, 2))
  {
    return;
  }
  const int32_t rpm =
      static_cast<int32_t>(60.0 / params_.scan_interval);

  const KeyValues kvs =
      {
          {"MODL", model_name_},
          {"DMIN", "23"},
          {"DMAX", "60000"},
          {"PROT", "SCIP 2.2"},
          {"ARES", std::to_string(params_.angle_resolution)},
          {"AMIN", std::to_string(params_.angle_min)},
          {"AMAX", std::to_string(params_.angle_max)},
          {"AFRT", std::to_string(params_.angle_front)},
          {"SCAN", std::to_string(rpm)},
      };
  responseKeyValues(cmd, status_ok, kvs);
}

void URGSimulator::handleTM(const std::string cmd)
{
  if (!validateExtraString(cmd, 3))
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mu_);

  if (sensor_state_ == SensorState::ERROR_DETECTED)
  {
    response(cmd, status_error_abnormal);
    return;
  }

  switch (cmd[2])
  {
    case '0':
      switch (sensor_state_)
      {
        case SensorState::IDLE:
        case SensorState::SINGLE_SCAN:
          response(cmd, status_ok);
          sensor_state_ = SensorState::TIME_ADJUSTMENT;
          return;
        case SensorState::TIME_ADJUSTMENT:
          response(cmd, "02");
          return;
        default:
          response(cmd, status_error_denied);
          return;
      }
      break;
    case '1':
      // Actual sensors return timestamp even without entering TIME_ADJUSTMENT state.
      break;
    case '2':
      switch (sensor_state_)
      {
        case SensorState::TIME_ADJUSTMENT:
          response(cmd, status_ok);
          sensor_state_ = SensorState::IDLE;
          return;
        case SensorState::IDLE:
          response(cmd, "03");
          return;
        default:
          response(cmd, status_error_denied);
          return;
      }
      break;
    default:
      response(cmd, "01");
      return;
  }
  const uint32_t stamp = timestamp();
  const std::string time = encode::encode(
      std::vector<uint32_t>(1, stamp), encode::EncodeType::CED4);
  response(cmd, status_ok, encode::withChecksum(time) + "\n");
}

void URGSimulator::handleBM(const std::string cmd)
{
  if (!validateExtraString(cmd, 2))
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mu_);

  switch (sensor_state_)
  {
    case SensorState::ERROR_DETECTED:
      response(cmd, status_error_abnormal);
      return;
    case SensorState::IDLE:
      sensor_state_ = SensorState::SINGLE_SCAN;
      response(cmd, status_ok);
      return;
    case SensorState::SINGLE_SCAN:
    case SensorState::MULTI_SCAN:
      response(cmd, status_already);
      return;
    default:
      response(cmd, status_error_denied);
      return;
  }
}

void URGSimulator::handleQT(const std::string cmd)
{
  if (!validateExtraString(cmd, 2))
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mu_);

  switch (sensor_state_)
  {
    case SensorState::ERROR_DETECTED:
      response(cmd, status_error_abnormal);
      return;
    case SensorState::SINGLE_SCAN:
    case SensorState::MULTI_SCAN:
    case SensorState::IDLE:
      sensor_state_ = SensorState::IDLE;
      response(cmd, status_ok);
      return;
    default:
      response(cmd, status_error_denied);
      return;
  }
}

void URGSimulator::handleRS(const std::string cmd)
{
  if (!validateExtraString(cmd, 2))
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mu_);

  if (sensor_state_ == SensorState::ERROR_DETECTED)
  {
    response(cmd, status_error_abnormal);
    return;
  }
  timestamp_epoch_ = boost::posix_time::microsec_clock::universal_time();
  sensor_state_ = SensorState::IDLE;

  response(cmd, status_ok);
}

void URGSimulator::handleRB(const std::string cmd)
{
  if (!validateExtraString(cmd, 2))
  {
    return;
  }
  // Two RB commands within one second triggers sensor reboot
  const auto now = boost::posix_time::microsec_clock::universal_time();
  if (last_rb_ == boost::posix_time::not_a_date_time ||
      now - last_rb_ > boost::posix_time::seconds(1))
  {
    response(cmd, "01");
    last_rb_ = now;
    return;
  }
  response(cmd, status_ok);

  boot_timer_.expires_from_now(boost::posix_time::seconds(1));
  boot_timer_.async_wait(
      boost::bind(
          &URGSimulator::reboot,
          this));
}

void URGSimulator::handleMX(const std::string cmd)
{
  if (!validateExtraString(cmd, 15))
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mu_);

  try
  {
    measurement_start_step_ = std::stoi(cmd.substr(2, 4));
    measurement_end_step_ = std::stoi(cmd.substr(6, 4));
    measurement_grouping_step_ = std::stoi(cmd.substr(10, 2));
    measurement_skips_ = std::stoi(cmd.substr(12, 1));
    measurement_scans_ = std::stoi(cmd.substr(13, 2));
  }
  catch (const std::invalid_argument& e)
  {
    response(cmd, "02");
    return;
  }
  if (measurement_start_step_ > measurement_end_step_)
  {
    response(cmd, "05");
    return;
  }
  if (measurement_start_step_ < params_.angle_min ||
      measurement_end_step_ > params_.angle_max)
  {
    response(cmd, "04");
    return;
  }
  if (measurement_grouping_step_ == 0)
  {
    measurement_grouping_step_ = 1;
  }
  measurement_cnt_ = 0;
  measurement_sent_ = 0;
  measurement_cmd_ = cmd.substr(0, 13);
  measurement_extra_string_ = cmd.substr(15);

  sensor_state_ = SensorState::MULTI_SCAN;
  switch (cmd[1])
  {
    case 'D':
      measurement_mode_ = MeasurementMode::RANGE;
      break;
    case 'E':
      measurement_mode_ = MeasurementMode::RANGE_INTENSITY;
      break;
  }
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
  std::lock_guard<std::mutex> lock(mu_);

  if (sensor_state_ != SensorState::BOOTING &&
      sensor_state_ != SensorState::ERROR_DETECTED)
  {
    sensor_state_ = SensorState::IDLE;
  }
  socket_.close();
  accept();
}

void URGSimulator::reboot()
{
  std::cerr << "Booting" << std::endl;
  {
    std::lock_guard<std::mutex> lock(mu_);
    sensor_state_ = SensorState::BOOTING;
  }

  input_fifo_.stop();
  output_fifo_.stop();
  boot_timer_.cancel();
  scan_timer_.cancel();

  const auto delay = boost::posix_time::microseconds(
      static_cast<int64_t>(params_.boot_duration * 1e6));
  boot_timer_.expires_from_now(delay);
  boot_timer_.async_wait(
      boost::bind(
          &URGSimulator::booted,
          this));
  timestamp_epoch_ = boost::posix_time::microsec_clock::universal_time();

  if (socket_.is_open())
  {
    std::cerr << "Closing" << std::endl;
    socket_.close();
  }
  accept();
}

void URGSimulator::booted()
{
  std::cerr << "Booted" << std::endl;
  {
    std::lock_guard<std::mutex> lock(mu_);
    sensor_state_ = SensorState::IDLE;
    boot_cnt_++;
  }

  if (params_.model == Model::UST)
  {
    asyncRead();
  }

  next_scan_ = boost::posix_time::microsec_clock::universal_time();
  nextScan();
}

void URGSimulator::response(
    const std::string echo,
    const std::string status,
    const std::string data)
{
  const auto now = boost::posix_time::microsec_clock::universal_time();
  const double delay_sec = comm_delay_distribution_(rand_engine_);
  const auto delay = boost::posix_time::microseconds(
      static_cast<int64_t>(delay_sec * 1e6));
  const auto when = now + delay;

  const std::string text =
      echo + "\n" +
      encode::withChecksum(status) + "\n" +
      data + "\n";

  output_fifo_.post(boost::bind(&URGSimulator::send, this, text, when));
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
    const boost::posix_time::ptime& when)
{
  boost::asio::deadline_timer wait(io_service_);
  wait.expires_at(when);
  wait.wait();

  boost::system::error_code ec;
  boost::asio::write(socket_, boost::asio::buffer(data), ec);
  if (ec)
  {
    std::cerr << "Send failed: " << ec.message() << std::endl;
    return;
  }
}

uint32_t URGSimulator::timestamp(const boost::posix_time::ptime& now)
{
  const uint32_t diff = params_.clock_rate * (now - timestamp_epoch_).total_microseconds() / 1000.0;
  return diff & 0xFFFFFF;
}

void URGSimulator::accept()
{
  std::cerr << "Accepting" << std::endl;
  acceptor_.async_accept(
      socket_,
      boost::bind(
          &URGSimulator::accepted,
          this,
          boost::asio::placeholders::error));
}

void URGSimulator::accepted(
    const boost::system::error_code& ec)
{
  if (ec)
  {
    std::cerr << "Failed to accept: " << ec.message() << std::endl;
    accept();
    return;
  }
  std::cerr << "Accepted connection from "
            << socket_.remote_endpoint() << std::endl;
  socket_.set_option(boost::asio::ip::tcp::no_delay(true));

  {
    std::lock_guard<std::mutex> lock(mu_);

    if (params_.model == Model::UTM ||
        sensor_state_ != SensorState::BOOTING)
    {
      asyncRead();
    }
  }
}

void URGSimulator::nextScan()
{
  next_scan_ +=
      boost::posix_time::microseconds(
          static_cast<int64_t>(params_.scan_interval * 1e6));
  scan_timer_.expires_at(next_scan_);
  scan_timer_.async_wait(
      boost::bind(
          &URGSimulator::scan,
          this));
}

void URGSimulator::scan()
{
  {
    std::lock_guard<std::mutex> lock(mu_);

    if (sensor_state_ == SensorState::BOOTING)
    {
      return;
    }

    switch (sensor_state_)
    {
      case SensorState::SINGLE_SCAN:
      case SensorState::MULTI_SCAN:
        break;
      default:
        nextScan();
        return;
    }
  }

  const int num_points = params_.angle_max - params_.angle_min;
  RawScanData::Ptr raw_scan(new RawScanData);
  raw_scan->timestamp = timestamp(next_scan_);
  raw_scan->full_time = next_scan_;
  raw_scan->ranges.resize(num_points);
  raw_scan->intensities.resize(num_points);
  raw_scan_data_cb_(raw_scan);
  last_raw_scan_ = raw_scan;

  const double busy_seconds =
      params_.scan_interval *
      static_cast<double>(params_.angle_max) /
      static_cast<double>(params_.angle_resolution);
  boost::asio::deadline_timer scan_wait(io_service_);
  scan_wait.expires_at(
      next_scan_ +
      boost::posix_time::microseconds(
          static_cast<int64_t>(busy_seconds * 1e6)));
  scan_wait.wait();

  {
    std::lock_guard<std::mutex> lock(mu_);

    if (sensor_state_ == SensorState::MULTI_SCAN)
    {
      measurement_cnt_++;
      if (measurement_cnt_ > measurement_skips_)
      {
        sendScan();
        measurement_cnt_ = 0;
        measurement_sent_++;
        if (measurement_scans_ != 0 &&
            measurement_sent_ >= measurement_scans_)
        {
          sensor_state_ = SensorState::IDLE;
        }
        else if (measurement_scans_ == 0 &&
                 measurement_sent_ >= 100)
        {
          measurement_sent_ = 0;
        }
      }
    }
  }

  nextScan();
}

void URGSimulator::sendScan()
{
  std::vector<uint32_t> data;
  for (int i = measurement_start_step_; i <= measurement_end_step_; i += measurement_grouping_step_)
  {
    data.push_back(last_raw_scan_->ranges[i]);
    if (measurement_mode_ == MeasurementMode::RANGE_INTENSITY)
    {
      data.push_back(last_raw_scan_->intensities[i]);
    }
  }
  std::stringstream ss;

  const std::string time = encode::encode(
      std::vector<uint32_t>(1, last_raw_scan_->timestamp),
      encode::EncodeType::CED4);
  ss << encode::withChecksum(time) + "\n";

  const std::string encoded = encode::encode(data, encode::EncodeType::CED3);
  for (size_t i = 0; i < encoded.size(); i += 64)
  {
    const std::string line = encoded.substr(i, 64);
    ss << encode::withChecksum(line) << "\n";
  }

  std::stringstream ss_echo;
  ss_echo << measurement_cmd_;
  if (measurement_scans_ == 0)
  {
    ss_echo << "00";
  }
  else
  {
    ss_echo
        << std::setfill('0') << std::setw(2)
        << measurement_scans_ - measurement_sent_ - 1;
  }
  ss_echo << measurement_extra_string_;
  response(ss_echo.str(), "99", ss.str());
}

bool URGSimulator::validateExtraString(
    const std::string& cmd,
    const size_t expected_size)
{
  if (cmd.size() < expected_size)
  {
    response(cmd, status_error_command_short);
    return false;
  }
  if (cmd.size() == expected_size)
  {
    return true;
  }
  if (cmd[expected_size] != ';')
  {
    response(cmd, status_error_command_long);
    return false;
  }
  return true;
}

void URGSimulator::spin()
{
  reboot();

  std::thread th_input_queue(
      std::bind(&URGSimulator::fifo, this, std::ref(input_fifo_)));
  std::thread th_output_queue(
      std::bind(&URGSimulator::fifo, this, std::ref(output_fifo_)));

  while (!killed_)
  {
    io_service_.run();
  }

  th_input_queue.join();
  th_output_queue.join();
}

void URGSimulator::fifo(boost::asio::io_service& fifo)
{
  boost::asio::deadline_timer keepalive_timer(fifo);
  std::function<void(const boost::system::error_code& ec)> keepalive;

  keepalive = [&keepalive_timer, &keepalive](const boost::system::error_code& ec)
  {
    if (ec == boost::asio::error::operation_aborted)
    {
      return;
    }
    if (ec)
    {
      std::cerr << "fifo keepalive error: " << ec << std::endl;
      return;
    }
    keepalive_timer.expires_from_now(boost::posix_time::hours(1));
    keepalive_timer.async_wait(keepalive);
  };

  while (!killed_)
  {
    keepalive(boost::system::error_code());
    fifo.run();
    fifo.reset();
  }
}

void URGSimulator::kill()
{
  killed_ = true;
  io_service_.stop();
  input_fifo_.stop();
  output_fifo_.stop();
}

void URGSimulator::setState(const SensorState s)
{
  std::lock_guard<std::mutex> lock(mu_);
  sensor_state_ = s;
}

int URGSimulator::getBootCnt()
{
  std::lock_guard<std::mutex> lock(mu_);
  return boot_cnt_;
}

}  // namespace urg_sim
