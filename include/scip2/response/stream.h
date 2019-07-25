/*
 * Copyright 2018 The urg_stamped Authors
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

#ifndef SCIP2_RESPONSE_STREAM_H
#define SCIP2_RESPONSE_STREAM_H

#include <boost/asio.hpp>

#include <map>
#include <string>
#include <vector>

#include <scip2/decode.h>
#include <scip2/response/abstract.h>
#include <scip2/logger.h>

namespace scip2
{
class ScanData
{
public:
  uint32_t timestamp_;
  std::vector<int32_t> ranges_;
  std::vector<int32_t> intensities_;
};

class ResponseStream : public Response
{
public:
  using Callback = boost::function<void(
      const boost::posix_time::ptime &,
      const std::string &,
      const std::string &,
      const ScanData &)>;

protected:
  Callback cb_;

public:
  virtual std::string getCommandCode() const = 0;
  virtual void operator()(
      const boost::posix_time::ptime &,
      const std::string &,
      const std::string &,
      std::istream &) = 0;

  bool readTimestamp(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      std::istream &stream,
      ScanData &scan)
  {
    if (status == "00")
    {
      return false;
    }
    if (status != "99")
    {
      if (cb_)
        cb_(time_read, echo_back, status, scan);
      logger::error() << echo_back << " errored with " << status << std::endl;
      return false;
    }
    std::string stamp;
    if (!std::getline(stream, stamp))
    {
      logger::error() << "Failed to get timestamp" << std::endl;
      return false;
    }
    const uint8_t checksum = stamp.back();
    stamp.pop_back();  // remove checksum
    if (stamp.size() < 4)
    {
      logger::error() << "Wrong timestamp format" << std::endl;
      return false;
    }

    auto dec = Decoder<4>(stamp);
    auto it = dec.begin();
    scan.timestamp_ = *it;
    if ((dec.getChecksum() & 0x3F) + 0x30 != checksum)
    {
      logger::error() << "Checksum mismatch" << std::endl;
      return false;
    }
    return true;
  }
  void registerCallback(Callback cb)
  {
    cb_ = cb;
  }
};

class ResponseMD : public ResponseStream
{
public:
  std::string getCommandCode() const
  {
    return std::string("MD");
  }
  void operator()(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      std::istream &stream) override
  {
    ScanData scan;
    if (!readTimestamp(time_read, echo_back, status, stream, scan))
      return;
    scan.ranges_.reserve(512);

    std::string line;
    scip2::DecoderRemain remain;
    while (std::getline(stream, line))
    {
      if (line.size() == 0)
        break;

      const uint8_t checksum = line.back();
      line.pop_back();  // remove checksum
      if (line.size() < 3)
      {
        logger::error() << "Wrong stream format" << std::endl;
        return;
      }
      auto dec = Decoder<3>(line, remain);
      auto it = dec.begin();
      for (; it != dec.end(); ++it)
      {
        scan.ranges_.push_back(*it);
      }
      remain = it.getRemain();
      if ((dec.getChecksum() & 0x3F) + 0x30 != checksum)
      {
        logger::error() << "Checksum mismatch; scan dropped" << std::endl
                        << line << std::endl;
        return;
      }
    }
    if (cb_)
      cb_(time_read, echo_back, status, scan);
  }
};

class ResponseME : public ResponseStream
{
public:
  std::string getCommandCode() const
  {
    return std::string("ME");
  }
  void operator()(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      std::istream &stream) override
  {
    ScanData scan;
    if (!readTimestamp(time_read, echo_back, status, stream, scan))
      return;
    scan.ranges_.reserve(512);
    scan.intensities_.reserve(512);

    std::string line;
    scip2::DecoderRemain remain;
    while (std::getline(stream, line))
    {
      if (line.size() == 0)
        break;

      const uint8_t checksum = line.back();
      line.pop_back();  // remove checksum
      if (line.size() < 3)
      {
        logger::error() << "Wrong stream format" << std::endl;
        return;
      }
      auto dec = Decoder<6>(line, remain);
      auto it = dec.begin();
      for (; it != dec.end(); ++it)
      {
        scan.ranges_.push_back((*it) >> 18);
        scan.intensities_.push_back((*it) & 0x3FFFF);
      }
      remain = it.getRemain();
      if ((dec.getChecksum() & 0x3F) + 0x30 != checksum)
      {
        logger::error() << "Checksum mismatch; scan dropped" << std::endl
                        << line << std::endl;
        return;
      }
    }
    if (cb_)
      cb_(time_read, echo_back, status, scan);
  }
};

}  // namespace scip2

#endif  // SCIP2_RESPONSE_STREAM_H
