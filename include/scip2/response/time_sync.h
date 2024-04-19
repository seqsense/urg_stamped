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

#ifndef SCIP2_RESPONSE_TIME_SYNC_H
#define SCIP2_RESPONSE_TIME_SYNC_H

#include <boost/asio.hpp>

#include <string>
#include <map>

#include <scip2/response/abstract.h>
#include <scip2/logger.h>

namespace scip2
{
class Timestamp
{
public:
  uint32_t timestamp_;

  Timestamp()
    : timestamp_(0)
  {
  }
};

class ResponseTM : public Response
{
public:
  using Callback = boost::function<void(
      const boost::posix_time::ptime&,
      const std::string&,
      const std::string&,
      const Timestamp&)>;

protected:
  Callback cb_;

public:
  std::string getCommandCode() const
  {
    return std::string("TM");
  }
  void operator()(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status,
      std::istream& stream)
  {
    Timestamp timestamp;
    if (status != "00")
    {
      if (cb_)
      {
        cb_(time_read, echo_back, status, timestamp);
      }
      readUntilEnd(stream);
      return;
    }
    if (echo_back[2] == '1')
    {
      std::string stamp;
      if (!std::getline(stream, stamp))
      {
        logger::error() << "Failed to get timestamp" << std::endl;
        readUntilEnd(stream);
        return;
      }
      const uint8_t checksum = stamp.back();
      stamp.pop_back();  // remove checksum
      if (stamp.size() < 4)
      {
        logger::error() << "Wrong timestamp format" << std::endl;
        readUntilEnd(stream);
        return;
      }

      auto dec = Decoder<4>(stamp);
      auto it = dec.begin();
      timestamp.timestamp_ = *it;
      if ((dec.getChecksum() & 0x3F) + 0x30 != checksum)
      {
        logger::error() << "Checksum mismatch" << std::endl;
        readUntilEnd(stream);
        return;
      }
    }
    if (cb_)
    {
      cb_(time_read, echo_back, status, timestamp);
    }
    readUntilEnd(stream);
  }
  void registerCallback(Callback cb)
  {
    cb_ = cb;
  }
};

}  // namespace scip2

#endif  // SCIP2_RESPONSE_TIME_SYNC_H
