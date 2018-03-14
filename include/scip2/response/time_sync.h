/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#ifndef SCIP2_RESPONSE_TIME_SYNC_H
#define SCIP2_RESPONSE_TIME_SYNC_H

#include <boost/asio.hpp>

#include <string>
#include <map>

#include <scip2/response/abstract.h>

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
      const boost::posix_time::ptime &,
      const std::string &,
      const std::string &,
      const Timestamp &)>;

protected:
  Callback cb_;

public:
  std::string getCommandCode() const
  {
    return std::string("TM");
  }
  void operator()(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      std::istream &stream)
  {
    Timestamp timestamp;
    if (status != "00")
    {
      if (cb_)
        cb_(time_read, echo_back, status, timestamp);
      std::cout << echo_back << " errored with " << status << std::endl;
      return;
    }
    if (echo_back[2] == '1')
    {
      std::string stamp;
      if (!std::getline(stream, stamp))
      {
        std::cerr << "Failed to get timestamp" << std::endl;
        return;
      }
      const uint8_t checksum = stamp.back();
      stamp.pop_back();  // remove checksum
      if (stamp.size() < 4)
      {
        std::cerr << "Wrong timestamp format" << std::endl;
        return;
      }

      auto dec = Decoder<4>(stamp);
      auto it = dec.begin();
      timestamp.timestamp_ = *it;
      if ((dec.getChecksum() & 0x3F) + 0x30 != checksum)
      {
        std::cerr << "Checksum mismatch" << std::endl;
        return;
      }
    }
    if (cb_)
      cb_(time_read, echo_back, status, timestamp);
  }
  void registerCallback(Callback cb)
  {
    cb_ = cb;
  }
};

}  // namespace scip2

#endif  // SCIP2_RESPONSE_TIME_SYNC_H
