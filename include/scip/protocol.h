/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#ifndef SCIP_PROTOCOL_H
#define SCIP_PROTOCOL_H

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

#include <scip/connection.h>
#include <scip/response.h>

#include <map>

namespace scip
{
class Protocol
{
protected:
  Connection::Ptr connection_;
  ResponseProcessor response_processor_;

  void cbReceive(
      boost::asio::streambuf &buf,
      const boost::chrono::system_clock::time_point &time_read)
  {
    std::istream stream(&buf);
    std::string echo_back;
    if (!std::getline(stream, echo_back))
    {
      std::cerr << "Failed to get echo back" << std::endl;
      return;
    }
    std::string status;
    if (!std::getline(stream, status))
    {
      std::cerr << "Failed to get status" << std::endl;
      return;
    }
    status.pop_back();  // remove checksum

    response_processor_(echo_back, status, stream);

    std::string line;
    while (std::getline(stream, line))
    {
    }
  }

public:
  using Ptr = std::shared_ptr<Protocol>;

  Protocol(Connection::Ptr connection)
    : connection_(connection)
  {
    connection_->registerReceiveCallback(
        boost::bind(&scip::Protocol::cbReceive, this, _1, _2));
  }

  void sendCommand(const std::string &command)
  {
    connection_->send(command + "\n");
  }

  template <typename TResponse>
  void registerCallback(typename TResponse::Callback cb)
  {
    response_processor_.registerCallback<TResponse>(cb);
  }
};

}  // namespace scip

#endif  // SCIP_PROTOCOL_H
