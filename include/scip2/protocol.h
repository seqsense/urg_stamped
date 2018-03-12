/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#ifndef SCIP_PROTOCOL_H
#define SCIP_PROTOCOL_H

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

#include <scip2/connection.h>
#include <scip2/response.h>

#include <map>

namespace scip2
{
class Protocol
{
protected:
  Connection::Ptr connection_;
  ResponseProcessor response_processor_;

  void cbReceive(
      boost::asio::streambuf &buf,
      const boost::posix_time::ptime &time_read)
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

    response_processor_(time_read, echo_back, status, stream);

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
        boost::bind(&scip2::Protocol::cbReceive, this, _1, _2));
  }

  void sendCommand(
      const std::string &command,
      Connection::CallbackSend cb = Connection::CallbackSend())
  {
    connection_->send(command + "\n", cb);
  }

  template <typename TResponse>
  void registerCallback(typename TResponse::Callback cb)
  {
    response_processor_.registerCallback<TResponse>(cb);
  }
};

}  // namespace scip2

#endif  // SCIP_PROTOCOL_H
