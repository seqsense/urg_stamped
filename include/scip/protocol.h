/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#ifndef SCIP_PROTOCOL_H
#define SCIP_PROTOCOL_H

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

#include <scip/connection.h>

namespace scip
{
class Protocol
{
protected:
  Connection::Ptr connection_;

  void cbReceive(boost::asio::streambuf &buf)
  {
  }

public:
  using Ptr = std::shared_ptr<Protocol>;

  Protocol(Connection::Ptr connection)
    : connection_(connection)
  {
    connection_->registerReceiveCallback(
        boost::bind(&scip::Protocol::cbReceive, this, _1));
  }

  void sendCommand(const std::string &command)
  {
    connection_->send(command + "\r\n");
  }
};

}  // namespace scip

#endif  // SCIP_PROTOCOL_H
