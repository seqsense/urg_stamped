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

#ifndef SCIP2_PROTOCOL_H
#define SCIP2_PROTOCOL_H

#include <map>
#include <memory>
#include <string>

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

#include <scip2/connection.h>
#include <scip2/response.h>
#include <scip2/logger.h>

namespace scip2
{
class Protocol
{
protected:
  Connection::Ptr connection_;
  ResponseProcessor response_processor_;

  void cbReceive(
      boost::asio::streambuf& buf,
      const boost::posix_time::ptime& time_read)
  {
    std::istream stream(&buf);

    std::string echo_back;
    if (!std::getline(stream, echo_back))
    {
      logger::error() << "Failed to get echo back" << std::endl;
      return;
    }
    if (echo_back == "")
    {
      logger::debug() << "Empty response echo back " << std::endl;
      return;
    }

    std::string status;
    if (!std::getline(stream, status))
    {
      logger::error() << "Failed to get status" << std::endl;
      return;
    }
    if (status == "")
    {
      logger::debug() << "Empty response status" << std::endl;
      return;
    }
    status.pop_back();  // remove checksum

    response_processor_(time_read, echo_back, status, stream);
  }

public:
  using Ptr = std::shared_ptr<Protocol>;

  explicit Protocol(Connection::Ptr connection)
    : connection_(connection)
  {
    connection_->registerReceiveCallback(
        boost::bind(&scip2::Protocol::cbReceive, this, boost::arg<1>(), boost::arg<2>()));
  }

  void sendCommand(
      const std::string& command,
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

#endif  // SCIP2_PROTOCOL_H
