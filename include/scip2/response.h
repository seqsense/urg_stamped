/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#ifndef SCIP2_RESPONSE_H
#define SCIP2_RESPONSE_H

#include <boost/asio.hpp>

#include <map>
#include <string>

#include <scip2/response/abstract.h>
#include <scip2/response/parameters.h>
#include <scip2/response/stream.h>
#include <scip2/response/time_sync.h>

namespace scip2
{
class ResponseProcessor
{
protected:
  std::map<std::string, Response::Ptr> responses_;
  void registerResponse(Response::Ptr response)
  {
    responses_[response->getCommandCode()] = response;
  }

public:
  ResponseProcessor()
  {
    registerResponse(Response::Ptr(new ResponsePP));
    registerResponse(Response::Ptr(new ResponseVV));
    registerResponse(Response::Ptr(new ResponseII));
    registerResponse(Response::Ptr(new ResponseMD));
    registerResponse(Response::Ptr(new ResponseME));
    registerResponse(Response::Ptr(new ResponseTM));
  }
  void operator()(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      std::istream &stream) const
  {
    const std::string command_code(echo_back.substr(0, 2));
    const auto response = responses_.find(command_code);
    if (response == responses_.end())
    {
      std::cerr << "Unknown response " << command_code << std::endl;
      return;
    }
    (*(response->second))(time_read, echo_back, status, stream);
  }
  template <typename TResponse>
  void registerCallback(typename TResponse::Callback cb)
  {
    const auto response = responses_.find(TResponse().getCommandCode());
    assert(response != responses_.end());
    auto response_downcast = std::dynamic_pointer_cast<TResponse>(response->second);
    assert(response_downcast);

    response_downcast->registerCallback(cb);
  }
};

}  // namespace scip2

#endif  // SCIP2_RESPONSE_H
