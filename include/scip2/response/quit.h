/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#ifndef SCIP2_RESPONSE_QUIT_H
#define SCIP2_RESPONSE_QUIT_H

#include <boost/asio.hpp>

#include <string>
#include <map>

#include <scip2/response/abstract.h>

namespace scip2
{
class ResponseQT : public Response
{
public:
  using Callback = boost::function<void(
      const boost::posix_time::ptime &,
      const std::string &,
      const std::string &)>;

protected:
  Callback cb_;

public:
  std::string getCommandCode() const
  {
    return std::string("QT");
  }
  void operator()(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      std::istream &stream)
  {
    if (cb_)
      cb_(time_read, echo_back, status);
  }
  void registerCallback(Callback cb)
  {
    cb_ = cb;
  }
};

}  // namespace scip2

#endif  // SCIP2_RESPONSE_QUIT_H
