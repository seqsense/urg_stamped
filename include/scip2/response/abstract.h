/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#ifndef SCIP2_RESPONSE_ABSTRACT_H
#define SCIP2_RESPONSE_ABSTRACT_H

#include <boost/asio.hpp>

#include <string>

namespace scip2
{
class Response
{
public:
  using Ptr = std::shared_ptr<Response>;
  virtual std::string getCommandCode() const = 0;
  virtual void operator()(
      const boost::posix_time::ptime &,
      const std::string &,
      const std::string &,
      std::istream &) = 0;
};

}  // namespace scip2

#endif  // SCIP2_RESPONSE_ABSTRACT_H
