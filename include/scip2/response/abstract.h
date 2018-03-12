/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#ifndef SCIP_RESPONSE_ABSTRACT_H
#define SCIP_RESPONSE_ABSTRACT_H

#include <boost/asio.hpp>

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

#endif  // RESPONSE_H
