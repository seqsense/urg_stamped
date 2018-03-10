/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#ifndef SCIP_RESPONSE_ABSTRACT_H
#define SCIP_RESPONSE_ABSTRACT_H

#include <boost/asio.hpp>

namespace scip
{
class Response
{
public:
  using Ptr = std::shared_ptr<Response>;
  virtual std::string getCommandCode() const = 0;
  virtual void operator()(
      const std::string &echo_back,
      const std::string &status,
      std::istream &stream) = 0;
};

}  // namespace scip

#endif  // RESPONSE_H
