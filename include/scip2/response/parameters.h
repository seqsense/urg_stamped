/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#ifndef SCIP2_RESPONSE_PARAMETERS_H
#define SCIP2_RESPONSE_PARAMETERS_H

#include <boost/asio.hpp>

#include <string>
#include <map>

#include <scip2/response/abstract.h>

namespace scip2
{
class ResponseParams : public Response
{
public:
  using Callback = boost::function<void(
      const boost::posix_time::ptime &,
      const std::string &,
      const std::string &,
      const std::map<std::string, std::string> &)>;

protected:
  Callback cb_;

public:
  virtual std::string getCommandCode() const = 0;
  void operator()(
      const boost::posix_time::ptime &time_read,
      const std::string &echo_back,
      const std::string &status,
      std::istream &stream)
  {
    std::map<std::string, std::string> params;
    if (status != "00")
    {
      if (cb_)
        cb_(time_read, echo_back, status, params);
      std::cout << echo_back << " errored with " << status << std::endl;
      return;
    }
    std::string line;
    while (std::getline(stream, line))
    {
      if (line.size() == 0)
        break;
      const auto delm = std::find(line.begin(), line.end(), ':');
      if (delm == line.end())
      {
        std::cout << "Parameter decode error" << std::endl;
        return;
      }
      const auto end = std::find(line.begin(), line.end(), ';');
      if (end == line.end())
      {
        std::cout << "Parameter decode error" << std::endl;
        return;
      }
      const std::string key(line.begin(), delm);
      const std::string value(delm + 1, end);
      params[key] = value;
    }
    if (cb_)
      cb_(time_read, echo_back, status, params);
  }
  void registerCallback(Callback cb)
  {
    cb_ = cb;
  }
};

class ResponsePP : public ResponseParams
{
public:
  std::string getCommandCode() const
  {
    return std::string("PP");
  }
};

class ResponseVV : public ResponseParams
{
public:
  std::string getCommandCode() const
  {
    return std::string("VV");
  }
};

class ResponseII : public ResponseParams
{
public:
  std::string getCommandCode() const
  {
    return std::string("II");
  }
};

}  // namespace scip2

#endif  // SCIP2_RESPONSE_PARAMETERS_H
