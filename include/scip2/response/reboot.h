/*
 * Copyright 2020 The urg_stamped Authors
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

#ifndef SCIP2_RESPONSE_REBOOT_H
#define SCIP2_RESPONSE_REBOOT_H

#include <boost/asio.hpp>

#include <string>

#include <scip2/response/abstract.h>

namespace scip2
{
class ResponseRB : public Response
{
public:
  using Callback = boost::function<void(
      const boost::posix_time::ptime&,
      const std::string&,
      const std::string&)>;

protected:
  Callback cb_;

public:
  std::string getCommandCode() const
  {
    return std::string("RB");
  }
  void operator()(
      const boost::posix_time::ptime& time_read,
      const std::string& echo_back,
      const std::string& status,
      std::istream& stream)
  {
    if (cb_)
    {
      cb_(time_read, echo_back, status);
    }
    readUntilEnd(stream);
  }
  void registerCallback(Callback cb)
  {
    cb_ = cb;
  }
};

}  // namespace scip2

#endif  // SCIP2_RESPONSE_REBOOT_H
