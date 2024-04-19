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

#ifndef SCIP2_RESPONSE_ABSTRACT_H
#define SCIP2_RESPONSE_ABSTRACT_H

#include <memory>
#include <string>

#include <boost/asio.hpp>

namespace scip2
{
class Response
{
public:
  using Ptr = std::shared_ptr<Response>;
  virtual std::string getCommandCode() const = 0;
  virtual void operator()(
      const boost::posix_time::ptime&,
      const std::string&,
      const std::string&,
      std::istream&) = 0;
};

inline void readUntilEnd(std::istream& stream)
{
  std::string line;
  while (std::getline(stream, line) && line.size() > 0)
  {
  }
}

}  // namespace scip2

#endif  // SCIP2_RESPONSE_ABSTRACT_H
