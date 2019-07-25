/*
 * Copyright 2019 The urg_stamped Authors
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

#include <iostream>
#include <sstream>
#include <string>

#include <scip2/logger.h>

namespace scip2
{
namespace logger
{
namespace
{
class DefaultStreamBuffer : public std::stringbuf
{
private:
  bool active_;
  std::ostream *raw_;
  std::string type_;

public:
  DefaultStreamBuffer(std::ostream *raw, const char *type)
    : raw_(raw)
    , type_(type)
  {
  }
  virtual int sync()
  {
    if (raw_)
    {
      *raw_ << "[" << type_ << "] " << str();
    }
    str("");
    return 0;
  }
};

DefaultStreamBuffer debug_buf(&std::cout, "DEBUG");
DefaultStreamBuffer info_buf(&std::cout, " INFO");
DefaultStreamBuffer warn_buf(&std::cout, " WARN");
DefaultStreamBuffer error_buf(&std::cerr, "ERROR");
DefaultStreamBuffer fatal_buf(&std::cerr, "FATAL");

std::ostream default_debug(&debug_buf);
std::ostream default_info(&info_buf);
std::ostream default_warn(&warn_buf);
std::ostream default_error(&error_buf);
std::ostream default_fatal(&fatal_buf);

std::ostream *current_debug(&default_debug);
std::ostream *current_info(&default_info);
std::ostream *current_warn(&default_warn);
std::ostream *current_error(&default_error);
std::ostream *current_fatal(&default_fatal);
}  // namespace

void setDebugLogger(std::ostream *l)
{
  current_debug = l;
}
void setInfoLogger(std::ostream *l)
{
  current_info = l;
}
void setWarnLogger(std::ostream *l)
{
  current_warn = l;
}
void setErrorLogger(std::ostream *l)
{
  current_error = l;
}
void setFatalLogger(std::ostream *l)
{
  current_fatal = l;
}

std::ostream &debug()
{
  return *current_debug;
}
std::ostream &info()
{
  return *current_info;
}
std::ostream &warn()
{
  return *current_warn;
}
std::ostream &error()
{
  return *current_error;
}
std::ostream &fatal()
{
  return *current_fatal;
}
}  // namespace logger
}  // namespace scip2
