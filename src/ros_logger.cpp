/*
 * Copyright 2019-2021 The urg_stamped Authors
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

#include <ros/ros.h>

#include <scip2/logger.h>

namespace
{
class ROSOutStreamBuffer : public std::stringbuf
{
public:
  enum Type
  {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    LOG_FATAL,
  };
  Type type_;
  std::string prefix_;

  explicit ROSOutStreamBuffer(Type type)
    : type_(type)
  {
  }
  virtual int sync()
  {
    std::string log = str();
    if (!log.empty() && log.back() == '\n')
      log.pop_back();

    if (ros::ok())
    {
      switch (type_)
      {
        case LOG_DEBUG:
          ROS_DEBUG("%s%s", prefix_.c_str(), log.c_str());
          break;
        case LOG_INFO:
          ROS_INFO("%s%s", prefix_.c_str(), log.c_str());
          break;
        case LOG_WARN:
          ROS_WARN("%s%s", prefix_.c_str(), log.c_str());
          break;
        case LOG_ERROR:
          ROS_ERROR("%s%s", prefix_.c_str(), log.c_str());
          break;
        case LOG_FATAL:
          ROS_FATAL("%s%s", prefix_.c_str(), log.c_str());
          break;
      }
    }
    else
    {
      // Fallback to stderr/stdout after ros::shutdown.
      switch (type_)
      {
        case LOG_DEBUG:
          break;
        case LOG_INFO:
        case LOG_WARN:
          std::cout << prefix_ << log << std::endl;
          break;
        case LOG_ERROR:
        case LOG_FATAL:
          std::cerr << prefix_ << log << std::endl;
          break;
      }
    }
    str("");
    return 0;
  }

  void setPrefix(const std::string& prefix)
  {
    prefix_ = prefix;
  }
};

ROSOutStreamBuffer debug_buf(ROSOutStreamBuffer::LOG_DEBUG);
ROSOutStreamBuffer info_buf(ROSOutStreamBuffer::LOG_INFO);
ROSOutStreamBuffer warn_buf(ROSOutStreamBuffer::LOG_WARN);
ROSOutStreamBuffer error_buf(ROSOutStreamBuffer::LOG_ERROR);
ROSOutStreamBuffer fatal_buf(ROSOutStreamBuffer::LOG_FATAL);
std::ostream debug_logger(&debug_buf);
std::ostream info_logger(&info_buf);
std::ostream warn_logger(&warn_buf);
std::ostream error_logger(&error_buf);
std::ostream fatal_logger(&fatal_buf);
}  // namespace

namespace urg_stamped
{
void setROSLogger(const std::string& prefix)
{
  debug_buf.setPrefix(prefix);
  info_buf.setPrefix(prefix);
  warn_buf.setPrefix(prefix);
  error_buf.setPrefix(prefix);
  fatal_buf.setPrefix(prefix);
  scip2::logger::setDebugLogger(&debug_logger);
  scip2::logger::setInfoLogger(&info_logger);
  scip2::logger::setWarnLogger(&warn_logger);
  scip2::logger::setErrorLogger(&error_logger);
  scip2::logger::setFatalLogger(&fatal_logger);
}
}  // namespace urg_stamped
