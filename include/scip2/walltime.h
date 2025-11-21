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

#ifndef SCIP2_WALLTIME_H
#define SCIP2_WALLTIME_H

#include <cstdint>

#include <scip2/logger.h>

namespace scip2
{
template <int DEVICE_TIMESTAMP_BITS>
class Walltime
{
protected:
  bool initialized_;
  uint64_t time_device_prev_;
  uint64_t walltime_device_base_;

  constexpr static uint32_t middle_bits_ = (1 << DEVICE_TIMESTAMP_BITS) / 2;

public:
  uint64_t update(const uint64_t time_device_us)
  {
    const uint32_t time_device_ms = time_device_us / 1000;
    if (!initialized_)
    {
      time_device_prev_ = time_device_ms;
      initialized_ = true;
    }

    if (detectDeviceTimeUnderflow(time_device_ms))
    {
      if (walltime_device_base_ >= (1 << DEVICE_TIMESTAMP_BITS))
      {
        time_device_prev_ = (1 << DEVICE_TIMESTAMP_BITS) - time_device_ms;
        return static_cast<uint64_t>(walltime_device_base_ - time_device_prev_) * 1000;
      }
      logger::warn() << "Device time jumped. prev: " << time_device_prev_
                     << ", current: " << time_device_ms << std::endl;
    }

    if (time_device_ms < middle_bits_ &&
        middle_bits_ < time_device_prev_)
    {
      walltime_device_base_ += 1 << DEVICE_TIMESTAMP_BITS;
    }

    time_device_prev_ = time_device_ms;

    return static_cast<uint64_t>(walltime_device_base_ + time_device_ms) * 1000;
  }

  Walltime()
    : initialized_(false)
    , time_device_prev_(0)
    , walltime_device_base_(0)
  {
  }

private:
  bool detectDeviceTimeUnderflow(uint32_t time_device_ms) const
  {
    return (time_device_prev_ < middle_bits_ &&
            middle_bits_ < time_device_ms &&
            time_device_ms - time_device_prev_ > middle_bits_);
  }
};
}  // namespace scip2

#endif  // SCIP2_WALLTIME_H
