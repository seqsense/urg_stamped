/*
 * Copyright 2024 The urg_stamped Authors
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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <ros/time.h>

#include <urg_stamped/device_state_estimator.h>
#include <scip2/logger.h>

namespace urg_stamped
{
namespace device_state_estimator
{

void ClockEstimatorUUST2::startSync()
{
  sync_samples_.clear();
  best_delay_ = ros::Duration(RESPONSE_TIMER_INTERVAL);
  cnt_samples_ = 0;
}

void ClockEstimatorUUST2::pushSyncSample(
    const ros::Time& t_req, const ros::Time& t_res, const uint64_t device_wall_stamp)
{
  cnt_samples_++;
  const SyncSampleUUST2 s(t_req, t_res, device_wall_stamp);

  if (s.delay_ < ros::Duration(ACCEPTABLE_SAMPLE_DELAY))
  {
    sync_samples_.push_back(s);
  }
  if (s.delay_ < best_delay_)
  {
    best_delay_ = s.delay_;
  }
  if (comm_delay_.min_.isZero() || comm_delay_.min_ > s.delay_)
  {
    comm_delay_.min_ = s.delay_;
  }
}

bool ClockEstimatorUUST2::hasEnoughSyncSamples() const
{
  return cnt_samples_ >= MAX_SYNC_ATTEMPTS ||
         (sync_samples_.size() >= MIN_SYNC_SAMPLES &&
          best_delay_ < ros::Duration(DEVICE_TIMESTAMP_RESOLUTION));
}

bool ClockEstimatorUUST2::finishSync()
{
  if (sync_samples_.size() < MIN_SYNC_SAMPLES)
  {
    scip2::logger::error()
        << "Not enough number of usable sync responses. "
        << "Communication delay may be too large"
        << std::endl;
    return false;
  }

  if (best_delay_ >= ros::Duration(DEVICE_TIMESTAMP_RESOLUTION))
  {
    // UUST2 sets a device timestamp of sometime between command receive time
    // and 5ms response timer. So, the timestamp can be matched only when
    // it's returned within DEVICE_TIMESTAMP_RESOLUTION.
    scip2::logger::error()
        << "No sync response with small delay. "
        << "Communication delay may be too large"
        << std::endl;
    return false;
  }

  ros::Time min_t_origin = sync_samples_.front().t_origin_;
  for (const auto& s : sync_samples_)
  {
    if (s.t_origin_ < min_t_origin)
    {
      min_t_origin = s.t_origin_;
    }
  }

  std::vector<SyncSampleUUST2> sorted_samples;
  for (auto& s : sync_samples_)
  {
    const double origin_compensate =
        std::round((s.t_origin_ - min_t_origin).toSec() / DEVICE_TIMESTAMP_RESOLUTION) * DEVICE_TIMESTAMP_RESOLUTION;

    s.t_origin_ -= ros::Duration(origin_compensate);
    sorted_samples.push_back(s);

    SyncSampleUUST2 s2 = s;
    s2.t_frac_ += ros::Duration(RESPONSE_TIMER_INTERVAL);
    sorted_samples.push_back(s2);
  }

  std::sort(sorted_samples.begin(), sorted_samples.end());
  const SyncSampleUUST2 med = sorted_samples[sorted_samples.size() / 2];

  comm_delay_.sigma_ = ros::Duration(0.005);  // Not used by UST scan estimator

  scip2::logger::debug()
      << "delay: " << med.delay_
      << ", device timestamp: " << med.device_wall_stamp_
      << std::endl;

  const ClockSample clock =
      {
          .t_estim_ = med.t_process_,
          .stamp_ = med.device_wall_stamp_,
          .origin_ = med.t_origin_,
      };
  return pushClockSample(clock);
}

}  // namespace device_state_estimator
}  // namespace urg_stamped

