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
#include <memory>
#include <utility>
#include <vector>

#include <ros/time.h>

#include <urg_stamped/device_state_estimator.h>
#include <scip2/logger.h>

namespace urg_stamped
{
namespace device_state_estimator
{

ros::Time ClockState::stampToTime(const uint64_t stamp) const
{
  const double from_origin = (stamp_ + (int64_t)(stamp - stamp_) / gain_) / 1000.0;
  return origin_ + ros::Duration(from_origin);
}

ScanState::ScanState(const std::vector<ScanSample>& samples)
{
  std::vector<ScanSample> s(samples);
  std::sort(s.begin(), s.end());
  const ScanSample& med = s[s.size() / 2];
  origin_ = med.t_;
  interval_ = med.interval_;
}

ros::Time ScanState::fit(const ros::Time& t) const
{
  const double from_origin = (t - origin_).toSec();
  const double interval = interval_.toSec();
  const int n = std::lround(from_origin / interval);
  return origin_ + ros::Duration(interval * n);
}

void Estimator::startSync()
{
  sync_samples_.clear();
}

void Estimator::pushSyncSample(const ros::Time& t_req, const ros::Time& t_res, const uint64_t device_wall_stamp)
{
  sync_samples_.emplace_back(t_req, t_res, device_wall_stamp);
}

bool Estimator::hasEnoughSyncSamples() const
{
  const size_t n = sync_samples_.size();
  if (n < MIN_SYNC_SAMPLES)
  {
    return false;
  }
  if (n >= MAX_SYNC_SAMPLES)
  {
    return true;
  }
  const OriginFracPart overflow_range = originFracOverflow();
  return overflow_range.t_max_ > overflow_range.t_min_;
}

void Estimator::finishSync()
{
  const OriginFracPart overflow_range = originFracOverflow();
  if (!overflow_range)
  {
    scip2::logger::warn()
        << "failed to find origin fractional part overflow: "
        << overflow_range.t_min_ << ", " << overflow_range.t_max_
        << ", samples=" << sync_samples_.size()
        << std::endl;
    return;
  }
  const auto min_delay = findMinDelay(overflow_range);
  if (min_delay == sync_samples_.cend())
  {
    scip2::logger::warn() << "failed to find minimal delay sample" << std::endl;
    return;
  }
  comm_delay_sigma_ = delaySigma();

  const ClockState last = clock_;

  clock_.origin_ = overflow_range.compensate(min_delay->t_origin_);
  clock_.stamp_ = min_delay->device_wall_stamp_;
  clock_.t_estim_ = min_delay->t_process_;
  if (min_comm_delay_.isZero() || min_comm_delay_ > min_delay->delay_)
  {
    min_comm_delay_ = min_delay->delay_;
  }

  if (last.origin_.isZero())
  {
    return;
  }

  const double t_diff = (clock_.t_estim_ - last.t_estim_).toSec();
  const double origin_diff =
      (clock_.origin_ - last.origin_).toSec();
  const double gain = (t_diff - origin_diff) / t_diff;
  clock_.gain_ = gain;
  clock_.initialized_ = true;

  scip2::logger::debug()
      << "origin: " << clock_.origin_
      << ", gain: " << gain
      << ", delay: " << min_delay->delay_
      << ", device timestamp: " << min_delay->device_wall_stamp_
      << std::endl;
}

std::vector<SyncSample>::const_iterator Estimator::findMinDelay(const OriginFracPart& overflow_range) const
{
  if (sync_samples_.size() == 0)
  {
    return sync_samples_.cend();
  }
  auto it_min_delay = sync_samples_.cbegin();
  for (auto it = sync_samples_.cbegin() + 1; it != sync_samples_.cend(); it++)
  {
    if (overflow_range.isOnOverflow(it->t_process_))
    {
      continue;
    }
    if (it->delay_ < it_min_delay->delay_)
    {
      it_min_delay = it;
    }
  }
  return it_min_delay;
}

OriginFracPart Estimator::originFracOverflow() const
{
  if (sync_samples_.size() == 0)
  {
    return OriginFracPart();
  }
  auto it_min_origin = sync_samples_.begin();
  auto it_max_origin = sync_samples_.begin();
  for (auto it = sync_samples_.begin() + 1; it != sync_samples_.end(); it++)
  {
    if (it->t_origin_ < it_min_origin->t_origin_)
    {
      it_min_origin = it;
    }
    if (it->t_origin_ > it_max_origin->t_origin_)
    {
      it_max_origin = it;
    }
  }
  double t_min = std::fmod(it_min_origin->t_process_.toSec(), 0.001);
  double t_max = std::fmod(it_max_origin->t_process_.toSec(), 0.001);
  if (t_min > t_max + 0.0005)
  {
    t_max += 0.001;
  }
  else if (t_max > t_min + 0.0005)
  {
    t_min += 0.001;
  }
  if (std::abs(t_max - t_min) > 0.00025)
  {
    return OriginFracPart(t_min, t_max, false);
  }
  return OriginFracPart(t_min, t_max);
}

std::pair<ros::Time, bool> Estimator::pushScanSample(const ros::Time& t_recv, const uint64_t device_wall_stamp)
{
  const std::pair<ros::Time, bool> t_scan_raw = pushScanSampleRaw(t_recv, device_wall_stamp);
  if (!t_scan_raw.second)
  {
    return t_scan_raw;
  }

  recent_t_scans_.emplace_back(t_scan_raw.first);
  if (recent_t_scans_.size() < MIN_SCAN_SAMPLES)
  {
    return t_scan_raw;
  }
  if (recent_t_scans_.size() >= MAX_SCAN_SAMPLES)
  {
    recent_t_scans_.pop_front();
  }

  std::vector<ScanSample> samples;
  for (size_t i = 1; i < recent_t_scans_.size(); ++i)
  {
    const ros::Duration interval = recent_t_scans_[i] - recent_t_scans_[i - 1];
    samples.emplace_back(recent_t_scans_[i], interval);
  }
  ScanState s(samples);
  scan_ = s;
  return std::pair<ros::Time, bool>(s.fit(t_scan_raw.first), true);
}

std::pair<ros::Time, bool> Estimator::pushScanSampleRaw(const ros::Time& t_recv, const uint64_t device_wall_stamp)
{
  const ros::Time t_stamp = clock_.stampToTime(device_wall_stamp);
  if (!clock_.initialized_)
  {
    return std::pair<ros::Time, bool>(t_stamp, true);
  }

  const ros::Time t_sent = t_recv - min_comm_delay_;
  const ros::Duration stamp_to_send = t_sent - t_stamp;
  ros::Duration new_min_stamp_to_send = min_stamp_to_send_;
  if (new_min_stamp_to_send.isZero() || stamp_to_send < new_min_stamp_to_send)
  {
    new_min_stamp_to_send = stamp_to_send;
  }
  if (min_stamp_to_send_.isZero())
  {
    min_stamp_to_send_ = new_min_stamp_to_send;
  }
  if (stamp_to_send - new_min_stamp_to_send < ros::Duration(0.001) && new_min_stamp_to_send < min_stamp_to_send_)
  {
    std::cerr
        << "reduce min_stamp_to_send_ "
        << device_wall_stamp
        << " " << min_stamp_to_send_
        << " " << new_min_stamp_to_send
        << std::endl;
    min_stamp_to_send_ =
        min_stamp_to_send_ * (1 - MIN_STAMP_TO_SEND_ALPHA) +
        new_min_stamp_to_send * MIN_STAMP_TO_SEND_ALPHA;
  }
  if (stamp_to_send - min_stamp_to_send_ > ros::Duration(0.001) + comm_delay_sigma_ * 2 &&
      stamp_to_send - min_stamp_to_send_ < ros::Duration(0.002))
  {
    std::cerr
        << "increase min_stamp_to_send_ "
        << device_wall_stamp
        << " " << min_stamp_to_send_
        << " " << new_min_stamp_to_send
        << std::endl;
    min_stamp_to_send_ =
        min_stamp_to_send_ * (1 - MIN_STAMP_TO_SEND_ALPHA) +
        (stamp_to_send - ros::Duration(0.001)) * MIN_STAMP_TO_SEND_ALPHA;
  }

  const ros::Duration t_frac = stamp_to_send - min_stamp_to_send_ - comm_delay_sigma_;
  const ros::Time t_scan_raw = t_stamp + t_frac;
  const bool valid = t_frac < ros::Duration(0.0015);

  if (!valid)
  {
    std::cerr
        << "invalid estimation result "
        << stamp_to_send
        << " "
        << min_stamp_to_send_
        << " "
        << t_frac
        << " "
        << min_comm_delay_
        << " "
        << comm_delay_sigma_
        << std::endl;
  }

  return std::pair<ros::Time, bool>(t_scan_raw, valid);
}

ros::Duration Estimator::delaySigma() const
{
  if (sync_samples_.size() == 0)
  {
    return ros::Duration();
  }
  double sum = 0;
  for (const auto& s : sync_samples_)
  {
    const double delay_diff = (s.delay_ - min_comm_delay_).toSec();
    sum += delay_diff * delay_diff;
  }
  return ros::Duration(std::sqrt(sum / sync_samples_.size()));
}

}  // namespace device_state_estimator
}  // namespace urg_stamped
