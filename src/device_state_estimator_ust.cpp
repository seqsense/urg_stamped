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

#include <fstream>

#include <ros/time.h>

#include <urg_stamped/device_state_estimator.h>
#include <scip2/logger.h>

namespace urg_stamped
{
namespace device_state_estimator
{

std::pair<ros::Time, bool> EstimatorUST::pushScanSample(const ros::Time& t_recv, const uint64_t device_wall_stamp)
{
  const ros::Time t_stamp = clock_.stampToTime(device_wall_stamp);
  return std::pair<ros::Time, bool>(t_stamp, true);
}

}  // namespace device_state_estimator
}  // namespace urg_stamped
