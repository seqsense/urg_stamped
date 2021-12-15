/*
 * Copyright 2021 The urg_stamped Authors
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

#ifndef SCIP2_PARAM_H
#define SCIP2_PARAM_H

#include <string>
#include <algorithm>

namespace scip2
{
struct ParsedParam
{
  bool parsed;
  bool checksum_matched;
  std::string key;
  std::string value;
  std::string error;

  ParsedParam()
    : parsed(false)
    , checksum_matched(false)
  {
  }
};

static ParsedParam parseParamLine(const std::string& line)
{
  ParsedParam ret;

  if (line.size() < 3)
  {
    // ":;[checksum]"
    ret.error = "parameter line must have at least 3 chars";
    return ret;
  }

  const auto delm = std::find(line.begin(), line.end(), ':');
  if (delm == line.end())
  {
    ret.error = "delimiter not found";
    return ret;
  }
  auto ie = line.rbegin();
  const uint8_t checksum = *ie;
  ++ie;
  if (*ie != ';')
  {
    ret.error = "checksum delimiter not found";
    return ret;
  }
  const auto end = line.end() - 2;
  ret.parsed = true;
  ret.key = std::string(line.begin(), delm);
  ret.value = std::string(delm + 1, end);

  uint8_t sum = 0;
  for (auto it = line.begin(); it != end; ++it)
  {
    sum += *it;
  }
  if ((sum & 0x3F) + 0x30 == checksum)
  {
    ret.checksum_matched = true;
  }
  return ret;
}
}  // namespace scip2

#endif  // SCIP2_PARAM_H
