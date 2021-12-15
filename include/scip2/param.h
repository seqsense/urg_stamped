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

namespace scip2
{
struct ParsedParam
{
  bool parsed;
  bool checksum_matched;
  std::string key;
  std::string value;
};

static ParsedParam parseParamLine(const std::string& line)
{
  ParsedParam ret;

  const auto delm = std::find(line.begin(), line.end(), ':');
  if (delm == line.end())
  {
    return ret;
  }
  const auto end = std::find(line.begin(), line.end(), ';');
  if (end == line.end())
  {
    return ret;
  }
  ret.parsed = true;
  ret.key = std::string(line.begin(), delm);
  ret.value = std::string(delm + 1, end);

  const uint8_t checksum = line.back();
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
