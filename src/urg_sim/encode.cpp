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

#include <string>
#include <vector>

#include <urg_sim/encode.h>

namespace urg_sim
{
namespace encode
{

std::string checksum(const std::string& s)
{
  char sum = 0;
  for (const char c : s)
  {
    sum += c;
  }
  return std::string(1, (sum & 0x3F) + 0x30);
}

std::string withChecksum(const std::string& s)
{
  return s + checksum(s);
}

std::string encode(const std::vector<uint32_t>& v, const EncodeType ced)
{
  std::string out;

  for (const uint32_t w : v)
  {
    for (int i = 0; i < static_cast<int>(ced); ++i)
    {
      out += ((w >> ((static_cast<int>(ced) - i - 1) * 6)) & 0x3F) + 0x30;
    }
  }
  return out;
}

}  // namespace encode
}  // namespace urg_sim

