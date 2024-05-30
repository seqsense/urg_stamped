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

#ifndef URG_SIM_ENCODE_H
#define URG_SIM_ENCODE_H

#include <cstdint>
#include <string>
#include <vector>

namespace urg_sim
{
namespace encode
{

std::string checksum(const std::string& a);
std::string withChecksum(const std::string& s);

enum class EncodeType
{
  CED2 = 2,
  CED3 = 3,
  CED4 = 4,
};

std::string encode(const std::vector<uint32_t>& v, const EncodeType ced);

}  // namespace encode
}  // namespace urg_sim

#endif  // URG_SIM_ENCODE_H
