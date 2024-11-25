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

#ifndef URG_STAMPED_STRINGS_H
#define URG_STAMPED_STRINGS_H

#include <sstream>
#include <string>
#include <vector>

namespace urg_stamped
{
namespace strings
{
inline std::vector<std::string> split(const std::string& s, const char delim)
{
  std::vector<std::string> out;
  std::stringstream stream(s);
  std::string el;
  while (std::getline(stream, el, delim))
  {
    out.push_back(el);
  }
  return out;
}
};  // namespace strings
};  // namespace urg_stamped

#endif  // URG_STAMPED_STRINGS_H

