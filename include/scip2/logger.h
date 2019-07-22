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

#ifndef SCIP2_LOGGER_H
#define SCIP2_LOGGER_H

#include <iostream>

namespace scip2
{
namespace logger
{
extern std::ostream debug;
extern std::ostream info;
extern std::ostream warn;
extern std::ostream error;
extern std::ostream fatal;
}  // namespace logger
}  // namespace scip2

#endif  // SCIP2_LOGGER_H
