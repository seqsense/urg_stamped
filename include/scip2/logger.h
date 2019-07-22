/*
 * Copyright 2019 The urg_stamped Authors
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
void setDebugLogger(std::ostream *l);
void setInfoLogger(std::ostream *l);
void setWarnLogger(std::ostream *l);
void setErrorLogger(std::ostream *l);
void setFatalLogger(std::ostream *l);
std::ostream &debug();
std::ostream &info();
std::ostream &warn();
std::ostream &error();
std::ostream &fatal();
}  // namespace logger
}  // namespace scip2

#endif  // SCIP2_LOGGER_H
