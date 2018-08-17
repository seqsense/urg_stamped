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

#ifndef FILTER_H
#define FILTER_H

#include <cmath>

template <typename FLT>
class Filter
{
public:
  enum Type
  {
    HPF,
    LPF
  };

protected:
  FLT x_;
  FLT k_[4];

public:
  Filter(const enum Type type, const FLT tc)
    : x_(0)
  {
    switch (type)
    {
      case LPF:
        k_[3] = -1 / (1.0 + 2 * tc);
        k_[2] = -k_[3];
        k_[1] = (1.0 - 2 * tc) * k_[3];
        k_[0] = -k_[1] - 1.0;
        break;
      case HPF:
        k_[3] = -1 / (1.0 + 2 * tc);
        k_[2] = -k_[3] * 2 * tc;
        k_[1] = (1.0 - 2 * tc) * k_[3];
        k_[0] = 2 * tc * (-k_[1] + 1.0);
        break;
    }
  }
  FLT in(const FLT in)
  {
    x_ = k_[0] * in + k_[1] * x_;
    const auto out = k_[2] * in + k_[3] * x_;

    return out;
  }
};

#endif  // FILTER_H
