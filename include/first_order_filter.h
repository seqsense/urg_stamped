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

#ifndef FIRST_ORDER_FILTER_H
#define FIRST_ORDER_FILTER_H

template <typename FLT>
class FirstOrderFilter
{
protected:
  FLT x_;
  FLT k_[4];

public:
  FirstOrderFilter()
    : x_(0)
  {
    // Pass through
    k_[0] = k_[1] = k_[3] = 0;
    k_[2] = 1;
  }
  FLT update(const FLT &in)
  {
    x_ = k_[0] * in + k_[1] * x_;
    const auto out = k_[2] * in + k_[3] * x_;

    return out;
  }
};

template <typename FLT>
class FirstOrderLPF : public FirstOrderFilter<FLT>
{
public:
  explicit FirstOrderLPF(const FLT &time_constant)
  {
    this->k_[3] = -1 / (1.0 + 2 * time_constant);
    this->k_[2] = -this->k_[3];
    this->k_[1] = (1.0 - 2 * time_constant) * this->k_[3];
    this->k_[0] = -this->k_[1] - 1.0;
  }
};

template <typename FLT>
class FirstOrderHPF : public FirstOrderFilter<FLT>
{
public:
  explicit FirstOrderHPF(const FLT &time_constant)
  {
    this->k_[3] = -1 / (1.0 + 2 * time_constant);
    this->k_[2] = -this->k_[3] * 2 * time_constant;
    this->k_[1] = (1.0 - 2 * time_constant) * this->k_[3];
    this->k_[0] = 2 * time_constant * (-this->k_[1] + 1.0);
  }
};

#endif  // FIRST_ORDER_FILTER_H
