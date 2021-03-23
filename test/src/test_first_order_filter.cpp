/*
 * Copyright 2018-2021 The urg_stamped Authors
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

#include <gtest/gtest.h>

#include <cmath>

#include <urg_stamped/first_order_filter.h>

namespace urg_stamped
{
TEST(FirstOrderFilter, PassThrough)
{
  FirstOrderFilter<double> flt;

  ASSERT_EQ(flt.update(0.0), 0.0);
  ASSERT_EQ(flt.update(1.0), 1.0);
  ASSERT_EQ(flt.update(2.0), 2.0);
}

TEST(FirstOrderFilter, LPF)
{
  FirstOrderLPF<double> flt(100);

  ASSERT_EQ(flt.update(0.0), 0.0);
  for (size_t i = 0; i < 100 - 1; ++i)
    flt.update(1.0);
  ASSERT_NEAR(flt.update(1.0), 1.0 - 1.0 / std::exp(1), 1e-2);
}

TEST(FirstOrderFilter, HPF)
{
  FirstOrderHPF<double> flt(100);

  ASSERT_EQ(flt.update(0.0), 0.0);
  for (size_t i = 0; i < 100 - 2; ++i)
    flt.update(1.0);
  ASSERT_NEAR(flt.update(1.0), 1.0 / std::exp(1), 1e-2);
}
}  // namespace urg_stamped

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
