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

#include <gtest/gtest.h>

#include <string>

#include <scip2/decode.h>

TEST(DecoderTest, testChecksumValidation)
{
  // Checksum example shown in SCIP2.0 Specification
  const std::string line("Hokuyo");
  scip2::Decoder<1> dec(line);
  ASSERT_EQ((dec.getChecksum() & 0x3F) + 0x30, 'o');
}

TEST(DecoderTest, testDecodeSingle)
{
  // Examples shown in SCIP2.0 Specification
  const std::string line2("CB");
  scip2::Decoder<2> dec2(line2);
  ASSERT_EQ(*dec2.begin(), 1234u);

  const std::string line3("1Dh");
  scip2::Decoder<3> dec3(line3);
  ASSERT_EQ(*dec3.begin(), 5432u);

  const std::string line4("m2@0");
  scip2::Decoder<4> dec4(line4);
  ASSERT_EQ(*dec4.begin(), 16000000u);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
