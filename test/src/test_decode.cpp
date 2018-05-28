/*
 * Copyright 2018 SEQSENSE, Inc.
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

#include <scip2/decode.h>

TEST(DecoderTest, testChecksumValidation)
{
  // Checksum example shown in SCIP2.0 Specification
  scip2::Decoder<1> dec("Hokuyo");
  ASSERT_EQ((dec.getChecksum() & 0x3F) + 0x30, 'o');
}

TEST(DecoderTest, testDecodeSingle)
{
  // Examples shown in SCIP2.0 Specification
  scip2::Decoder<2> dec2("CB");
  ASSERT_EQ(*dec2.begin(), 1234);

  scip2::Decoder<3> dec3("1Dh");
  ASSERT_EQ(*dec3.begin(), 5432);

  scip2::Decoder<4> dec4("m2@0");
  ASSERT_EQ(*dec4.begin(), 16000000);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
