/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
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
