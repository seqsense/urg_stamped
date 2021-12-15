/*
 * Copyright 2021 The urg_stamped Authors
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

#include <scip2/param.h>

TEST(ParseParamLine, parsed)
{
  // Checksum example shown in SCIP2.0 Specification
  const std::string line("PROT:SCIP 2.0;N");
  const scip2::ParsedParam p = scip2::parseParamLine(line);
  ASSERT_EQ(true, p.parsed) << p.error;
  ASSERT_EQ(true, p.checksum_matched);
  ASSERT_EQ("PROT", p.key);
  ASSERT_EQ("SCIP 2.0", p.value);
}

TEST(ParseParamLine, shortLine)
{
  const std::string line(";");
  const scip2::ParsedParam p = scip2::parseParamLine(line);
  ASSERT_EQ(false, p.parsed);
}

TEST(ParseParamLine, noDelimiter)
{
  const std::string line("PROT+SCIP 2.0;N");
  const scip2::ParsedParam p = scip2::parseParamLine(line);
  ASSERT_EQ(false, p.parsed);
}

TEST(ParseParamLine, noChecksumDelimiter)
{
  const std::string line("PROT:SCIP 2.0");
  const scip2::ParsedParam p = scip2::parseParamLine(line);
  ASSERT_EQ(false, p.parsed);
}
TEST(ParseParamLine, checksumMismatch)
{
  const std::string line("PROT:SCIP 2.0;X");
  const scip2::ParsedParam p = scip2::parseParamLine(line);
  ASSERT_EQ(true, p.parsed) << p.error;
  ASSERT_EQ(false, p.checksum_matched);
  ASSERT_EQ("PROT", p.key);
  ASSERT_EQ("SCIP 2.0", p.value);
}

TEST(ParseParamLine, containsSemicolon)
{
  const std::string line("TIME:j;o[;H");
  const scip2::ParsedParam p = scip2::parseParamLine(line);
  ASSERT_EQ(true, p.parsed) << p.error;
  ASSERT_EQ(true, p.checksum_matched);
  ASSERT_EQ("TIME", p.key);
  ASSERT_EQ("j;o[", p.value);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
