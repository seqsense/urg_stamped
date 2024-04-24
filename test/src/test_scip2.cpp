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

#include <gtest/gtest.h>

#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include <scip2/protocol.h>
#include <scip2/connection.h>

class ConnectionDummy : public scip2::Connection
{
public:
  void spin() final
  {
  }
  void stop() final
  {
  }
  void send(const std::string&, CallbackSend = CallbackSend()) final
  {
  }
  void startWatchdog(const boost::posix_time::time_duration&) final
  {
  }
  void feed(boost::asio::streambuf& buf, const boost::posix_time::ptime& time_read)
  {
    receive(buf, time_read);
  }
};

TEST(SCIP2, MultipleResponses)
{
  std::shared_ptr<ConnectionDummy> dev(new ConnectionDummy());
  scip2::Protocol p(dev);

  const auto now = boost::posix_time::microsec_clock::universal_time();
  int num_receive = 0;

  const auto cb =
      [&num_receive, now](
          const boost::posix_time::ptime& time_read,
          const std::string& echo_back,
          const std::string& status)
  {
    num_receive++;
    EXPECT_EQ(now, time_read);
    EXPECT_EQ("00", status);
  };
  p.registerCallback<scip2::ResponseQT>(cb);

  /*
  Input data:

  QT  // First QT response
  00P

  FOO // Unknown response
  BAR
  QT  // This line must be ignored as a part of unknown command

  QT  // This must be ignored as it doesn't have correct status line

  QT  // Second QT response
  00P

  */
  boost::asio::streambuf buf;
  std::ostream os(&buf);
  os << "QT\n00P\n\nF";
  dev->feed(buf, now);
  ASSERT_EQ(1, num_receive);

  os << "OO\nBAR\nQT\n\n";
  dev->feed(buf, now);
  ASSERT_EQ(1, num_receive);

  os << "QT\n\nQT\n";
  dev->feed(buf, now);
  ASSERT_EQ(1, num_receive);

  os << "00P\n\n";
  dev->feed(buf, now);
  ASSERT_EQ(2, num_receive);

  os << "\n\n";
  dev->feed(buf, now);
  ASSERT_EQ(2, num_receive);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
