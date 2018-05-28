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

#ifndef SCIP2_CONNECTION_H
#define SCIP2_CONNECTION_H

#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

#include <string>

#include <old_boost_fix.h>

namespace scip2
{
class Protocol;
class Connection
{
  friend class Protocol;

protected:
  using CallbackConnect = boost::function<void(void)>;
  using CallbackClose = boost::function<void(void)>;
  using CallbackReceive = boost::function<void(
      boost::asio::streambuf &, const boost::posix_time::ptime &)>;
  using CallbackSend = boost::function<void(
      const boost::posix_time::ptime &)>;

  CallbackConnect cb_connect_;
  CallbackClose cb_close_;
  CallbackReceive cb_receive_;

  void close()
  {
    if (cb_close_)
      cb_close_();
  }
  void connect()
  {
    if (cb_connect_)
      cb_connect_();
  }
  void receive(
      boost::asio::streambuf &buf,
      const boost::posix_time::ptime &time_read)
  {
    if (cb_receive_)
      cb_receive_(buf, time_read);
  }

public:
  using Ptr = std::shared_ptr<Connection>;

  virtual void spin() = 0;
  virtual void stop() = 0;
  virtual void send(const std::string &, CallbackSend = CallbackSend()) = 0;
  virtual void startWatchdog(const boost::posix_time::time_duration &) = 0;

  void registerCloseCallback(CallbackClose cb)
  {
    cb_close_ = cb;
  }
  void registerReceiveCallback(CallbackReceive cb)
  {
    cb_receive_ = cb;
  }
  void registerConnectCallback(CallbackConnect cb)
  {
    cb_connect_ = cb;
  }
  Connection()
  {
  }
};

class ConnectionTcp : public Connection
{
protected:
  boost::asio::io_service io_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::streambuf buf_;
  boost::asio::deadline_timer timeout_;
  boost::asio::deadline_timer watchdog_;
  boost::posix_time::time_duration watchdog_duration_;

  void clearWatchdog()
  {
    if (watchdog_duration_ == boost::posix_time::time_duration())
      return;
    watchdog_.cancel();

    watchdog_.expires_from_now(watchdog_duration_);
    watchdog_.async_wait(
        boost::bind(&ConnectionTcp::onWatchdog, this, boost::asio::placeholders::error));
  }
  void onWatchdog(const boost::system::error_code &error)
  {
    if (!error)
    {
      std::cerr << "Watchdog timeout" << std::endl;
      close();
    }
  }

  void onReceive(const boost::system::error_code &error)
  {
    const auto time_read = boost::posix_time::microsec_clock::universal_time();
    if (error)
    {
      std::cerr << "Receive error" << std::endl;
      close();
      return;
    }
    clearWatchdog();
    receive(buf_, time_read);
    asyncRead();
  }
  void onSend(const boost::system::error_code &error, CallbackSend cb)
  {
    const auto time_send = boost::posix_time::microsec_clock::universal_time();
    if (error)
    {
      std::cerr << "Send error" << std::endl;
      close();
      return;
    }
    if (cb)
      cb(time_send);
  }

  void asyncRead()
  {
    boost::asio::async_read_until(
        socket_, buf_, "\n\n",
        boost::bind(&ConnectionTcp::onReceive, this, boost::asio::placeholders::error));
  }
  void onConnect(const boost::system::error_code &error)
  {
    if (error)
    {
      std::cerr << "Connection error" << std::endl;
      close();
      return;
    }
    timeout_.cancel();
    connect();
    asyncRead();
  }
  void onConnectTimeout(const boost::system::error_code &error)
  {
    if (!error)
    {
      std::cerr << "Connection timeout" << std::endl;
      close();
      return;
    }
  }

public:
  using Ptr = std::shared_ptr<ConnectionTcp>;

  ConnectionTcp(const std::string &ip, const uint16_t port)
    : socket_(io_)
    , timeout_(io_)
    , watchdog_(io_)
  {
    boost::asio::ip::tcp::endpoint endpoint(
        boost::asio::ip::address::from_string(ip), port);
    socket_.async_connect(
        endpoint,
        boost::bind(
            &ConnectionTcp::onConnect,
            this, boost::asio::placeholders::error));

    timeout_.expires_from_now(boost::posix_time::seconds(2));
    timeout_.async_wait(
        boost::bind(
            &ConnectionTcp::onConnectTimeout,
            this, boost::asio::placeholders::error));
  }
  void spin()
  {
    io_.run();
  }
  void stop()
  {
    io_.stop();
  }
  void send(const std::string &data, CallbackSend cb = CallbackSend())
  {
    boost::shared_ptr<std::string> buf(new std::string(data));
    boost::asio::async_write(
        socket_, boost::asio::buffer(*buf),
        boost::bind(
            &ConnectionTcp::onSend,
            this, boost::asio::placeholders::error, cb));
  }
  void startWatchdog(const boost::posix_time::time_duration &duration)
  {
    watchdog_duration_ = duration;
    clearWatchdog();
  }
};

}  // namespace scip2

#endif  // SCIP2_CONNECTION_H
