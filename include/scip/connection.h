/*
 * Copyright(c) 2018, SEQSENSE, Inc.
 * All rights reserved.
 */

#ifndef SCIP_CONNECTION_H
#define SCIP_CONNECTION_H

#include <boost/asio.hpp>

#include <string>

namespace scip
{
class Connection
{
protected:
  using CallbackClose = boost::function<void(void)>;

  bool active_;
  CallbackClose cb_close_;

  void close()
  {
    if (cb_close_)
      cb_close_();
  }

public:
  using Ptr = std::shared_ptr<Connection>;

  virtual void spin() = 0;
  virtual void stop() = 0;

  bool isActive()
  {
    return active_;
  }
  void registerCloseCallback(CallbackClose cb)
  {
    cb_close_ = cb;
  }
  Connection()
    : active_(false)
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

  void onReceive(const boost::system::error_code &error)
  {
    std::cerr << "recieve" << std::endl;
  }

  void onConnect(const boost::system::error_code &error)
  {
    if (!error)
    {
      active_ = true;
      timeout_.cancel();
      boost::asio::async_read(
          socket_, buf_, boost::asio::transfer_at_least(4),
          boost::bind(&ConnectionTcp::onReceive, this, boost::asio::placeholders::error));

      std::cerr << "connected" << std::endl;
    }
    else
    {
      std::cerr << "connection error" << std::endl;
      close();
    }
  }
  void onConnectTimeout(const boost::system::error_code &error)
  {
    if (!error)
    {
      std::cerr << "connection timeout" << std::endl;
      close();
    }
  }

public:
  ConnectionTcp(const std::string &ip, const uint16_t port)
    : socket_(io_)
    , timeout_(io_)
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
};

}  // namespace scip

#endif  // SCIP_CONNECTION_H
