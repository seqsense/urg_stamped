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

#ifndef E2E_UTILS_H
#define E2E_UTILS_H

#include <string>

#include <boost/asio/ip/tcp.hpp>

#include <ros/ros.h>

#include <ros/network.h>
#include <ros/xmlrpc_manager.h>
#include <xmlrpcpp/XmlRpc.h>

inline bool shutdownNode(const std::string& name)
{
  XmlRpc::XmlRpcValue req, res, payload;
  req[0] = ros::this_node::getName();
  req[1] = name;
  if (!ros::master::execute("lookupNode", req, res, payload, true))
  {
    return false;
  }

  std::string host;
  uint32_t port;
  if (!ros::network::splitURI(static_cast<std::string>(res[2]), host, port))
  {
    return false;
  }

  XmlRpc::XmlRpcClient cli(host.c_str(), port, "/");
  XmlRpc::XmlRpcValue req2, res2;
  return cli.execute("shutdown", req2, res2);
}

#endif  // E2E_UTILS_H
