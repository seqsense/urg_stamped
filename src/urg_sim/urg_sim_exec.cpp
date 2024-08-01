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

#include <iostream>
#include <string>
#include <vector>

#include <boost/asio/ip/tcp.hpp>

#include <urg_sim/urg_sim.h>

int main(int argc, char** argv)
{
  const urg_sim::URGSimulator::Params params_ust =
      {
          .model = urg_sim::URGSimulator::Model::UST,
          .boot_duration = 0.5,
          .comm_delay_base = 0.0004,
          .comm_delay_sigma = 0.00005,
          .scan_interval = 0.02505,
          .clock_rate = 1.00005,
          .hex_ii_timestamp = false,
          .angle_resolution = 1440,
          .angle_min = 0,
          .angle_max = 1080,
          .angle_front = 540,
      };
  const urg_sim::URGSimulator::Params params_utm =
      {
          .model = urg_sim::URGSimulator::Model::UTM,
          .boot_duration = 2.0,
          .comm_delay_base = 0.0005,
          .comm_delay_sigma = 0.0004,
          .scan_interval = 0.02505,
          .clock_rate = 1.00005,
          .hex_ii_timestamp = false,
          .angle_resolution = 1440,
          .angle_min = 0,
          .angle_max = 1080,
          .angle_front = 540,
      };

  urg_sim::URGSimulator::Params params;

  if (argc < 1 || std::string(argv[1]) == "UTM")
  {
    params = params_utm;
    std::cerr << "Using UTM model" << std::endl;
  }
  else if (std::string(argv[1]) == "UST")
  {
    params = params_ust;
    std::cerr << "Using UST model" << std::endl;
  }
  else
  {
    std::cerr << "Unknown model: " << argv[1] << std::endl;
    return 1;
  }

  urg_sim::URGSimulator sim(
      boost::asio::ip::tcp::endpoint(
          boost::asio::ip::tcp::v4(),
          10940),
      params);
  std::cerr
      << "Listening on :" << sim.getLocalEndpoint().port()
      << std::endl;

  sim.spin();

  return 1;
}
