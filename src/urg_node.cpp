/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Chad Rockey, Michael Carroll, Mike O'Driscoll
 */
 #include <chrono>
 #include <memory>
 #include <sstream>
#include "urg_node/urg_node_driver.h"

// boost headers
#include <boost/lexical_cast.hpp>

// rcutils headers
#include <rcutils/cmdline_parser.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

void on_parameter_event(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event, rclcpp::Logger logger)
{
  // TODO(wjwwood): The message should have an operator<<, which would replace all of this.
  std::stringstream ss;
  if(!event->new_parameters.empty())
  {
    ss << "\nParameter event:\n new parameters:";
    for (auto & new_parameter : event->new_parameters) {
      ss << "\n  " << new_parameter.name;
    }
  }
  if(!event->changed_parameters.empty())
  {
    ss << "\n changed parameters:";
    for (auto & changed_parameter : event->changed_parameters) {
      ss << "\n  " << changed_parameter.name;
    }
  }
  if(!event->deleted_parameters.empty())
  {
    ss << "\n deleted parameters:";
    for (auto & deleted_parameter : event->deleted_parameters) {
      ss << "\n  " << deleted_parameter.name;
    }
  }
  ss << "\n";
  RCLCPP_INFO(logger, ss.str().c_str())
}


int main(int argc, char **argv)
{
  // Initialize node and nodehandles
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("urg_node");


  //Create a Sync param service client, each node already has a param service by default
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node);
  std::chrono::seconds sec(1);
  while (!parameters_client->wait_for_service(sec)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.")
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...")
  }


  // Setup callback for changes to parameters.
  auto sub_params = parameters_client->on_parameter_event(
    [node](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
      on_parameter_event(event, node->get_logger());
    });

  // Support the optional serial port command line argument
  /*
  *Switched to using parameters loaded via a yaml file, rather than command line arguments
  *
  *
  std::string serialPort = "/dev/ttyACM0";
  std::string option = "--serial-port";
  if (rcutils_cli_option_exist(argv, argv + argc, option.c_str())) {
    serialPort = rcutils_cli_get_option(argv, argv + argc, option.c_str());
  }

  // Support the optional user latency command line argument
  double userLatency = 0;
  option = "--user-latency";
  if (rcutils_cli_option_exist(argv, argv + argc, option.c_str())) {
    std::string strLatency = rcutils_cli_get_option(argv, argv + argc, option.c_str());
    userLatency = boost::lexical_cast<double>(strLatency);
  }

  // Support the optional IP address command line argument
  std::string ipAddress = "";
  option = "--ip-addr";
  if (rcutils_cli_option_exist(argv, argv + argc, option.c_str())) {
    ipAddress = rcutils_cli_get_option(argv, argv + argc, option.c_str());
  }

  // Support the optional IP port command line argument
  int ipPort = 0;
  option = "--port";
  if (rcutils_cli_option_exist(argv, argv + argc, option.c_str())) {
    std::string strIPPort = rcutils_cli_get_option(argv, argv + argc, option.c_str());
    ipPort = boost::lexical_cast<int>(strIPPort);
  }

  // Support the optional laser frame id command line argument
  std::string laserFrameId = "laser";
  option = "--laser-frame-id";
  if (rcutils_cli_option_exist(argv, argv + argc, option.c_str())) {
    laserFrameId = rcutils_cli_get_option(argv, argv + argc, option.c_str());
  }
  */

  urg_node::UrgNode urgNode;


  // Update settings
  urgNode.setSerialPort(parameters_client->get_parameter<std::string>("serial_port","/dev/ttyACM0"));
  urgNode.setUserLatency(parameters_client->get_parameter<double>("user_latency",0.0));
  urgNode.setIPAdddress(parameters_client->get_parameter<std::string>("ip_address",""));
  urgNode.setIPPort(parameters_client->get_parameter<int>("ip_port",0));
  urgNode.setLaserFrameId(parameters_client->get_parameter<std::string>("laser_frame_id","laser"));

  // Run the urg node
  urgNode.run();

  rclcpp::spin(node);

  return 0;
}
