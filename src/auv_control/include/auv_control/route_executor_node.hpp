// Copyright 2026 hanwen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUV_CONTROL__ROUTE_EXECUTOR_NODE_HPP_
#define AUV_CONTROL__ROUTE_EXECUTOR_NODE_HPP_

#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace auv_control
{

std::shared_ptr<rclcpp::Node> make_route_executor_node(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace auv_control

#endif  // AUV_CONTROL__ROUTE_EXECUTOR_NODE_HPP_
