/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
* LOCAL includes
*/
#include "int.hpp"

namespace naoqi
{
namespace publisher
{

MemoryIntPublisher::MemoryIntPublisher( const std::string& topic ):
  BasePublisher( topic )
{}

void MemoryIntPublisher::publish(const std_msgs::msg::Int32& msg )
{
  pub_->publish( msg );
}

void MemoryIntPublisher::reset( rclcpp::Node* node )
{
  pub_ = node->create_publisher< std_msgs::msg::Int32 >( topic_, 10 );
  is_initialized_ = true;
}

} //publisher
} // naoqi
