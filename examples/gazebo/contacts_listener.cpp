/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>
#include <queue>
#include "prx/utilities/defs.hpp"
#include "prx/planning/world_model.hpp"
#include "prx/planning/planners/rrt.hpp"
#include "prx/gazebo/gz_sim.hpp"
#include "prx/utilities/general/constants.hpp"

using namespace prx;

std::queue<gazebo::msgs::Contact> contacts;
std::vector<std::vector<double>> contacts_wheel_1;
std::vector<std::vector<double>> contacts_wheel_2;
std::vector<std::vector<double>> contacts_wheel_3;
std::vector<std::vector<double>> contacts_wheel_4;

std::vector<std::pair<std::string, double>> rollers_names = 
  { {"roller1::", 1},   {"roller2::", 2},   {"roller3::", 3},   {"roller4::", 4},   {"roller5::", 5},
    {"roller6::", 6},   {"roller7::", 7},   {"roller8::", 8},   {"roller9::", 9},   {"roller10::", 10}, 
    {"roller11::", 11}, {"roller12::", 12}, {"roller13::", 13}, {"roller14::", 14}, {"roller15::", 15}, 
    {"roller16::", 16}, {"roller17::", 17}, {"roller18::", 18}, {"roller19::", 19}, {"roller20::", 20}, 
    {"roller21::", 21} };
/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(ConstContactsPtr &_msg)
{
  // Dump the message contents to stdout.
  for (auto c : _msg -> contact())
  {
      contacts.push(c);
    // std::cout << "found!" << '\n';
    // }
  }
  // std::cout << _msg -> DebugString();
}

void contacts_to_vector()
{
  auto _msg = contacts.front();
  contacts.pop();

  if (_msg.collision1().find("RUmnibot::OmniWheel1") != std::string::npos ||  
      _msg.collision2().find("RUmnibot::OmniWheel1") != std::string::npos ) 
  {
    // std::cout << _msg.DebugString()<< std::endl;
    for (auto roller : rollers_names)
    {
      if (_msg.collision1().find(roller.first) != std::string::npos ||  
          _msg.collision2().find(roller.first) != std::string::npos ) 
      {
        gazebo::common::Time ti (_msg.time().sec(), _msg.time().nsec());
        contacts_wheel_1.push_back({ti.Double(), roller.second, 
          _msg.position()[0].x(), _msg.position()[0].y(), _msg.position()[0].z()});
      }
    }
  }
  else if (_msg.collision1().find("RUmnibot::OmniWheel2") != std::string::npos ||  
           _msg.collision2().find("RUmnibot::OmniWheel2") != std::string::npos ) 
  {
    // std::cout << _msg.DebugString()<< std::endl;
    for (auto roller : rollers_names)
    {
      if (_msg.collision1().find(roller.first) != std::string::npos ||  
          _msg.collision2().find(roller.first) != std::string::npos ) 
      {
        gazebo::common::Time ti (_msg.time().sec(), _msg.time().nsec());
        contacts_wheel_2.push_back({ti.Double(), roller.second, 
          _msg.position()[0].x(), _msg.position()[0].y(), _msg.position()[0].z()});
      }
    }
  }
  else if (_msg.collision1().find("RUmnibot::OmniWheel3") != std::string::npos ||  
           _msg.collision2().find("RUmnibot::OmniWheel3") != std::string::npos ) 
  {
    // std::cout << _msg.DebugString()<< std::endl;
    for (auto roller : rollers_names)
    {
      if (_msg.collision1().find(roller.first) != std::string::npos ||  
          _msg.collision2().find(roller.first) != std::string::npos ) 
      {
        gazebo::common::Time ti (_msg.time().sec(), _msg.time().nsec());
        contacts_wheel_3.push_back({ti.Double(), roller.second, 
          _msg.position()[0].x(), _msg.position()[0].y(), _msg.position()[0].z()});
      }
    }
  }
  else if (_msg.collision1().find("RUmnibot::OmniWheel4") != std::string::npos ||  
           _msg.collision2().find("RUmnibot::OmniWheel4") != std::string::npos ) 
  {
    // std::cout << _msg.DebugString()<< std::endl;
    for (auto roller : rollers_names)
    {
      if (_msg.collision1().find(roller.first) != std::string::npos ||  
          _msg.collision2().find(roller.first) != std::string::npos ) 
      {
        gazebo::common::Time ti (_msg.time().sec(), _msg.time().nsec());
        contacts_wheel_4.push_back({ti.Double(), roller.second, 
          _msg.position()[0].x(), _msg.position()[0].y(), _msg.position()[0].z()});
      }
    }
  }
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node -> Subscribe("/gazebo/default/physics/contacts", cb);

  // Busy wait loop...replace with your own code as needed.
  bool keep_running = true;
  auto c = std::cin.get();
  if (c == 'q')
  {
    std::cout << "to vector!" << std::endl;
  }
  while (contacts.size() != 0)
  {
    // if (contacts.size() > 0)
    // {
      contacts_to_vector();
    // }
    // else
    // {
    //   gazebo::common::Time::MSleep(10);

    // }
  }

  vector_to_file(out_path + "omnibot_trajs/gazebo/wheel1_contacts.txt",
                  contacts_wheel_1, std::ofstream::trunc);
  vector_to_file(out_path + "omnibot_trajs/gazebo/wheel2_contacts.txt",
                  contacts_wheel_2, std::ofstream::trunc);
  vector_to_file(out_path + "omnibot_trajs/gazebo/wheel3_contacts.txt",
                  contacts_wheel_3, std::ofstream::trunc);
  vector_to_file(out_path + "omnibot_trajs/gazebo/wheel4_contacts.txt",
                  contacts_wheel_4, std::ofstream::trunc);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}