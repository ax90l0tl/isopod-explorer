/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <math.h>

#include <gz/msgs/posewithcovariance.pb.h>

#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/Util.hh>

#include "depth_sensor.hpp"

using namespace custom;

//////////////////////////////////////////////////
bool Depth_Sensor::Load(const sdf::Sensor &_sdf)
{
  auto type = gz::sensors::customType(_sdf);
  if ("Depth_Sensor" != type)
  {
    gzerr << "Trying to load [Depth_Sensor] sensor, but got type ["
           << type << "] instead." << std::endl;
    return false;
  }

  // Load common sensor params
  gz::sensors::Sensor::Load(_sdf);

  // Advertise topic where data will be published
  this->pub = this->node.Advertise<gz::msgs::PoseWithCovariance>(this->Topic());

  if (!_sdf.Element()->HasElement("gz:Depth_Sensor"))
  {
    gzdbg << "No custom configuration for [" << this->Topic() << "]"
           << std::endl;
    return true;
  }

  // Load custom sensor params
  auto customElem = _sdf.Element()->FindElement("gz:Depth_Sensor");

  if (!customElem->HasElement("noise"))
  {
    gzdbg << "No noise for [" << this->Topic() << "]" << std::endl;
    return true;
  }

  sdf::Noise noiseSdf;
  noiseSdf.Load(customElem->FindElement("noise"));
  this->noise = gz::sensors::NoiseFactory::NewNoiseModel(noiseSdf);
  if (nullptr == this->noise)
  {
    gzerr << "Failed to load noise." << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool Depth_Sensor::Update(const std::chrono::steady_clock::duration &_now)
{
  gz::msgs::PoseWithCovariance msg;
  *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());

  this->depth = this->noise->Apply(this->depth);

  msg.set_data(this->depth);

  this->AddSequence(msg.mutable_header());
  this->pub.Publish(msg);

  return true;
}


//////////////////////////////////////////////////
const gz::math::Vector3d &Depth_Sensor::Position() const
{

  return this->depth;
}