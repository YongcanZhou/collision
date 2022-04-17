/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>

#include "model_move.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelMove);


/////////////////////////////////////////////////
void ModelMove::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->model = _parent;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->Name());

  // create the animation
  this->anim = gazebo::common::PoseAnimationPtr(
        new gazebo::common::PoseAnimation("test", 5, false));
  // name the animation "test",
  // make it last 5 seconds,
  // and set it not on a repeat loop 

  //set change position
  while (true)
  {
    int currFrame = this->anim->GetKeyFrameCount();
    gazebo::common::PoseKeyFrame *key = this->anim->CreateKeyFrame(++currFrame);

    static double x{0.0}, y{0.0}, z{0.0};
    x += 0.5;
    y += 0.7;
    z += 0.6;

    key->Translation(ignition::math::Vector3d(
          x,
          y,
          z));
    key->Rotation(ignition::math::Quaterniond(0, 0, 0));

  };

  // set the animation
  this->model->SetAnimation(this->anim);

}
