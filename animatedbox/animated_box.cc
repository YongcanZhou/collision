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
#include <chrono>
#include <vector>

#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        this->model = _parent;

        // create the animation
        gazebo::common::PoseAnimationPtr anim(
              // name the animation "test",
              // make it last 10 seconds,
              // and not set it on a repeat loop
              new gazebo::common::PoseAnimation("test", 10.26536, false));

        gazebo::common::PoseKeyFrame *key;

        // set starting location of the box
        key = anim->CreateKeyFrame(0);
        key->Translation(ignition::math::Vector3d(0, 0, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // // set waypoint location after 2 seconds
        // key = anim->CreateKeyFrame(2.5);
        // key->Translation(ignition::math::Vector3d(-50, -50, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(4.35);
        // key->Translation(ignition::math::Vector3d(10, 20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(6.0666);
        // key->Translation(ignition::math::Vector3d(-10, 20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));


        // key = anim->CreateKeyFrame(8.026);
        // key->Translation(ignition::math::Vector3d(10, -20, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));

        // // set final location equal to starting location
        // key = anim->CreateKeyFrame(10.26536);
        // key->Translation(ignition::math::Vector3d(0, 0, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // // set the animation
        // _parent->SetAnimation(anim);


        static double x{0.0}, y{0.0}, z{0.0};

        // //test---------------------------------------------------------------------
        // for (int i = 0; i <10;i++){
        //     x += 0.5;
        //     y += 0.7;
        //     z += 0.6;
        //   key = anim->CreateKeyFrame(i);
        //   key->Translation(ignition::math::Vector3d(x, y, z));
        //   key->Rotation(ignition::math::Quaterniond(0, 0, i));
        // }
        // // set the animation
        // _parent->SetAnimation(anim);

        // ---------------------------------------------------------------------
        // record start time
        auto start = std::chrono::steady_clock::now();

        static double x{0.0}, y{0.0}, z{0.0};
        while (x == 5){
          int currFrame = anim->GetKeyFrameCount();

          std::cout <<"test time"<<currFrame<<std::endl;

          x += 0.5;
          y += 0.7;
          z += 0.6;
          key = anim->CreateKeyFrame(currFrame+1);
          key->Translation(ignition::math::Vector3d(x, y, z));
          key->Rotation(ignition::math::Quaterniond(0, 0, 1));

          //per 1000ms
          std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000));
          
          // // set the animation
          // _parent->SetAnimation(anim);
        }
        
        // set the animation
        _parent->SetAnimation(anim);

    }
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}
