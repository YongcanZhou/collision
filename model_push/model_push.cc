#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math.hh>

#include "dynamic_simulator.h"

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;  // 	
      this->world = _parent->GetWorld();  // physics::get_world(worldName)

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
      zyc::init_simlator();
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      static double x{0.0}, y{0.0}, z{0.0}, roll{0.0}, pitch{0.0}, yaw{0.0};
      ignition::math::Pose3d pose(x, y, z, roll, pitch, yaw);  // = orig_pose;    
      // this->model->SetWorldPose(pose); //Set the world pose of the entity.
      this->model->SetRelativePose(pose); //Set the pose of the entity relative to its parent.
      zyc::DynamicSimulator(x, y, z, roll, pitch, yaw);
      std::cout <<"pose  "<<pose<<std::endl; 
    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::WorldPtr world;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
