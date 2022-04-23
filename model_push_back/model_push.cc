#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math.hh>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->world = _parent->GetWorld();  // physics::get_world(worldName)

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // ---------------------------------------------------------------------
      // record start time
      auto start = std::chrono::steady_clock::now();

      static double x{0.0}, y{0.0}, z{0.0}, roll{0.0}, pitch{0.0}, yaw{0.0};
      // std::cout <<"x"<<x<<std::endl;

      ignition::math::Pose3d pose(x, y, z, roll, pitch, yaw);  // = orig_pose;    
      this->model->SetWorldPose(pose);
      // this->model->GetByName    
      // //per 1ms = 0.001s
      // std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));

      static std::thread sim_thread_;  //


      if  ( sim_thread_.joinable())
      {
        return;
      }
      else{
        //using lambda
        sim_thread_ = std::thread([&]()
        {
          // std::chrono::steady_clock
          // 记录开始时间
          auto start = std::chrono::steady_clock::now();

          while (true){
            x += 0.05;
            y += 0.07;
            z += 0.06;
            roll += 0.1;
            pitch += 0.3;
            yaw += 0.2;//mutex  protect

            std::cout<<"test"<<std::endl;
            std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000));
          }
        });
      }
 

      
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
