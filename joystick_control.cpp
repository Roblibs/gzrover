#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "joystick.hpp"

namespace igm = ignition::math;
namespace gzp = gazebo::physics;

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public:
    Joystick 	joy;
    igm::Angle Target;
    //gazebo::physics::JointController *pj1;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      printf("Hello Rover from Model's SDF PID\n");
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));

    
      joy.start("/dev/input/js0");
      printf("Joystick started\n");
      Target = 0;

      double PID_PGain = _sdf->Get<double>("PID_PGain");
      double PID_IGain = _sdf->Get<double>("PID_IGain");
      double PID_DGain = _sdf->Get<double>("PID_DGain");
      double PID_IMAX = _sdf->Get<double>("PID_IMAX");
      double PID_cmdMAX = _sdf->Get<double>("PID_cmdMAX");

      gazebo::common::PID j_pid(
                                PID_PGain, //P
                                PID_IGain,  //I
                                PID_DGain,  //D
                                PID_IMAX, //I MAX
                                -PID_IMAX,  //I min
                                PID_cmdMAX, //cmd MAX
                                -PID_cmdMAX   //cmd min
                                );
      physics::JointControllerPtr pj1 = this->model->GetJointController();


      pj1->SetPositionPID("rover::j_right_leg",j_pid);
      pj1->SetPositionPID("rover::j_left_leg",j_pid);
      pj1->SetPositionPID("rover::j_right_arm",j_pid);
      pj1->SetPositionPID("rover::j_left_arm",j_pid);
      
      pj1->SetPositionTarget("rover::j_right_leg",-Target.Radian());
      pj1->SetPositionTarget("rover::j_left_leg",-Target.Radian());
      pj1->SetPositionTarget("rover::j_right_arm",Target.Radian());
      pj1->SetPositionTarget("rover::j_left_arm",Target.Radian());
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      bool isUpdated = false;
      if(joy.update())//multiple events will be filtered, only last would appear afterwards
      {
        //joy.printUpdates();
        isUpdated = true;
      }
      JAxis &axis = joy.getAxis(4);
      if(axis.isUpdated())
      {
        Target = igm::Angle::Pi;//init to Pi
        Target *= axis.getValue();
        std::cout << "Target : " << Target << std::endl;
      }
      joy.consumeAll();
      
      physics::JointControllerPtr pj1 = this->model->GetJointController();
      pj1->SetPositionTarget("rover::j_right_leg",-Target.Radian());
      pj1->SetPositionTarget("rover::j_left_leg",-Target.Radian());
      pj1->SetPositionTarget("rover::j_right_arm",Target.Radian());
      pj1->SetPositionTarget("rover::j_left_arm",Target.Radian());
      pj1->Update();
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}