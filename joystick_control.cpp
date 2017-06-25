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
    gazebo::physics::JointController *pj1;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      printf("Hello gzrover from RobLibs PID Update\n");
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));

    
      joy.start("/dev/input/js0");
      printf("Joystick started\n");
      Target = 0;

      gazebo::common::PID j_pid(
                                10, //P
                                1,  //I
                                0.01,  //D
                                10, //I MAX
                                -10,  //I min
                                30, //cmd MAX
                                -30   //cmd min
                                );
      pj1 = new physics::JointController(_parent);
      pj1->AddJoint(this->model->GetJoint("j_right_leg"));
      pj1->AddJoint(this->model->GetJoint("j_left_leg"));
      pj1->AddJoint(this->model->GetJoint("j_right_arm"));
      pj1->AddJoint(this->model->GetJoint("j_left_arm"));

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
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
      //this->model->GetLink("j_right_leg")->SetForce(ignition::math::Vector3d(10, 0, 0));
      //this->model->GetLink("j_right_leg")->SetForce(0,10);
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