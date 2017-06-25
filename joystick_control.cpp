#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "joystick.hpp"

namespace igm = ignition::math;
namespace gzp = gazebo::physics;

void regulateJoint(gzp::JointPtr joint,igm::Angle &Target)
{
  igm::Angle e_p = joint->Position();
  igm::Angle e_a = Target - e_p;
  e_a.Normalize();
  double Correction = - e_a.Radian() * 10;
  joint->SetForce(0,Correction);
  //      std::cout << "Position : " << e_p << std::endl;
  //  std::cout << "Error : " << e_a << std::endl;
}

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public:
    Joystick 	joy;
    igm::Angle Target;
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      printf("Hello gzrover from RobLibs\n");
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&ModelPush::OnUpdate, this, _1));

    
      joy.start("/dev/input/js0");
      printf("Joystick started\n");
      Target = igm::Angle::Pi;
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
        Target += igm::Angle::Pi;
        std::cout << "Target : " << Target << std::endl;
      }
      joy.consumeAll();

      //this->model->GetJoint("j_right_leg")->JointC
      regulateJoint(this->model->GetJoint("j_right_leg"),Target);
      regulateJoint(this->model->GetJoint("j_left_leg"),Target);
      igm::Angle minTarget = Target * -1;
      regulateJoint(this->model->GetJoint("j_right_arm"),minTarget);
      regulateJoint(this->model->GetJoint("j_left_arm"),minTarget);

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}