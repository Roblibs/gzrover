#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "joystick.hpp"
#include "servos.hpp"

namespace igm = ignition::math;
namespace gzp = gazebo::physics;

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public:
    Joystick 	joy;
    igm::Angle Target;
    igm::Angle twist;
    double Wheels;
    double Turn;
    ServosController servos;
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
      twist = 0;
      Wheels = 0;
      Turn = 0;

      std::string ArmsServo = _sdf->Get<std::string>("ArmsServo");
      std::cout << "ArmsServo : " << ArmsServo << std::endl;
      //---------------------------------------------------------------------
      servos.SetModel(_parent);
      servos.SetServo("rover::j_right_leg",ArmsServo);
      servos.SetServo("rover::j_left_leg",ArmsServo);
      servos.SetServo("rover::j_right_arm",ArmsServo);
      servos.SetServo("rover::j_left_arm",ArmsServo);

      servos.SetPositionTarget("rover::j_right_leg",-Target.Radian());
      servos.SetPositionTarget("rover::j_left_leg",-Target.Radian());
      servos.SetPositionTarget("rover::j_right_arm",Target.Radian());
      servos.SetPositionTarget("rover::j_left_arm",Target.Radian());

    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      bool isUpdated = false;
      if(joy.update())//multiple events will be filtered, only last would appear afterwards
      {
        joy.printUpdates();
        isUpdated = true;
      }
      bool going_up = false;
      JAxis &getup_axis = joy.getAxis(5);
      if(getup_axis.isUpdated())
      {
        Target = igm::Angle::Pi;//init to Pi
        Target *= ((getup_axis.getValue()+1)/2);// 0->1
        std::cout << "Target : " << Target << std::endl;
        going_up = true;
      }
      JAxis &turnup_axis = joy.getAxis(2);
      if(turnup_axis.isUpdated() && !going_up)
      {
        Target = igm::Angle::Pi;//init to Pi
        Target *= -((turnup_axis.getValue()+1)/2);// 0->1
        std::cout << "Target : " << Target << std::endl;
      }
      
      JAxis &twist_axis = joy.getAxis(1);
      if(twist_axis.isUpdated())
      {
        twist = igm::Angle::Pi;//init to Pi
        twist *= twist_axis.getValue();
        std::cout << "Twist : " << twist << std::endl;
      }
      
      JAxis &move_axis = joy.getAxis(4);
      if(move_axis.isUpdated())
      {
        Wheels = move_axis.getValue()*5;
        std::cout << "Wheels : " << Wheels << std::endl;
      }

      joy.consumeAll();
      
      igm::Angle Target_Front = twist - Target;
      igm::Angle Target_Rear = twist + Target;

      servos.SetPositionTarget("rover::j_right_leg",Target_Front.Radian());
      servos.SetPositionTarget("rover::j_left_leg",Target_Front.Radian());
      servos.SetPositionTarget("rover::j_right_arm",Target_Rear.Radian());
      servos.SetPositionTarget("rover::j_left_arm",Target_Rear.Radian());
      servos.update();

      model->GetJoint("j_left_arm_wheel")->SetForce(0,Wheels);
      model->GetJoint("j_right_arm_wheel")->SetForce(0,Wheels);
      model->GetJoint("j_left_leg_wheel")->SetForce(0,Wheels);
      model->GetJoint("j_right_leg_wheel")->SetForce(0,Wheels);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}