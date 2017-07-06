
#include "servos.hpp"

DCModel::DCModel()
{
    Voltage = 0;
    Ra = 1;
    La = 1;
    motorFlux = 1;
    counterElF = 0;
    current = 0;
    torque = 0;
    speed = 0;
}

void Servo::update()
{

}

ServosController::ServosController()
{
    isPID = false;
    isAX12A = false;
    isBLDC = false;
}

void ServosController::SetModel(gazebo::physics::ModelPtr l_model)
{
    model = l_model;
}

void ServosController::SetBattery(gazebo::common::BatteryPtr v_bat)
{
    bat = v_bat;
}

void ServosController::SetPid(const std::string &jointName)
{
    servos.insert(std::make_pair(jointName,new Servo()));
    servos[jointName]->type = ServoType::PID;
    //                                              P,I,D, IMAX, Imin, cmdMAX, cmdmin
    //servos[jointName]->pid = new gazebo::common::PID(20,10,5,10,-10,50,-50);
    double cmdMAX = 1.47;//15 Kg cm => 1.47 N.m
    servos[jointName]->pid = new gazebo::common::PID(3,2,1,0.5,-0.5,cmdMAX,-cmdMAX);
    gazebo::physics::JointControllerPtr pj1 = model->GetJointController();
    pj1->SetPositionPID(jointName,*servos[jointName]->pid);
}

void ServosController::Set_ax12a(const std::string &jointName)
{
    servos.insert(std::make_pair(jointName,new Servo()));
    servos[jointName]->type = ServoType::AX12A;
    servos[jointName]->dc = new DCModel();
}


void ServosController::SetServo(const std::string &jointName, const std::string &servoName)
{
    if(servoName.compare("PID") == 0)
    {
        SetPid(jointName);
        isPID = true;
    }
    else if(servoName.compare("AX12A") == 0)
    {
        Set_ax12a(jointName);
        isAX12A = true;
    }
    else if(servoName.compare("AX-GM2212-72Kv") == 0)
    {
        Set_bldc_72(jointName);
        isBLDC = true;
    }
    
}

void ServosController::SetPositionTarget(const std::string &jointName, double target)
{
    if(servos[jointName]->type == ServoType::PID)
    {
        gazebo::physics::JointControllerPtr pj1 = model->GetJointController();
        pj1->SetPositionTarget(jointName,target);
    }
}

void ServosController::SetSpeedTarget(const std::string &jointName, double target)
{
    if(servos[jointName]->type == ServoType::AX12A)
    {

    }
}

void ServosController::SetTorqueTarget(const std::string &jointName, double target)
{
    if(servos[jointName]->type == ServoType::AX12A)
    {
        servos[jointName]->dc->control(bat->Voltage());//currently set to full voltage
    }
}

	
void ServosController::update()
{
    for(auto& servo : servos)
    {
        if(servo.second->type == ServoType::PID)
        {
            model->GetJointController()->Update();
        }
        else// all other types of servos do a self update of parameters
        {
            servo.second->update();
            if(servo.second->type == ServoType::AX12A)
            {
                model->GetJoint(servo.first)->SetForce(0,servo.second->dc->getTorque());
            }
        }
    }
}