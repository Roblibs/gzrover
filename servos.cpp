
#include "servos.hpp"

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
    }
    else if(servoName.compare("AX-GM2212-72Kv") == 0)
    {
        Set_bldc_72(jointName);
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
	
void ServosController::update()
{
    if(isPID)
    {
        gazebo::physics::JointControllerPtr pj1 = model->GetJointController();
        pj1->Update();
    }
    //To do handle other sorts of servos
}