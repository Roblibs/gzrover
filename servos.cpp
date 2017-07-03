
#include "servos.hpp"

    std::map<std::string,Servo> servos;

void ServosController::SetModel(gazebo::physics::ModelPtr l_model)
{
    model = l_model;
}

void ServosController::SetPid(const std::string &jointName)
{
    servos.insert(std::make_pair(jointName,new Servo()));
    servos[jointName]->type = "pid";
    //                                              P,I,D, IMAX, Imin, cmdMAX, cmdmin
    servos[jointName]->pid = new gazebo::common::PID(20,10,5,10,-10,50,-50);
    gazebo::physics::JointControllerPtr pj1 = model->GetJointController();
    pj1->SetPositionPID(jointName,*servos[jointName]->pid);
}

void ServosController::Set_ax12a(const std::string &jointName)
{
    //    servos[jointName] = 
}


void ServosController::SetServo(const std::string &jointName, const std::string &servoName)
{
    if(servoName.compare("pid") == 0)
    {
        SetPid(jointName);
    }
    else if(servoName.compare("ax12a") == 0)
    {
        Set_ax12a(jointName);
    }
}

void ServosController::SetPositionTarget(const std::string &jointName, double target)
{
    if(servos[jointName]->type.compare("pid") == 0)
    {
        gazebo::physics::JointControllerPtr pj1 = model->GetJointController();
        pj1->SetPositionTarget(jointName,target);
    }
}
	
void ServosController::update()
{
    //to do check if we have any servo using the Model's JointController
    gazebo::physics::JointControllerPtr pj1 = model->GetJointController();
    pj1->Update();
    //To do handle other sorts of servos
}