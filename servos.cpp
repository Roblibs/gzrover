
#include "servos.hpp"

DCModel::DCModel()
{
    Voltage = 0;
    Ra = 10;    //10 Ohm
    La = 0.001; //1 mH
    Ki = 0.005;
    Kv = 0.05;
    reduction = 254.0;
    counterElF = 0;
    current = 0;
    prevcurrent = 0;
    torque = 0;
    speed = 0;
    prevtime = 0.0;
}

double DCModel::safe_derive_current(double simtime)
{
    double res = 0;
    if(simtime != 0.0)                  //making sure it is a valid time and not the default value
        if(simtime != prevtime)         //making sure the time has advanced so that dt is not 0
            if(prevtime != 0.0)         //cannot calculate a dt on first step
            {
                double tdiff = simtime - prevtime;
                if(tdiff >= min_dt)     //plausibilisation, small values are more likely to be errors
                    if(tdiff <= max_dt) //plausibilisation, close to time constants, and eliminates gaps
                    {
                        res = (current - prevcurrent) / tdiff;
                    }
            }
    return res;
}

void DCModel::update(double simtime)
{
    //Stall Torque :  imax = 0.9, Torque = 1.5 N.m => K = 1,66 * red
    //Max speed    :  Rpm max = 1.2 Rps, U = 9.6 v => K = 8 / red
    //red = 2.2
    //std::cout << "[" << Voltage << "]" << std::endl;
    counterElF = Kv * speed * reduction;//speed is in rad/sec => Hz

    current = (Voltage - counterElF - La * didt) / Ra;
    char f_val[30];    sprintf(f_val,"i=%0.2f ",current);    std::cout << f_val;
    torque = Ki * current * reduction;
    if(torque != 0)
    {
        char f_val[30];    sprintf(f_val,"T=%0.2f ",torque);    std::cout << f_val << std::endl;
    }

    didt = safe_derive_current(simtime);

    //store values for next cycle
    prevtime = simtime;
    prevcurrent = current;
}

void DCModel::setParams(double r,double l, double ki, double kv)
{
    Ra = r;
    La = l;
    Ki = ki;
    Kv = kv;
}

void DCModel::control(double v)
{
    Voltage = v;
}

void Servo::update(double simtime)
{
    if(type == ServoType::DC)
    {
        dc->update(simtime);
    }
}

void Servo::updateSpeed(double o)
{
    if(type == ServoType::DC)
    {
        dc->speed = o;
    }
}

double Servo::getCurrent()
{
    if(type == ServoType::DC)
    {
        return dc->current;
    }
}

double Servo::getTorque()
{
    if(type == ServoType::DC)
    {
        return dc->torque;
    }
}


ServosController::ServosController()
{
    isPID = false;
    isDC = false;
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
    servos[jointName]->type = ServoType::DC;
    servos[jointName]->dc = new DCModel();
    //@9.6V => 900 mA; R = 10 Ohm
    //L ~ 1 mH
    //K ; @9.6V, 0 Load => U = K . Omega_rps => K = 9.6 / (0.196 sec / 60 deg)
    //0.196 sec / 60 deg => 0.85 Rpsec / reduction => 216 rps
    //Kv = 9.6 / (rps * 254) = 9.6 / 216 = ~ 0.044
    //Ki = (Stall_Torque/254) / i_max = ~ 0.0064
    servos[jointName]->dc->setParams(10,0.001,0.0064,0.044);//r, l, ki, kv
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
        isDC = true;
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
    //TODO remove, for testing purpose set full power for any pos control
    if(servos[jointName]->type == ServoType::DC)
    {
        servos[jointName]->dc->control(bat->Voltage());//currently set to full voltage
    }
}

void ServosController::SetSpeedTarget(const std::string &jointName, double target)
{
    if(servos[jointName]->type == ServoType::DC)
    {

    }
}

void ServosController::SetTorqueTarget(const std::string &jointName, double target)
{
    if(servos[jointName]->type == ServoType::DC)
    {
        servos[jointName]->dc->control(bat->Voltage());//currently set to full voltage
    }
}
	
void ServosController::update(double simtime)
{
    for(auto& servo : servos)
    {
        //std::cout << servo.first << " " <<  << std::endl;
        if(servo.second->type == ServoType::PID)
        {
            model->GetJointController()->Update();
        }
        else if(servo.second->type == ServoType::DC)
        {
            //------------update the motors physics from the simulation------------
            double jspeed = model->GetJoint(servo.first)->GetVelocity(0);
            servo.second->updateSpeed(jspeed);
            //------------------process the electrical simulation------------------
            for(int i=0;i<Mechanical_To_Electrical_Simulation_Factor;i++)
            {
                servo.second->update(simtime / Mechanical_To_Electrical_Simulation_Factor);
            }
            //------------inject the generated Force into the simulation--------------
            model->GetJoint(servo.first)->SetForce(0,servo.second->getTorque());
            //TODO consume the current from the battery
            //servo.second->getCurrent();
        }
    }
}