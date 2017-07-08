
#include "servos.hpp"

#include <cmath>

namespace igm = ignition::math;
namespace gzc = gazebo::common;
namespace gzt = gazebo::transport;

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

void DCModel::run_step(double simtime)
{
    //Stall Torque :  imax = 0.9, Torque = 1.5 N.m => K = 1,66 * red
    //Max speed    :  Rpm max = 1.2 Rps, U = 9.6 v => K = 8 / red
    //red = 2.2
    //std::cout << "[" << Voltage << "]" << std::endl;
    counterElF = Kv * speed * reduction;//speed is in rad/sec => Hz

    current = (Voltage - counterElF - La * didt) / Ra;
    //char f_val[30];    sprintf(f_val,"i=%0.2f ",current);    std::cout << f_val;
    torque = Ki * current * reduction;
    //if(torque != 0){char f_val[30];    sprintf(f_val,"T=%0.2f ",torque);    std::cout << f_val << std::endl;}
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

Servo::Servo()
{
    isPID_Pos = false;
    isPID_Speed = false;
    isPID_Torque = false;
    isAdvertizing = false;
}

void Servo::run_step(double simtime, double batVoltage)
{
    if(type == ServoType::DC)
    {
        //PIDs regulation
        if(isPID_Pos)
        {
            //calculate the error diff
            igm::Angle pid_Target = target;
            igm::Angle errAngle = position - pid_Target;
            //errAngle.Normalize();
            pos_pid->Update(errAngle.Radian(),gzc::Time(simtime));
            //inputs
            dc->control(batVoltage * pos_pid->GetCmd());//[-Vbat , Vbat]
        }
        else if(isPID_Speed)
        {

        }
        else if(isPID_Torque)
        {

        }
        //Electrical Motor simulation
        dc->run_step(simtime);
        //Advertise
        gazebo::msgs::Any pos = gazebo::msgs::ConvertAny(position.Radian());
        pub_test->Publish(pos);
    }
}

void Servo::updatePosition(double pos)
{
    position = pos;//math::Angle <= double
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
    double res = 0.0;
    if(type == ServoType::DC)
    {
        //the Servo handles the voltage inversion, so the consumed current is always positive
        res = fabs(dc->current);
    }
    return res;
}

double Servo::getTorque()
{
    if(type == ServoType::DC)
    {
        return dc->torque;
    }
}

void Servo::Set_ax12a()
{
    type = ServoType::DC;
    dc = new DCModel();
    //@9.6V => 900 mA; R = 10 Ohm
    //L ~ 1 mH
    //K ; @9.6V, 0 Load => U = K . Omega_rps => K = 9.6 / (0.196 sec / 60 deg)
    //0.196 sec / 60 deg => 0.85 Rpsec / reduction => 216 rps
    //Kv = 9.6 / (rps * 254) = 9.6 / 216 = ~ 0.044
    //Ki = (Stall_Torque/254) / i_max = ~ 0.0064
    dc->setParams(10,0.001,0.0064,0.044);//r, l, ki, kv
    //--------------------------------------------------------
    //position => [-Pi , Pi]
    double iMAX = 0.5;
    double cmdMAX = 1;//the regulator will multiply by battery voltage
    pos_pid = new gazebo::common::PID(  3,           // 1/p : Error that brings cmd to max ~ pi/10
                                        2,
                                        1,
                                        iMAX,-iMAX,
                                        cmdMAX,-cmdMAX);
}

void Servo::Advertise_ax12a(const std::string &servo_topic_path)
{
    isAdvertizing = true;
    node = gzt::NodePtr(new gzt::Node());
    node->Init();
    pub_test = node->Advertise<gazebo::msgs::Any>(servo_topic_path+"/pos");
    
}

void Servo::SetPositionTarget(double v_target)
{
    isPID_Pos = true;
    isPID_Speed = false;
    isPID_Torque = false;
    
    target = v_target;
}

void Servo::SetSpeedTarget(double v_target)
{
    isPID_Pos = false;
    isPID_Speed = true;
    isPID_Torque = false;
    
    target = v_target;
}

void Servo::SetTorqueTarget(double v_target)
{
    isPID_Pos = false;
    isPID_Speed = false;
    isPID_Torque = true;
    
    target = v_target;
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

    isPID = true;
}

void ServosController::Set_ax12a(const std::string &jointName)
{
    servos.insert(std::make_pair(jointName,new Servo()));
    servos[jointName]->Set_ax12a();
    std::string servo_topic_path = "/gazebo/default/"+model->GetName()+"/servos/"+jointName;
    servos[jointName]->Advertise_ax12a(servo_topic_path);
    isDC = true;
}


void ServosController::SetServo(const std::string &jointName, const std::string &servoName)
{
    if(servoName.compare("PID") == 0)
    {
        SetPid(jointName);
    }
    else if(servoName.compare("AX12A") == 0)
    {
        Set_ax12a(jointName);
    }
    else if(servoName.compare("AX-GM2212-72Kv") == 0)
    {
        Set_bldc_72(jointName);
        isBLDC = true;
    }
    
}

void ServosController::SetPositionTarget(const std::string &jointName, double target)
{
    Servo &serv = *servos[jointName];
    if(serv.type == ServoType::PID)
    {
        gazebo::physics::JointControllerPtr pj1 = model->GetJointController();
        pj1->SetPositionTarget(jointName,target);
    }
    if(serv.type == ServoType::DC)
    {
        serv.SetPositionTarget(target);
        //std::cout << "PosTarget " << target;
    }
}

void ServosController::SetSpeedTarget(const std::string &jointName, double target)
{
    servos[jointName]->SetSpeedTarget(target);
}

void ServosController::SetTorqueTarget(const std::string &jointName, double target)
{
    servos[jointName]->SetTorqueTarget(target);
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
            gazebo::physics::JointPtr joint = model->GetJoint(servo.first);
            //------------update the motors physics from the simulation------------
            servo.second->updateSpeed(joint->GetVelocity(0));
            //------------update the servo encoder for position regulation------------
            servo.second->updatePosition(joint->Position());
            //---------process the Servo control loop + electrical simulation---------
            //every step needs the new battery level value
            servo.second->run_step(simtime,bat->Voltage());
            //------------inject the generated Force into the simulation--------------
            joint->SetForce(0,servo.second->getTorque());
            //TODO consume the current from the battery
            /*
            std::cout << " Vel " << joint->GetVelocity(0);
            std::cout << " Pos " << joint->Position();
            std::cout << " Volt " << bat->Voltage();
            std::cout << " Tq " << servo.second->getTorque();
            std::cout << " i " << servo.second->getCurrent() << std::endl;
            */
        }
    }
}