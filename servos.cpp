
#include "servos.hpp"
#include "gazebo/common/Assert.hh"

#include <cmath>

namespace igm = ignition::math;
namespace gzc = gazebo::common;
namespace gzt = gazebo::transport;
namespace gzm = gazebo::msgs;


double clamp(double val, double low, double high) 
{
  return std::max(low, std::min(val, high));
}

DCModel::DCModel()
{
    Voltage = 0;
    Ra = 10;    //10 Ohm
    La = 0.0001; //0.1 mH
    Ki = 0.005;
    Kv = 0.05;
    reduction = 254.0;
    counterElF = 0;
    current = 0;
    didt = 0;
    prevcurrent = 0;
    torque = 0;
    speed = 0;
}

void DCModel::reset()
{
    Voltage = 0;
    counterElF = 0;
    current = 0;
    didt = 0;
    prevcurrent = 0;
    torque = 0;
    speed = 0;
}

void DCModel::step(double dt)
{
    //Stall Torque :  imax = 0.9, Torque = 1.5 N.m => K = 1,66 * red
    //Max speed    :  Rpm max = 1.2 Rps, U = 9.6 v => K = 8 / red
    //red = 2.2
    //std::cout << "[" << Voltage << "]" << std::endl;
    counterElF = Kv * speed * reduction;//speed is in rad/sec => Hz
    if((counterElF < -20)||(counterElF>20))
    {
        //std::cout << "counterElf Too High>counterElF: " << counterElF << " Kv " << Kv << " speed " << speed << " dt " << dt <<" reduction " << reduction << std::endl;
    }
    counterElF = clamp(counterElF,-8,8);

    //current = (Voltage - counterElF - La * didt) / Ra;//didt is not filtred and is too high
    current = (Voltage - counterElF) / Ra;
    if((current < -6)||(current>6))
    {
        std::cout << "current Too High(no didt)>current: "<< current <<" Voltage " << Voltage << " counterElF " << counterElF << " La " << La << " didt " << didt << " Ra " << Ra << std::endl;
    }
    current = clamp(current,-1,1);
    //GZ_ASSERT(((current < -2)||(current>2)), "Current Too High");
    //char f_val[30];    sprintf(f_val,"i=%0.2f ",current);    std::cout << f_val;
    torque = Ki * current * reduction;
    if((torque < -6)||(torque>6))
    {
        std::cout << "torque too High>torque: " << torque <<" Ki " << Ki << " current " << current << " reduction " << reduction << std::endl;
    }
    torque = clamp(torque,-1,1);
    //GZ_ASSERT(((torque < -2)||(torque>2)), "Torque Too High");
    //if(torque != 0){char f_val[30];    sprintf(f_val,"T=%0.2f ",torque);    std::cout << f_val << std::endl;}
    didt = (current - prevcurrent) / dt;
    //store values for next cycle
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

Servo::Servo():speedFilter(6),positionFilter(6),jitterMeasure(6)
{
    isPID_Pos = false;
    isPID_Speed = false;
    isPID_Torque = false;
    isPublishing = false;
    isLogging = false;
    prevtime = 0;
    jitter = 0;
    
}

bool Servo::safe_dt(double simtime, double &dt)
{
    double res = false;
    if(simtime != 0.0)                  //making sure it is a valid time and not the default value
        if(simtime > prevtime)         //making sure the time has advanced so that dt is not 0
        {
            double tdiff = simtime - prevtime;
            if(tdiff >= min_dt)     //plausibilisation, small values are more likely to be errors
                if(tdiff <= max_dt) //plausibilisation, close to time constants, and eliminates gaps
                {
                    dt = tdiff;
                    res = true;
                }
        }
    return res;
}
int valcount = 0;
void Servo::run_step(double simtime, double batVoltage)
{
    if(type == ServoType::DC)
    {
        double dt = 0;
        if(safe_dt(simtime,dt))
        {
            //PIDs regulation
            if(isPID_Pos)
            {
                //calculate the error diff
                igm::Angle pid_Target = target;
                igm::Angle errAngle = position - pid_Target;
                //errAngle.Normalize();
                double e = errAngle.Radian() / 3.141592654;//normalized error ~ [-1 , 1]
                pos_pid->Update(e,gzc::Time(dt));
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
            dc->step(dt);
        }
        else
        {
            char f_val[60];    sprintf(f_val," min_dt=%0.8f ",min_dt);    std::cout << f_val;
            sprintf(f_val," max_dt=%0.8f ",max_dt);    std::cout << f_val;
            sprintf(f_val," dt=%0.8f ",dt);    std::cout << f_val;
            std::cout << "   Simulation Time Not Plausible. simtime: "<< simtime << std::endl;
            //clear accumulated energy
            pos_pid->Reset();

        }
        if(isPublishing)
        {
            pub_pos_target->Publish(gzm::ConvertAny(target));
            pub_torque->Publish(gzm::ConvertAny(getTorque()));
            pub_current->Publish(gzm::ConvertAny(getCurrent()));
            pub_posf->Publish(gzm::ConvertAny(position.Radian()));
            pub_speedf->Publish(gzm::ConvertAny(dc->speed));
        }
        if(isLogging)
        {
            //gzlog << "Torque" << "\t" << "Current" << "\t" << "PosRef" << "\t" << "Positionf" << "\t" << "speedf" << std::endl;
            gzlog << "\t" << name << "\t" << getTorque() << "\t" << getCurrent() << "\t" << target << "\t" << position.Radian() << "\t" << dc->speed << std::endl;

        }
        prevtime = simtime;
    }
}

void Servo::updatePosition(double pos)
{
    //This position is used for the Servo PID, when jittering, the jitter is amplified and applied as torque
    position = positionFilter.add_read(pos);//math::Angle <= double
    //std::cout << "pos: " << pos << " : " << position.Radian();
}

void Servo::updateSpeed(double o)
{
    jitterMeasure.addMeasure(o);
    //the speed is used to create the Counter Electromotive Force, so when jittering with an unrealistic high up and down
    //it results in multiple jittering DC model parameters
    if(type == ServoType::DC)
    {
        dc->speed = speedFilter.add_read(o);
        //std::cout << "speed: " << o << " : " << dc->speed << " : " << std::endl;
    }
}

double Servo::getJitter()
{
    return jitterMeasure.read();
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
    double res = 0.0;
    if(type == ServoType::DC)
    {
        res = dc->torque;
    }
    return res;
}

void Servo::Set_ax12a()
{
    type = ServoType::DC;
    dc = new DCModel();
    //@9.6V => 900 mA; R = 10 Ohm
    //L ~ 0.1 mH
    //K ; @9.6V, 0 Load => U = K . Omega_rps => K = 9.6 / (0.196 sec / 60 deg)
    //0.196 sec / 60 deg => 0.85 Rpsec / reduction => 216 rps
    //Kv = 9.6 / (rps * 254) = 9.6 / 216 = ~ 0.044 - speed coming from phyisics too high
    //Ki = (Stall_Torque/254) / i_max = ~ 0.0064
    dc->setParams(10,0.0001,0.0064,0.008);//r, l, ki, kv
    //--------------------------------------------------------
    //position => [-Pi , Pi]
    double iMAX = 0.8;
    double cmdMAX = 1;//the regulator will multiply by battery voltage
    //3,2,1, 0.5  1
    pos_pid = new gazebo::common::PID(  3,           // 1/p : Error that brings cmd to max ~ pi/10
                                        1,
                                        0.5,
                                        iMAX,-iMAX,
                                        cmdMAX,-cmdMAX);
    pos_pid->Reset();

    isLogging = true;
}

void Servo::Advertise_ax12a(const std::string &servo_topic_path)
{
    isPublishing = false;
    node = gzt::NodePtr(new gzt::Node());
    node->Init();
    pub_pos_target  = node->Advertise<gazebo::msgs::Any>(servo_topic_path+"/posref",100*10000,10000);//100*10000,10000
    pub_torque      = node->Advertise<gazebo::msgs::Any>(servo_topic_path+"/torque",100*10000,10000);
    pub_current     = node->Advertise<gazebo::msgs::Any>(servo_topic_path+"/current",100*10000,10000);
    pub_posf        = node->Advertise<gazebo::msgs::Any>(servo_topic_path+"/posf",100*10000,10000);
    pub_speedf      = node->Advertise<gazebo::msgs::Any>(servo_topic_path+"/speedf",100*10000,10000);
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
    servos[jointName]->pid->Reset();
    gazebo::physics::JointControllerPtr pj1 = model->GetJointController();
    pj1->SetPositionPID(jointName,*servos[jointName]->pid);

    isPID = true;
}

void ServosController::Set_ax12a(const std::string &jointName)
{
    servos.insert(std::make_pair(jointName,new Servo()));
    servos[jointName]->Set_ax12a();
    servos[jointName]->name = jointName;
    std::string servo_topic_path = "/gazebo/default/"+model->GetName()+"/servos/"+jointName;
    servos[jointName]->Advertise_ax12a(servo_topic_path);
    isDC = true;

    gzlog << "\t" << "Name" << "\t" << "Torque" << "\t" << "Current" << "\t" << "PosRef" << "\t" << "Positionf" << "\t" << "speedf" << std::endl;
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

bool ServosController::isTimeTicked(double period)
{
    static double lastTick = 0;
    double timeNow = model->GetWorld()->RealTime().Double();
    bool isRes = false;
    if(lastTick == 0)
    {
        lastTick = timeNow;
        isRes = true;
    }
    else
    {
        if((timeNow-lastTick) > period)
        {
            lastTick = timeNow;
            isRes = true;
        }
    }
    return isRes;
}

void ServosController::update(double simtime)
{
    static int count = 0;
    count++;
    double sumJitter = 0;
    int nb_serv = 0;
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
            sumJitter+=servo.second->getJitter();
            nb_serv++;
        }
    }
    if(isTimeTicked(3.0))
    {
        std::cout << "@ " << count << " ; avg jitter: "<< sumJitter/nb_serv << std::endl;
    }
}