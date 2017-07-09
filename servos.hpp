

#include <string>
#include <map>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>

enum class ServoType { PID, DC};

class DCModel
{
public:
    DCModel();
    //Input Control Parameter
    double Voltage;//Applied Voltage V

    //Motor Static Params
    double Ra,La;//Resistance and inductance of the armature circuit
    double Ki;//current constant
    double Kv;//Voltage constant
    double reduction;
    
    //Motor State Params
    double counterElF;//Counter Electromotive Force V
    double current;//Armature current
    double prevcurrent;
    double didt;//current derivate

    //Output result, to apply to the Joint
    double torque;//Motor Torque

    //input from the Physics engine
    double speed;//Rotation (rad/s)
public:
    void step(double dt);
    void reset();
public:
    void setParams(double r,double l, double ki, double kv);    //on init
    void control(double v);                                     //input regulate
};

class Servo
{
public:
    const double min_dt = 1.e-6;//microsecond - it does not make sense to go that far, error likeliness
    const double max_dt = 5.e-3;//milisecond - it gets close to the time constants
public:
    Servo();
    void Set_ax12a();
    void Advertise_ax12a(const std::string &servo_topic_path);
private:
    bool safe_dt(double simtime,double &dt);
    double prevtime;
public:
    ServoType           type;
    gazebo::common::PID *pid;   //Direct Axis Control PID
    DCModel             *dc;

    gazebo::common::PID *pos_pid;
    gazebo::common::PID *speed_pid;
    gazebo::common::PID *torque_pid;
    gazebo::transport::NodePtr node;
    //Publishers
    gazebo::transport::PublisherPtr   pub_pos_target;
    gazebo::transport::PublisherPtr   pub_torque;
    gazebo::transport::PublisherPtr   pub_current;
    //boolean control
    bool    isPID_Pos;
    bool    isPID_Speed;
    bool    isPID_Torque;
    bool    isPublishing;
public:
    //The DC model does not need this because it does not influence the model params
    //relevant for regulation at servo level such as an encoder
    ignition::math::Angle position;
    //One unique target for all of Pos,Speed,Torque
    //As they cannot be regulated simultaneously, switching the control set 
    //a new context for this target variable
    double target;
public:
    void run_step(double simtime, double batVoltage);
public:
    void updatePosition(double pos);//input axis angle !must be called if pos_pid in use!
    void updateSpeed(double o);     //input Speed !must be called if speed_pid in use!
    //-----------------
	void SetPositionTarget(double target);
	void SetSpeedTarget(double target);
	void SetTorqueTarget(double target);
    //-----------------
    double getCurrent();            //output regulate, intermediate loop
    double getTorque();             //output Result
    
};

class ServosController
{
public:
    const int Mechanical_To_Electrical_Simulation_Factor = 1;//only makes sense when pid is active
public:
    ServosController();
private:
    gazebo::physics::ModelPtr model;
    gazebo::common::BatteryPtr bat;
    std::map<std::string,Servo*> servos;
    bool    isPID;
    bool    isDC;
    bool    isBLDC;
public:
	std::string 	Name;
public:
	void SetModel(gazebo::physics::ModelPtr l_model);
    void SetBattery(gazebo::common::BatteryPtr v_bat);
	void SetServo(const std::string &jointName, const std::string &servoName);
	void SetPositionTarget(const std::string &jointName, double target);
	void SetSpeedTarget(const std::string &jointName, double target);
	void SetTorqueTarget(const std::string &jointName, double target);

    void SetPid(const std::string &jointName);
    void Set_ax12a(const std::string &jointName);
    void Set_bldc_72(const std::string &jointName);
    
    void update(double simtime = 0.0);
	
};
