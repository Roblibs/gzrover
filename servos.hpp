

#include <string>
#include <map>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

enum class ServoType { PID, AX12A};

class DCModel
{
public:
    DCModel();
    //Input Control Parameter
    double Voltage;//Applied Voltage V

    //Motor Static Params
    double Ra,La;//Resistance and inductance of the armature circuit
    double motorFlux;//Motor Flux
    
    //Motor State Params
    double counterElF;//Counter Electromotive Force V
    double current;//Armature current

    //Output result, to apply to the Joint
    double torque;//Motor Torque

    //input from the Physics engine
    double speed;//Rotation (rad/s)
public:
    void setParams(double r,double l, double f);   //on init
    void control(double v);                        //input regulate
    void updateSpeed(double o);                    //input Speed
    double getCurrent();                           //output regulate, intermediate loop
    double getTorque();                            //output Result
};

class Servo
{
public:
    ServoType           type;
    gazebo::common::PID *pid;
    DCModel             *dc;
public:
    void update();
    
};

class ServosController
{
public:
    ServosController();
private:
    gazebo::physics::ModelPtr model;
    gazebo::common::BatteryPtr bat;
    std::map<std::string,Servo*> servos;
    bool    isPID;
    bool    isAX12A;
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
    
    void update();
	
};
