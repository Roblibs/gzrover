

#include <string>
#include <map>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

class Servo
{
public:
    std::string type;
    double pgain,igain,dgain,imax,cmd_max;
    gazebo::common::PID *pid;
};

class ServosController
{
private:
    gazebo::physics::ModelPtr model;
    std::map<std::string,Servo*> servos;
public:
	std::string 	Name;
public:
	void SetModel(gazebo::physics::ModelPtr l_model);
	void SetServo(const std::string &jointName, const std::string &servoName);
	void SetPositionTarget(const std::string &jointName, double target);

    void SetPid(const std::string &jointName);
    void Set_ax12a(const std::string &jointName);
    
    void update();
	
};
