

#include <string>
#include <map>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

enum class ServoType { PID, AX12A};

class Servo
{
public:
    ServoType type;
    gazebo::common::PID *pid;
    
};

class ServosController
{
public:
    ServosController();
private:
    gazebo::physics::ModelPtr model;
    std::map<std::string,Servo*> servos;
    bool    isPID;
    bool    isAX12A;
    bool    isBLDC;
public:
	std::string 	Name;
public:
	void SetModel(gazebo::physics::ModelPtr l_model);
	void SetServo(const std::string &jointName, const std::string &servoName);
	void SetPositionTarget(const std::string &jointName, double target);

    void SetPid(const std::string &jointName);
    void Set_ax12a(const std::string &jointName);
    void Set_bldc_72(const std::string &jointName);
    
    void update();
	
};
