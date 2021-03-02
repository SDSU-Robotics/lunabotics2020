#include "ros/ros.h"
#include "std_msgs/Float32.h" 
//#include "move_base_msgs/MoveBaseAction.h"
#include "GlobalVariables.h"

class DigTasks
{
    private:
        // listeners are private
        bool prime; //put linear actuator in vertical position
        bool drvExcvZone; //traverse straight until excavation zone
        bool extLinAct(); //extend linear actuator to full extension
        bool excv; //lower linear actuator and apply motor current for torque while belt is activated
        bool slowAndSteady; //once linear actuator is 180 degress from prime position, drive forward at constant rate
        bool relocate; //if too close to edge, reverse from trench and begin new trench
        bool reverse; //reverse function for relocating
        bool liftLinAct; //when reversed from wall, lift lin act to vertical position
        bool newTrench; //repeat necessary above functions
        std_msgs::Float32 ExcvLinearActuatorPosMsg; 

    public:
        void setExcvLinearActuatorVar(const std_msgs::Float32 msg);
        float excvLinearActuatorPos = 0;
};


    void DigTasks::setExcvLinearActuatorVar(const std_msgs::Float32 msg)
    {

	    // Set linear actuator position
	    excvLinearActuatorPos = msg.data;
    }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "DigTasks");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    DigTasks digTasks;
	ros::Publisher ExcvLinearActuatorPosPub = n.advertise<std_msgs::Float32>("ExcvTrencherPos", 100);
	ros::Subscriber ExcvLinearActuatorPosSub = n.subscribe("ExcvExtendCurrent", 1000, &DigTasks::setExcvLinearActuatorVar, &digTasks);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

// fully extends linear actuator
bool DigTasks::extLinAct()
{
    bool extending = true;
    excvLinearActuatorPos = GlobalVariables::ExcvMaxPotReading;

    if (excvLinearActuatorPos == GlobalVariables::ExcvMaxPotReading)
    {
        extending = false;
    }

    return extending;
}



/*bool DigTasks::prime()
{
    //lift lin act to vertical
    bool lifting = true;

}

/*
// drives to excavation zone
bool DigTasks::drvExcvZone()
{

}


// excavates
bool DigTasks::excv()
{
    if (switch = false)
    {
        motorCurrent = x;
        beltCurrent = x;
    }
    
    else
    {slowAndSteady;}
}

// excavates while slowly moving forward after switch is flipped
bool DigTasks::slowAndSteady()
{
    
}

// tells robot to relocate because robot has reached a wall/obstacle
bool DigTasks::relocate()
{

}

// reverses robot out of spot
bool DigTasks::reverse()
{
    //if (relocate)
    {

    }
}

// lifts the linear actuator out of the ground to relocate
bool DigTasks::liftLinAct()
{
    //if (relocate)
    {
        PotReading = x;
    }
}

// moves robot to a new spot to start digging again
bool DigTasks::newZone()
{

}*/