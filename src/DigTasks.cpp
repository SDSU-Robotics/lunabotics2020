#include "ros/ros.h"
#include "std_msgs/Float32.h" 
#include "move_base_msgs/MoveBaseAction.h"

class DigTasks
{
    private:
    // listeners are private
    bool prime;
    bool drvExcvZone;
    bool extLinAct;
    bool excv;
    bool slowAndSteady;
    bool relocate;
    bool reverse;
    bool liftLinAct;
    bool newZone;

    public:
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DigTasks");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

/*
// lifts linear actuator and extends flags
bool DigTasks::prime()
{
    //extend linear actuator
    

    //extend flags
    

}

// drives to excavation zone
bool DigTasks::drvExcvZone()
{

}

// fully extends linear actuator
bool DigTasks::extLinAct()
{
    if (fullyextended==false)
    {
        extend

        if (complete)
        fullyextended==true;
    }

    if (fullyextended=true)
    {excv;}
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
        linearActuator = x;
    }
}

// moves robot to a new spot to start digging again
bool DigTasks::newZone()
{

}*/