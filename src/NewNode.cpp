#include "ros/ros.h"
#include "Task.h"
#include "TaskManager.h"
#include <list>
#define NAVIGATION_FRAME "frame"

class NewTask : public Task
{
    public:
    bool basic() override
    {
        ROS_INFO("New Task");

        return false;
    }
};


class ExtLinAct : public Task
{
    //passes the message to Task.cpp
    public: 
        ExtLinAct(std_msgs::Float32 &msg) : Task(msg) 
        {

        }

    bool basic() override
    {
    //Topic: ExcvTrencherPos
    //Message: ExcvLinActPosMsg

    //set message to 1
    f32Msg->data = 1;

    return false;
    }

};


class StartConveyor : public Task
{
    public: 
        StartConveyor(std_msgs::Float32 &msg) : Task(msg) 
        {

        }

    bool basic() override
    {
    //Topic: ExcvConveyorDrvPwr
    //Message: ExcvConveyorEnableMsg

    //set message to 1
    f32Msg->data = 1;

    return false;
    }

};


class StopConveyor : public Task
{
    public: 
        StopConveyor(std_msgs::Float32 &msg) : Task(msg) 
        {

        }

    bool basic() override
    {
    //Topic: ExcvConveyorDrvPwr
    //Message: ExcvConveyorEnableMsg

    //set message to 0
    f32Msg->data = 0;

    return false;
    }

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "NewNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    //Publishers
    ros::Publisher ExcvLinearActuatorPosPub = n.advertise<std_msgs::Float32>("ExcvTrencherPos", 100);
    ros::Publisher conveyorTogglePub = n.advertise<std_msgs::Float32>("ExcvConveyorDrvPwr", 100);

    //Message Declarations
    std_msgs::Float32 excvLinActPosMsg;
    std_msgs::Float32 ExcvConveyorEnableMsg;

    //Message initializations
    excvLinActPosMsg.data = 0;
    ExcvConveyorEnableMsg.data = 0;


    //TaskManager class instance
    TaskManager tm;


    //"Task functions" class instances
    ExtLinAct extLinAct(excvLinActPosMsg);
    StartConveyor startConveyor(ExcvConveyorEnableMsg);
    StopConveyor stopConveyor(ExcvConveyorEnableMsg);


    //adding task object to task manager (will run in order added)
    tm.addTask(extLinAct);


    while (ros::ok())
    {
        tm.cycle();

        ExcvLinearActuatorPosPub.publish(excvLinActPosMsg);
        conveyorTogglePub.publish(ExcvConveyorEnableMsg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



