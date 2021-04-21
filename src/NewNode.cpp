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


class Pause : public Task
{
    public: 
        Pause(std_msgs::Bool &msg) : Task(msg) 
        {

        }

    bool basic() override
    {
    //Topic: PausePublisher
    //Message: PauseMsg

    //set message to True
    boolMsg->data = true;

    return false;
    }

};


class EnableTrencherPID : public Task
{
    public: 
    EnableTrencherPID(std_msgs::Bool &msg) : Task(msg) 
    {

    }

    bool basic() override
    {
    //Topic: ExcvTrencherToggle
    //Message: EnableTrencherPIDMsg

    //set message to 1
    f32Msg->data = true;

    return false;
    }

};

class HopperServoOn : public Task
{
    public:
    HopperServoOn(std_msgs::UInt16 &msg) : Task(msg)
    {

    }

    bool basic() override
    {

        //Topic: excvDoorServo
        //Message: excv_door

        uint16 -> data = 5;


        return false;
    }
};

class HopperServoOff : public Task
{
    public:
    HopperServoOff(std_msgs::UInt16 &msg) : Task(msg)
    {

    }

    bool basic() override
    {
        //Topic: excvDoorServo
        //Message: excv_door

        uint16 -> data = 100;


    bool basic() override
    {
        ROS_INFO("Task1 RAN");
        ros::Duration(0.01).sleep();
       
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
    ros::Publisher PausePub = n.advertise<std_msgs::Bool>("PausePublisher", 100);
	ros::Publisher EnableTrencherPIDPub = n.advertise<std_msgs::Bool>("ExcvTrencherToggle", 100);
    ros::Publisher HopperServoPub = n.advertise<std_msgs::UInt16>("excvDoorServo", 100);

    //Message Declarations
    std_msgs::Float32 excvLinActPosMsg;
    std_msgs::Float32 ExcvConveyorEnableMsg;
    std_msgs::Bool PauseMsg;
    std_msgs::Bool EnableTrencherPIDMsg;
    std_msgs::UInt16 excv_door;


    //Message initializations
    excvLinActPosMsg.data = 0;
    ExcvConveyorEnableMsg.data = 0;
    PauseMsg.data = false;
    EnableTrencherPIDMsg.data = false;


    //TaskManager class instance
    TaskManager tm;


    //"Task functions" class instances
    ExtLinAct extLinAct(excvLinActPosMsg);
    StartConveyor startConveyor(ExcvConveyorEnableMsg);
    StopConveyor stopConveyor(ExcvConveyorEnableMsg);
    EnableTrencherPID enableTrencherPID(EnableTrencherPIDMsg);
    HopperServoOn hopperServoOn(excv_door);
    HopperServoOff hopperServoOff(excv_door);


    //adding task object to task manager (will run in order added)
    tm.addTask(extLinAct);
    tm.addTask(startConveyor);
    tm.addTask(stopConveyor);
    tm.addTask(enableTrencherPID);
    tm.addTask(hopperServoOn);
    tm.addTask(hopperServoOff);



    while (ros::ok())
    {
        tm.cycle();
        //TaskLoop::exitLoop();
        

        ExcvLinearActuatorPosPub.publish(excvLinActPosMsg);
        conveyorTogglePub.publish(ExcvConveyorEnableMsg);
        PausePub.publish(PauseMsg);
        EnableTrencherPIDPub.publish(EnableTrencherPIDMsg);
        HopperServoPub.publish(excv_door);


        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



