#include "ros/ros.h"
#include "Task.h"
#include "TaskManager.h"
#include <list>
#include "std_msgs/UInt16.h"
#include <ros/timer.h>
#include <time.h>
#include "Callback.h"


class NewTask : public Task
{
    public:
    bool basic() override
    {
        ROS_INFO("New Task");

        return false;
    }

    //void callback(const ros::TimerEvent&) override
    //{
        
    //}
};

class ExtLinAct : public Task
{
    public:
    ExtLinAct(std_msgs::UInt16 &msg) : Task(msg)
    {

    }   

    bool basic() override
    {
        // Topic: TPortExtendPos
        // Message: extend_pwr

        uint16 -> data = 150;

        return false;
    }
};

class RetractLinAct : public Task
{
    public:
    RetractLinAct(std_msgs::UInt16 &msg) : Task(msg)
    {

    }   

    bool basic() override
    {
        // Topic: TPortExtendPos
        // Message: extend_pwr

        uint16 -> data = 45;

        return false;
    }
};

class ExtFlags : public Task
{
    public:
    ExtFlags(std_msgs::UInt16 &msg) : Task(msg)
    {

    }

    bool basic() override
    {
        // Topic: TPortFlagPos
        // Message: flag_pwr

        uint16 -> data = 140;

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
        // Topic: TPortConveyorDrvCurrent
        // Message: conveyor_pwr
        
        float32 -> data = 1;

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
        // Topic: TPortConveyorDrvCurrent
        // Message: conveyor_pwr
        
        float32 -> data = 0;

        return false;
    }
};

class StartToDig : public Task
{
    public:
    StartToDig(std_msgs::Bool &msg) : Task(msg)
    {

    }

    bool basic() override
    {
        // Topic: 
        // Message: to_dig

        boolean -> data = 1;
    }
};

class StartToSieve : public Task
{
    public:
    StartToSieve(std_msgs::Bool &msg) : Task(msg)
    {

    }

    bool basic() override
    {
        // Topic: 
        // Message: to_sieve

        boolean -> data = 1;
    }
};

class Wait : public Task
{
    public:
    Wait(bool waitbool, float waitfloat, Task &t) : Task(waitbool, waitfloat, t)
    {

    }
    bool initialize() override
    {
        ROS_INFO("bool initialize() override called");
       // timer = n->createTimer(ros::Duration(cfloat), &Task::callback, &this, true);
        timer = n->createTimer(ros::Duration(cfloat), this->callback, true);
        ROS_INFO("Timer made");
    } 
    
    bool basic() override
    {
        return cbool;
    }
};

class Print : public Task
{
    public:
    bool basic() override
    {
        ROS_INFO("Print called");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DugNewNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    // Publishers
	ros::Publisher extend_pub = n.advertise<std_msgs::UInt16>("TPortExtendPos", 100);
    ros::Publisher flag_pub = n.advertise<std_msgs::UInt16>("TPortFlagPos", 100);
    ros::Publisher conveyor_current_pub = n.advertise<std_msgs::Float32>("TPortConveyorDrvCurrent", 100);
    ros::Publisher dig_path_pub = n.advertise<std_msgs::Bool>("message", 100);
    ros::Publisher sieve_path_sub = n.advertise<std_msgs::Bool>("message", 100);
    

    // Messages
	std_msgs::UInt16 extend_pwr;
    std_msgs::UInt16 flag_pwr;
    std_msgs::Float32 conveyor_pwr;
    std_msgs::Bool to_dig;
    std_msgs::Bool to_sieve;

    // Message initialization
    extend_pwr.data = 0;
    flag_pwr.data = 0;
    conveyor_pwr.data = 0;
    to_dig.data = 0;
    to_sieve.data = 0;
    

    // Class instances
    Task timerCall;
    ExtLinAct extLinAct(extend_pwr);
    RetractLinAct retractLinAct(extend_pwr);
    ExtFlags extFlags(flag_pwr);
    StartConveyor startConveyor(conveyor_pwr);
    StopConveyor stopConveyor(conveyor_pwr);
    StartToDig startToDig(to_dig);
    StartToSieve startToSieve(to_sieve);
    Wait wait(true, 2, timerCall);
    Print print;

    // adding task object to task manager
    // runs in the order listed
    TaskManager tm;
    /*tm.addTask(extLinAct);
    tm.addTask(retractLinAct);
    tm.addTask(extFlags);
    tm.addTask(startConveyor);
    tm.addTask(stopConveyor);
    tm.addTask(startToDig);
    tm.addTask(startToSieve);*/
    
    tm.addTask(print);
    tm.addTask(wait);
    tm.addTask(print);
    
    while (ros::ok())
    {
        tm.cycle();

        extend_pub.publish(extend_pwr);
        flag_pub.publish(flag_pwr);
        conveyor_current_pub.publish(conveyor_pwr);
       

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}