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
    Wait(std_msgs::Bool &msg, std_msgs::Float32 &f32msg) : Task(msg)
    {

    }
    bool initialize() override
    {
        ros::NodeHandle n;
        //ros::Timer ros::NodeHandle::createTimer(ros::Duration(5), &task::callback, bool oneshot = true);
        ros::Timer timer = n.createTimer(ros::Duration(5), &Callback::callback, &callback, true);
    }

     
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "NewNode");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    // Publishers
	ros::Publisher extend_pub = n.advertise<std_msgs::UInt16>("TPortExtendPos", 100);
    ros::Publisher flag_pub = n.advertise<std_msgs::UInt16>("TPortFlagPos", 100);
    ros::Publisher conveyor_current_pub = n.advertise<std_msgs::Float32>("TPortConveyorDrvCurrent", 100);
    ros::Publisher dig_path_pub = n.advertise<std_msgs::Bool>("message", 100);
    ros::Publisher sieve_path_sub = n.advertise<std_msgs::Bool>("message", 100);
    ros::Publisher timer_bool_pub = n.advertise<std_msgs::Bool>("message", 100);
    ros::Publisher timer_float_pub = n.advertise<std_msgs::Float32>("message", 100);

    // Messages
	std_msgs::UInt16 extend_pwr;
    std_msgs::UInt16 flag_pwr;
    std_msgs::Float32 conveyor_pwr;
    std_msgs::Bool to_dig;
    std_msgs::Bool to_sieve;
    std_msgs::Float32 timer_float;
    std_msgs::Bool timer_bool;

    // Message initialization
    extend_pwr.data = 0;
    flag_pwr.data = 0;
    conveyor_pwr.data = 0;
    to_dig.data = 0;
    to_sieve.data = 0;

    // Class instances
    ExtLinAct extLinAct(extend_pwr);
    RetractLinAct retractLinAct(extend_pwr);
    ExtFlags extFlags(flag_pwr);
    StartConveyor startConveyor(conveyor_pwr);
    StopConveyor stopConveyor(conveyor_pwr);
    StartToDig startToDig(to_dig);
    StartToSieve startToSieve(to_sieve);
    Wait wait(timer_bool, timer_float);

    // adding task object to task manager
    // runs in the order listed
    TaskManager tm;
    tm.addTask(extLinAct);
    tm.addTask(retractLinAct);
    tm.addTask(extFlags);
    tm.addTask(startConveyor);
    tm.addTask(stopConveyor);
    tm.addTask(startToDig);
    tm.addTask(startToSieve);
    tm.addTask(wait);
    
    while (ros::ok())
    {
        tm.cycle();

        extend_pub.publish(extend_pwr);
        flag_pub.publish(flag_pwr);
        conveyor_current_pub.publish(conveyor_pwr);
        timer_bool_pub.publish(timer_bool);
        timer_float_pub.publish(timer_float);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}