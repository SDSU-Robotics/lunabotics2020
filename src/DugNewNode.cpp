#include "ros/ros.h"
#include "Task.h"
#include "TaskManager.h"
#include <list>
#include "std_msgs/UInt16.h"
#include <ros/timer.h>
#include <time.h>
#include "Callback.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include <math.h>

#define PI 3.14159265358979323846
#define WIGGLEROOM 0.5
#define TOLERANCE (PI/64)

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
    Wait(bool waitbool, float waitfloat) : Task(waitbool, waitfloat)
    {

    }
    bool initialize() override
    {
        //ROS_INFO("bool initialize() override called");
       // timer = n->createTimer(ros::Duration(cfloat), &Task::callback, &this, true);
        timer = n->createTimer(ros::Duration(cfloat), boost::bind(&Task::callback, this, _1), true);

        ROS_INFO("Timer made");
    } 
    
    bool basic() override
    {
        
        return cbool;
    }

    void callback(const ros::TimerEvent&) override
    {
        ROS_INFO("callback called");
        cbool = false;
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

class DugOrientation : public Task
{
    public:
    DugOrientation(geometry_msgs::TransformStamped &tf, std_msgs::Float32 &f1, std_msgs::Float32 &f2) : Task(tf, f1, f2)
    {
        
    }

    bool basic() override
    {
         if (transformStamped->transform.rotation.y >= 
         ((PI/2)-atan((transformStamped->transform.translation.z-WIGGLEROOM)/transformStamped->transform.translation.x))+TOLERANCE)
         {
            //write publisher for lspeed and rspeed
            float32 -> data = 0.3;  // left
            float32_2 -> data = 0.6;  // right
            
         }   
         else if (transformStamped->transform.rotation.y >= 
         ((PI/2)-atan((transformStamped->transform.translation.z-WIGGLEROOM)/transformStamped->transform.translation.x))-TOLERANCE)
         {
            float32 -> data = 0.6;  // left
            float32_2 -> data = 0.3;  // right
         }
         else
         {
             return false;
         }
         return true;
    }
};

class DriveForward : public Task
{
    public:
    DriveForward(geometry_msgs::TransformStamped &tf, std_msgs::Float32 &f1, std_msgs::Float32 &f2) : Task(tf, f1, f2)
    {
        
    }
    bool basic() override
    {

        if (transformStamped->transform.translation.x >= 0.01)
        {
            float32 -> data = 0.5;  // left
            float32_2 -> data = 0.5;  // right
        }
        else
        {
            return false;
        }
        return true;
    }
};

class TurnBack : public Task
{
    public:
    TurnBack(geometry_msgs::TransformStamped &tf, std_msgs::Float32 &f1, std_msgs::Float32 &f2) : Task(tf, f1, f2)
    {
        
    }

    bool basic() override
    {
         if (transformStamped->transform.rotation.y >= (tan(transformStamped->transform.translation.x/transformStamped->transform.translation.z)+TOLERANCE))
         {
            //write publisher for lspeed and rspeed
            float32 -> data = -0.3;  // left
            float32_2 -> data = -0.6;  // right
            
         }   
         else if (transformStamped->transform.rotation.y >= (tan((transformStamped->transform.translation.x/transformStamped->transform.translation.z)-TOLERANCE)))
         {
            float32 -> data = -0.6;  // left
            float32_2 -> data = -0.3;  // right
         }
         else
         {
             return false;
         }
         return true;
    }
};

class DriveBack : public Task
{
    public:
    DriveBack(geometry_msgs::TransformStamped &tf, std_msgs::Float32 &f1, std_msgs::Float32 &f2) : Task(tf, f1, f2)
    {
        
    }
    bool basic() override
    {

        if (transformStamped->transform.translation.x >= 0.01)
        {
            float32 -> data = -0.5;  // left
            float32_2 -> data = -0.5;  // right
        }
        else
        {
            return false;
        }
        return true;
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
    ros::Publisher dig_path_pub = n.advertise<std_msgs::Bool>("DigData", 100);
    ros::Publisher sieve_path_pub = n.advertise<std_msgs::Bool>("CollectData", 100);
    

    // Messages
	std_msgs::UInt16 extend_pwr;
    std_msgs::UInt16 flag_pwr;
    std_msgs::Float32 conveyor_pwr;
    std_msgs::Bool to_dig;
    std_msgs::Bool to_sieve;

    std_msgs::UInt16 OnOff; // used for HopperServoOn and HopperServoOff

    std_msgs::Float32 lSpeed;
    std_msgs::Float32 rSpeed;
    geometry_msgs::TransformStamped dugTf;


    // Message initialization
    extend_pwr.data = 0;
    flag_pwr.data = 0;
    conveyor_pwr.data = 0;
    to_dig.data = 0;
    to_sieve.data = 0;
    OnOff.data == 0;
    

    // Class instances
    /*ExtLinAct extLinAct(extend_pwr);
    RetractLinAct retractLinAct(extend_pwr);
    ExtFlags extFlags(flag_pwr);
    StartConveyor startConveyor(conveyor_pwr);
    StopConveyor stopConveyor(conveyor_pwr);
    StartToDig startToDig(to_dig);
    StartToSieve startToSieve(to_sieve);
    HopperServoOn hopperServoOn(OnOff);
    HopperServoOff hopperServoOff(OnOff);
    Wait wait5sec(true, 5);
    Wait wait10sec(true, 10);
    Wait wait15sec(true, 15);
    DigOrientation digAdjust(dugTf, lSpeed, rSpeed);*/

    Task *startToDig;
    Task *digAdjust;
    Task *driveForward;
    Task *hopperServoOn;
    Task *wait5sec;
    Task *hopperServoOff;
    Task *driveBack;
    Task *turnBack;
    Task *startToSieve;
    Task *extLinAct;
    Task *wait10sec;
    Task *startConveyor;
    Task *wait15sec;
    Task *stopConveyor;
    Task *retractLinAct;


    // adding task object to task manager
    // runs in the order listed
    TaskManager tm;
    
    tm.addTask(dugOrientation);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    wait5sec = new Wait(true, 5);
    tm.addTask(*wait5sec);  // adjust time waiting as needed

    while (ros::ok())
    {
        if (tm.done)
        {/*
            startToDig = new StartToDig(to_dig);
            digAdjust = new DigOrientation(dugTf, lSpeed, rSpeed);
            driveForward = new DriveForward(dugTf, lSpeed, rSpeed);
            hopperServoOn = new HopperServoOn(OnOff);
            wait5sec = new Wait(true, 5);
            hopperServoOff = new HopperServoOff(OnOff);
            driveBack = new DriveBack(dugTf, lSpeed, rSpeed);
            turnBack = new TurnBack(dugTf, lSpeed, rSpeed);
            startToSieve = new StartToSieve(to_sieve);
            extLinAct = new ExtLinAct(extend_pwr);
            wait10sec = new Wait(true, 10);
            startConveyor = new StartConveyor(conveyor_pwr);
            wait15sec = new Wait(true, 15);
            stopConveyor = new StopConveyor(conveyor_pwr);
            retractLinAct = new RetractLinAct(extend_pwr);

            tm.reset();
            tm.addTask(*startToDig);
            tm.addTask(*digAdjust);
            tm.addTask(*driveForward);
            tm.addTask(*hopperServoOn);
            tm.addTask(*wait5sec);  // adjust time waiting as needed
            tm.addTask(*hopperServoOff);
            tm.addTask(*driveBack);
            tm.addTask(*turnBack);
            tm.addTask(*startToSieve);
            tm.addTask(*extLinAct);
            tm.addTask(*wait10sec);  // adjust time waiting as needed
            tm.addTask(*startConveyor);
            tm.addTask(*wait15sec);  // adjust time waiting as needed
            tm.addTask(*stopConveyor);
            tm.addTask(*retractLinAct);
            */
            
        }

           

        tm.cycle();

        // Get transform tree
        try
        {
            dugTf = tfBuffer.lookupTransform("map", "ar_marker_5", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            //ROS_WARN("%s", ex.what());
            //ros::Duration(1.0).sleep();
            continue;
        }



        extend_pub.publish(extend_pwr);
        flag_pub.publish(flag_pwr);
        conveyor_current_pub.publish(conveyor_pwr);
        dig_path_pub.publish(to_dig);
        sieve_path_pub.publish(to_sieve);
        //lSpeed.publish();
        //rSpeed.publish();



        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
