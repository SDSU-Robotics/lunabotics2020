#ifndef _TASK_H  //Guard
#define _TASK_H

#include "geometry_msgs/PoseStamped.h"

#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <ros/timer.h>
#include "Callback.h"
using namespace std;            

class Task
{
    public:
        //member functions()

        Task(double _xPos, double _zPos, double _yRot, int time, int duration, geometry_msgs::PoseStamped &position);
        Task(std_msgs::UInt16 &msg);
        Task(std_msgs::Float32 &msg);
        Task(std_msgs::Bool &msg);
        Task(bool otherbool, float otherfloat);
        Task();

        virtual bool initialize();
        virtual bool onFinish();
        virtual bool basic();
        virtual bool navigation();
        virtual void callback(const ros::TimerEvent&);

        //Task *task;
        ros::Timer timer;
        ros::NodeHandle *n;
        
        std_msgs::UInt16 *uint16;
        std_msgs::Float32 *float32;
        std_msgs::Bool *boolean;
        bool cbool;
        float cfloat;
        
        enum TaskType
        {
            BASIC,
            NAVIGATION
        };
        TaskType taskType = BASIC;
    
    
        // Data Members
        
        // Navigation Data
        double xPos;
        double zPos;
        double yRot;
        int time;
        int duration;
        geometry_msgs::PoseStamped *posMsg;
/*
        // Float32 msg
        std_msgs::Float32 *f32Msg;

        // Bool msg
        std_msgs::Bool *boolMsg;

        // UInt16 msg
        std_msgs::UInt16 *uint16Msg;
        */
};
#endif
