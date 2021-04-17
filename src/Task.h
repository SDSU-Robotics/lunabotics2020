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
        Task(double _xPos, double _zPos,double _yRot, geometry_msgs::PoseStamped &position);
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

        Task *task;
        
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
    
    private:
        // Data Members

        // Navigation Data
        double xPos;
        double zPos;
        double yRot;
        geometry_msgs::PoseStamped *posMsg;
};
#endif
