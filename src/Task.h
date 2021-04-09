#ifndef _TASK_H  //Guard
#define _TASK_H

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
using namespace std;            

class Task
{
    public:
        //member functions()
        Task();
        Task(double _xPos, double _zPos, double _yRot, int time, int duration, geometry_msgs::PoseStamped &position);
        Task(std_msgs::Float32 &msg);
        Task(std_msgs::Bool &msg);
       
        virtual bool initialize();
        virtual bool onFinish();
        virtual bool basic();
        virtual bool navigation();
        
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

        // Float32 msg
        std_msgs::Float32 *f32Msg;

        // Bool msg
        std_msgs::Bool *boolMsg;
};
#endif
