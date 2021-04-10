#ifndef _TASK_H  //Guard
#define _TASK_H

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
using namespace std;            

class Task
{
    public:
        //member functions()
        Task();
        Task(double _xPos, double _zPos,double _yRot, geometry_msgs::PoseStamped &position);
        Task(std_msgs::UInt16 &msg);
        Task(std_msgs::Float32 &msg);

        virtual bool initialize();
        virtual bool onFinish();
        virtual bool basic();
        virtual bool navigation();
        virtual void callback();
        virtual bool timer();

        std_msgs::UInt16 *uint16;
        std_msgs::Float32 *float32;
        
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
