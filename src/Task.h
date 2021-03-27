#ifndef _TASK_H  //Guard
#define _TASK_H

#include "geometry_msgs/PoseStamped.h"
using namespace std;            

class Task
{
    public:
        //member functions()
        Task();
        Task(double _xPos, double _zPos,double _yRot, geometry_msgs::PoseStamped &position);
       
        virtual bool initialize();
        virtual bool onFinish();
        virtual bool basic();
        virtual bool navigation();
        
        enum TaskType
        {
            BASIC,
            NAVIGATION
        };
        TaskType taskType = BASIC;;
    
    private:
        // Data Members

        // Navigation Data
        double xPos;
        double zPos;
        double yRot;
        geometry_msgs::PoseStamped *posMsg;
};
#endif
