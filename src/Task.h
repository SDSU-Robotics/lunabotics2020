#ifndef _TASK_H  //Guard
#define _TASK_H
#include "geometry_msgs/Pose.h"


using namespace std;


class Task
{
    public:
        //member functions()
        Task();
        virtual bool initialize();
        //TaskBucket taskBucket();
        virtual bool onFinish();
        virtual bool basic();
        virtual bool navigation(geometry_msgs::Pose);
        enum goRun
        {
            BASIC,
            NAVIGATION
        };

    private:
        //Data Members
        
};
#endif
