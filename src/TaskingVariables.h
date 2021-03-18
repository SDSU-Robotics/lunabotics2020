 #ifndef _TASKINGVARIABLES_H  //Guard
#define _TASKINGVARIABLES_H

struct TaskBucket
{
    class NormalTask
    {
        public:
            virtual bool normal();
    }

    class NavTask
    {
        public:
            virtual bool nav(geometry_msgs::Pose);
    }
};


#endif