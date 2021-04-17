#ifndef _TASK_LOOP_H
#define _TASK_LOOP_H

#include <list>
#include "Task.h"

class TaskLoop
{
    private:
        bool exit;
    
    public:
        std::list<Task*> TaskLoopList;
};

#endif