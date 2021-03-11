#ifndef _TASKING_H  //Guard
#define _TASKING_H

#include <list>
#include "Task.h"

struct TaskStruct
{
    Task task;
};

class Tasking
{
    public:
        void addTask(Task T);
        TaskStruct getTask(int element);

    private:
        std::list<TaskStruct> TaskList;
};

#endif