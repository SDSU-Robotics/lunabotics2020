#ifndef _TASKING_H  //Guard
#define _TASKING_H

#include <list>
#include "Task.h"

class Tasking
{
    public:
        void addTask(Task T);
        Task getTask(int element);

    private:
        std::list<Task> TaskList;
};

#endif