#ifndef _TASKING_H  //Guard
#define _TASKING_H

#include <list>
#include "Task.h"

class TaskManager
{
    public:
        void addTask(Task &T);
        Task* getTask(int element);
        bool cycle();

    private:
        int taskListElement = 0;
        bool isTaskRunning = true;
        bool isFirstTime = true;
        bool done = false;
        Task* currentTask;
        std::list<Task*> TaskList;
};

#endif