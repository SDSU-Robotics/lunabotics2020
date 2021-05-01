#ifndef _TASKING_H  //Guard
#define _TASKING_H

#include <list>
#include "Task.h"
#include "TaskLoop.h"

struct TaskStruct
{
    Task* task;
    TaskLoop taskLoop;

    bool isLoop;
};

class TaskManager
{
    public:
        void addTask(Task &T);
        void addTask(TaskLoop Tl);
        Task* getTask(int element);
        void reset();
        bool cycle();
        bool done = true;
        

    private:
        int taskListElement = 0;
        bool isTaskRunning = true;
        bool isFirstTime = true;
        Task *currentTask;
        std::list<Task*> TaskList;
};

#endif