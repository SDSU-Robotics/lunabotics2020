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
        TaskStruct* getTask(int element);

        bool cycle();

    private:
        int taskListElement = 0;
        bool isTaskRunning = true;
        bool isFirstTime = true;
        bool done = false;
        Task* currentTask;
        std::list<TaskStruct*> TaskList;
};

#endif