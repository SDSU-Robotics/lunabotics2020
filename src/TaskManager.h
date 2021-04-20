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
        void recallFunc(int taskListElement, bool done);

        bool cycle();

    private:
        int taskListElement = 0;
        bool isTaskRunning = true;
        bool isFirstTime = true;
        bool done = false;
        bool isDone = false;

        Task* currentTask;
        std::list<Task*> TaskList;
};

#endif