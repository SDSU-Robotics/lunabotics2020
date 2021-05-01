#ifndef _TASK_LOOP_H
#define _TASK_LOOP_H

#include <list>
#include "Task.h"
#include "ros/ros.h"

class TaskLoop
{
    public:
        std::list<Task*> TaskLoopList;
        static void exitLoop();
        static bool exit;

        TaskLoop();
        void addTask(Task &t);
        Task* getTask(int element);
        Task* getNextTask();
        
    private:
        bool firstTime;
        int taskLoopListElement;
};

#endif