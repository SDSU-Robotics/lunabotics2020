#include "TaskManager.h"
#include "Task.h"
#include "ros/ros.h"
#include "TaskLoop.h"
#include <iostream>

using namespace std;

void TaskManager::addTask(Task &T)
{
    // add to list
    TaskStruct ts;
    ts.task = &T;
    ts.isLoop = false;
    TaskList.push_back(*ts);
    return;
}

void TaskManager::addTask(TaskLoop TL)
{
    // add to list
    TaskStruct ts;
    ts.taskLoop = TL;
    ts.isLoop = true;
    TaskList.push_back(*ts);
    return;
}

*TaskStruct *TaskManager::getTask(int element)
{
    // get a pointer to the first element
    auto position = TaskList.begin();
    // move the pointer to the desired element
    advance(position, element);
    
    // Returns task at pointer 
    return *position;   
}

bool TaskManager::cycle()
{
    if(!done)
    {
        // call Initialize
        if(isFirstTime)
        {
            if(getTask(taskListElement).isLoop)
            {
                // Gets the next task from the loop list
                currentTask = getTask(taskListElement).taskLoop.getNextTask();
            }
            else
            {   
                // get next taks from manager list
                currentTask = getTask(taskListElement).task;
            }

            currentTask->initialize();
            isFirstTime = false;
        }
        
        // If current task is done
        if(!isTaskRunning)
        {
            currentTask->onFinish();

            if(getTask(taskListElement).isLoop &&
                getTask(taskListElement).taskLoop.exit)
            {
                taskListElement++;

            }
            else if(!getTask(taskListElement).isLoop)
            {
                taskListElement++;
            }

            //THESE LINES BELOW ARE DUPLICATE, FIX?
            // Check if all tasks are done
            if(taskListElement >= TaskList.size())
            {
                done = true;
                ROS_INFO("Task Manager Done");
            }

            isTaskRunning = true;
            isFirstTime = true;
        }
        else if(getTask(taskListElement).isLoop &&
                getTask(taskListElement).taskLoop.exit)
        {
            taskListElement++;

            //THESE LINES BELOW ARE DUPLICATE, FIX?
            // Check if all tasks are done, Exits Manager if true
            if(taskListElement >= TaskList.size())
            {
                done = true;
                ROS_INFO("Task Manager Done");
            }

            isTaskRunning = true;
            isFirstTime = true;

        }
        else
        {
            // determine Task Type and run correct task
            // ADD NEW TASK FUNCTION TYPES HERE
            switch(currentTask->taskType)
            {
                case Task::BASIC: isTaskRunning = currentTask->basic();
                break;
                case Task::NAVIGATION: isTaskRunning = currentTask->navigation();
                break;
                default: isTaskRunning = currentTask->basic(); 
            }
        }
    }
}



