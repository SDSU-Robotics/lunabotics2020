#include "TaskManager.h"
#include "Task.h"
#include "ros/ros.h"
#include "TaskLoop.h"
#include <iostream>
#include "JoyMap.h"

using namespace std;

void TaskManager::addTask(Task &T)
{
    // add to list
    TaskList.push_back(&T);
    return;
}

Task* TaskManager::getTask(int element)
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
             // get next task from manager list
            currentTask = getTask(taskListElement);
            //ROS_INFO("A");
            currentTask->initialize();
            //ROS_INFO("A");

            isFirstTime = false;
        }

        // If current task is done
        if(!isTaskRunning)
        {
            currentTask->onFinish();

            //THESE LINES BELOW ARE DUPLICATE, FIX?
            // Check if all tasks are done
            if(taskListElement >= TaskList.size()-1)
            {
                done = true;
                ROS_INFO("Task Manager Done");
            }
            ROS_INFO("TASK DONE");
            taskListElement++;
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

void TaskManager::reset()
{
    //ROS_INFO("reset called");
    TaskList.clear();
    taskListElement = 0;
    isTaskRunning = true;
    isFirstTime = true;
    done = false;
    //cout << done << endl; 
}

