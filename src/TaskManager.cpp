#include "TaskManager.h"
#include "Task.h"
#include "ros/ros.h"

using namespace std;

void TaskManager::addTask(Task t)
{
    // add to list
    TaskList.push_back(&t);
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
            //currentTask = getTask(taskListElement);
            currentTask.initialize();
            isFirstTime = false;
        }
        
        // If current task is done
        if(!isTaskRunning)
        {
            currentTask.onFinish();
            taskListElement++;

            // Check if all tasks are done
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
            switch(currentTask.taskType)
            {
                case Task::BASIC: isTaskRunning = getTask(taskListElement)->basic();
                break;
                case Task::NAVIGATION: isTaskRunning = currentTask.navigation();
                break;
                default: isTaskRunning = currentTask.basic(); 
            }
        }
    }
}



