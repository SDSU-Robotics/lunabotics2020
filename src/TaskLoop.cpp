#include "TaskLoop.h"
#include "ros/ros.h"
#include <iostream>

bool TaskLoop::exit; //Static member must be redeclared

TaskLoop::TaskLoop()    
{
    exit = false;
    taskLoopListElement = 0;
    firstTime = true;
}

void TaskLoop::addTask(Task &t)
{
    // add to list
    TaskLoopList.push_back(&t);
}

void TaskLoop::exitLoop()
{
    exit = true;
}

Task* TaskLoop::getTask(int element)
{
    // get a pointer to the first element
    auto position = TaskLoopList.begin();
    // move the pointer to the desired element
    advance(position, element);
    
    // Returns task at pointer 
    return *position;   
}

Task* TaskLoop::getNextTask()
{
    Task* t;
    std::cout << taskLoopListElement << std::endl;
    std::cout << TaskLoopList.size() << std::endl;

    if(firstTime)
    {
        t = getTask(0);
        firstTime = false;
        ROS_INFO("FIRST");
    }
    else
    {
        if(taskLoopListElement >= (TaskLoopList.size() - 1))
        {
            taskLoopListElement = 0;
            t = getTask(taskLoopListElement);
            ROS_INFO("A");
        }
        else
        {
            taskLoopListElement++;
            //taskLoopListElement++;
            t = getTask(taskLoopListElement);
            ROS_INFO("B");
        }
    }
    
    return t;
}