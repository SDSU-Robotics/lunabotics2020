#include "TaskManager.h"
#include "Task.h"

using namespace std;

void Tasking::addTask(Task t)
{
    // add to list
    TaskList.push_back(t);
    return;
}

Task Tasking::getTask(int element)
{
    // get a pointer to the first element
    auto position = TaskList.begin();
    // move the pointer to the desired element
    advance(position, element);
    
    // Returns task struct at pointer 
    return *position;   
}

void callAndRun ()
{
    //callInitialize
    //runExecute
}

