#include "Tasking.h"
#include "Task.h"

using namespace std;

void Tasking::addTask(Task t)
{
    //make a struct
    struct TaskStruct ts = {t};
    // add to list
    TaskList.push_back(ts);
    return;
}

TaskStruct Tasking::getTask(int element)
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

