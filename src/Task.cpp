#include <iostream>
#include "Task.h"

using namespace std;

Task::Task(TaskFunction f)
{
    taskFunction = f;
    //(*TaskFunction)();
}

void Task::run()
{
    (*taskFunction)();
}
