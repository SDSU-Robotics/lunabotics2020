#include "Tasking.h"
#include "Task.h"

using namespace std;
void Tasking::addFunction(taskFunction f)
{
    //make a struct
    TaskStruct ts;
    ts.task = f;
    // add to list
    tasks.push_front(ts);
    return;
}