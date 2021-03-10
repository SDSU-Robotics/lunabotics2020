#include "Tasking.h"
#include "Task.h"

using namespace std;

void Tasking::addTask(Task t)
{
    //make a struct
    struct TaskStruct ts = {t};
    // add to list
    TaskList.push_front(ts);
    return;
}