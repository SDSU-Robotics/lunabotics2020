#include <iostream>
#include "Task.h"

using namespace std;

bool fa()
{
    cout << "Run";
    return true;
}
void Task::setTaskFunction(TaskFunction f)
{
    taskFunction = f;
    //(*TaskFunction)();
}

int main()
{
    Task t;
    t.setTaskFunction(fa);
    t.taskFunction();



    return 0;

}