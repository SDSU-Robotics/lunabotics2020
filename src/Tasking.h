#ifndef _TASKING_H  //Guard
#define _TASKING_H
#include <list>
#include "Task.h"
class Tasking{

    public:
    void addFunction (TaskFunction F);


    private:
    struct TaskStruct 
    {
        Task task;
    };
    std::list<TaskStruct> tasks;

}; 

#endif