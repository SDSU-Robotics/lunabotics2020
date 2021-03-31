#include <iostream>
#include "Task.h"

using namespace std;

class BasicTask : public Task
{
    public:
    //place functions here
    virtual bool execute()
    {
        //return false if no longer running
        return false;
    }



};
class NavigationTask : public Task
{
    public:
    //place functions here
    virtual bool execute(int a)
    {
        //return false if no longer running
        return false;
    }

};