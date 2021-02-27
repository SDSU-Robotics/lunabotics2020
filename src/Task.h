#ifndef _TASK_H  //Guard
#define _TASK_H
using namespace std;
typedef bool(*TaskFunction)();
class Task
{
    public:

        //member functions()
        TaskFunction taskFunction;
        void setTaskFunction(TaskFunction f);

    private:
         //Data Members

};
#endif
