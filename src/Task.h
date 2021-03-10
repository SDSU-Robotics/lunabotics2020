#ifndef _TASK_H  //Guard
#define _TASK_H

using namespace std;
typedef bool(*TaskFunction)();

class Task
{
    public:
        //member functions()
        Task(TaskFunction f);
        void run();

    private:
        //Data Members
        TaskFunction taskFunction;
};
#endif
