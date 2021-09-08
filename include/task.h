#ifndef _task_H_
#define _task_H_
#include <FreeRTOS_TEENSY4.h>
#include "read_write_lock.hpp"
class Task
{
public:
    

    int setup(const char *name, UBaseType_t priority);
    // The function you would implement for each task
    // I don't know if this is a great way to do it within RTOS,
    // but it is /a/ way. You can create classes which inherit from
    // `Task` and then just have to implement `start()`, and the
    // rest would "just work"
    virtual int start() = 0;

    static void _real_entry(void *param);

private:
    TaskHandle_t m_handle;
};
#endif