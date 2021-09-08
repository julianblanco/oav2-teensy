#ifndef _task_H_
#define _task_H_
#include <FreeRTOS_TEENSY4.h>
#include "read_write_lock.hpp"
class Task
{
public:


    int setup(const char *name, UBaseType_t priority);

    virtual int start() = 0; //method

    static void _real_entry(void *param);

private:
    TaskHandle_t m_handle;
};
#endif