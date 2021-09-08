#include "task.h"

int Task::setup(const char *name, UBaseType_t priority)
{
  UBaseType_t result = xTaskCreate(
      Task::_real_entry,
      name,
      configMINIMAL_STACK_SIZE,
      (void *)this,
      priority,
      &m_handle);

  if (result != pdPass)
  {
    return 1;
  }
  return 0;
}
void Task::_real_entry(void *param)
{

  // Cast argument to class
  Task *self = (Task *)param;

  // Run the main entrypoint
  int result = self->start();

  // Check for errors and handle them relatively gracefully
  if (result != 0)
  {
    //something to trigger all task to stop and crash code
  }

  // Delete the task (this function can't return)
  vTaskDelete(self->m_handle);
}