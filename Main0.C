#include "vxWorks.h"
#include "semLib.h"
#include "msgQLib.h"
#include "taskLib.h"
#include "wdLib.h"
#include "Default.h"

extern void TaskMain(void);


#ifdef __cplusplus
extern "C"
{
#endif
  void Main0(void)
  {
    taskSpawn ("TaskMain",OS_PRIO_MAIN,VX_FP_TASK,OS_STACK_SIZE,(FUNCPTR)TaskMain,0,0,0,0,0,0,0,0,0,0);
  }
#ifdef __cplusplus
}
#endif

/*
  void WdogIsr(void)
  {
  wdStart();
  WdogTick  = wdCreate();

  }*/
