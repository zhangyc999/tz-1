#include "vxWorks.h"
#include "rngLib.h"
#include "semLib.h"

#include "msgQLib.h"
#include "taskLib.h"
#include "wdLib.h"
#include "eventLib.h"

#include "Default.h"



void TaskLevel(void)
{
  U32 Mode;
  U32 *EventRx;
  STATUS Status;
 // Mode = SWING_IDLE;
  while(1)
  {

    Status = eventReceive(VXEV01,EVENTS_WAIT_ANY ,OS_TIM_100Ms,EventRx);

    switch(Mode)
    {
   // case SWING_H_P:
      break;
   // case SWING_H_N:
      break;
   // case SWING_V_P:
      break;
   // case SWING_V_N:
      break;
  //  case SWING_TEST:
      break;
  //  case SWING_IDLE:
      break;
    default:
      break;
    }
  }
}
