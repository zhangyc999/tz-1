#include "vxWorks.h"
#include "taskLib.h"
#include "msgQLib.h"
#include "semLib.h"
#include "Eth.h"
#include "Default.h"


extern MSG_Q_ID MsgMain;

void TaskEth(void)
{
  U32 Mode;


  Mode = 0x20;
  while(1)
  {
    switch(Mode)
    {
    case 0x00:

      Mode = 0x10;
      break;

    case 0x10:

      Mode = 0x20;
      break;

    case 0x20:

      Mode = 0x21;
      break;

    case 0x21:

      Mode = 0x22;
      break;

    case 0x22:

      Mode = 0x23;
      break;

    case 0x23:

      Mode = 0x88;
      break;
    default:
      break;
    }
    //msgQSend (MsgMain, (I8 *)MsgPtr,OS_MSG_LEN_MAX8B, NO_WAIT, MSG_PRI_NORMAL);
    taskDelay(OS_TIM_5s);
  }
}
