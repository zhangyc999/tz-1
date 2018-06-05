#include "vxWorks.h"
#include "taskLib.h"
#include "msgQLib.h"
#include "semLib.h"
#include "Eth.h"
#include "Default.h"


extern MSG_Q_ID MsgMain;

void TaskTest(void)
{
  U32 Mode;
  sMSG_USER Msg;
 // Msg.Typ =     MSG_TYPE_ETH;
  Mode = 0x20;
  while(1)
  {
    switch(Mode)
    {
    case 0x00:
     // Msg.Dat =ETH_CMD_RAISE0;
      Mode = 0x10;
      break;

    case 0x10:
    //  Msg.Dat =ETH_CMD_RAISE1;
      Mode = 0x20;
      break;

    case 0x20:
    //  Msg.Dat =ETH_CMD_HORIZ1;
      Mode = 0x21;
      break;

    case 0x21:
    //  Msg.Dat =ETH_CMD_VERIT1;
      Mode = 0x22;
      break;

    case 0x22:
     // Msg.Dat =ETH_CMD_PROP1;
      Mode = 0x23;
      break;

    case 0x23:
    //  Msg.Dat =ETH_CMD_LEVE;
      Mode = 0x88;
      break;
    default:
      break;
    }
    msgQSend (MsgMain, (I8 *)&Msg.Typ,OS_MSG_LEN_MAX8B, NO_WAIT, MSG_PRI_NORMAL);
    taskDelay(OS_TIM_5s);
  }
}
