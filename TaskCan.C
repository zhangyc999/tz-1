#include "vxWorks.h"
#include "intLib.h"
#include "iv.h"
#include "taskLib.h"
#include "msgQLib.h"
#include "tickLib.h"
#include "semLib.h"
#include "sysLib.h"
#include "logLib.h"
#include "Can.h"
#include "Default.h"

#ifndef _ASMLANGUAGE
	IMPORT UINT8 sysInumTbl[]; // IRQ vs intNum table 
#endif

extern MSG_Q_ID MsgCanRx;
extern MSG_Q_ID MsgCan0Tx;
extern MSG_Q_ID MsgCan1Tx;

extern MSG_Q_ID MsgPower;

extern MSG_Q_ID MsgRaise;
extern MSG_Q_ID MsgSwingH;
extern MSG_Q_ID MsgSwingV;

extern MSG_Q_ID MsgMovFxyz;
extern MSG_Q_ID MsgMovBxyz;

extern MSG_Q_ID MsgTop;
extern MSG_Q_ID MsgShed;
extern MSG_Q_ID MsgMono;
extern MSG_Q_ID MsgLeve;
extern MSG_Q_ID MsgProp;

extern MSG_Q_ID MsgVisl;



void CanInit (U32 AddrBase,I32 Irq, void (*Func)(void));
void C0RxIsr(void);
void C1RxIsr(void);
void CxRxBufInit(void);



struct sCxBUF   C0RxBuf[CRxBUF_LEN];
struct sCxBUF   C1RxBuf[CRxBUF_LEN];
struct sCxBUF *pC0RxNodeIn;
struct sCxBUF *pC1RxNodeIn;



void TaskCanTx(void)
{
  U8 TempU8;
  U8 *pTempU8;
  I32 MsgLen;
  U32 Loop;
  struct sCxBUF *MsgCan;

  CxRxBufInit();
  taskDelay(OS_TIM_500Ms);
  CanInit(ADDR_CMD_CAN0,IRQ5_CAN0,C0RxIsr);
  CanInit(ADDR_CMD_CAN1,IRQ7_CAN1,C1RxIsr);

  while(1)
  {
    taskDelay(OS_TIM_5Ms);

    *(U8 *)ADDR_CMD_CAN0 = PELI_SR;
    TempU8 = *(U8 *)ADDR_DAT_CAN0;
    if((TempU8&0x80) == 0)
    {
      MsgLen = msgQReceive (MsgCan0Tx, ((I8 *)&MsgCan),OS_MSG_LEN_MAX4B,0);
      if((MsgLen == OS_MSG_LEN_MAX4B)&&(MsgCan->Sta == CxBUF_RDY))
      {
        MsgCan->Sta          = CxBUF_RDING;
        MsgCan->TimStamp     = tickGet();

        pTempU8 = (U8*)&MsgCan->FrmInfo;

        *(U8 *)ADDR_CMD_CAN0 = PELI_TXB0;
        *(U8 *)ADDR_DAT_CAN0 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN0 = PELI_TXB4;
        *(U8 *)ADDR_DAT_CAN0 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN0 = PELI_TXB3;
        *(U8 *)ADDR_DAT_CAN0 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN0 = PELI_TXB2;
        *(U8 *)ADDR_DAT_CAN0 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN0 = PELI_TXB1;
        *(U8 *)ADDR_DAT_CAN0 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN0 = PELI_TXB5;
        *(U8 *)ADDR_DAT_CAN0 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN0 = PELI_TXB6;
        *(U8 *)ADDR_DAT_CAN0 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN0 = PELI_TXB7;
        *(U8 *)ADDR_DAT_CAN0 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN0 = PELI_TXB8;
        *(U8 *)ADDR_DAT_CAN0 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN0 = PELI_TXB9;
        *(U8 *)ADDR_DAT_CAN0 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN0 = PELI_TXB10;
        *(U8 *)ADDR_DAT_CAN0 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN0 = PELI_TXB11;
        *(U8 *)ADDR_DAT_CAN0 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN0 = PELI_TXB12;
        *(U8 *)ADDR_DAT_CAN0 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN0 = PELI_CMR;
        *(U8 *)ADDR_DAT_CAN0 = 0x01;

        MsgCan->Sta          = CxBUF_NULL;
      }
      else
      {
        //CAN0 Txd Idle Frame
      }
    }

    *(U8 *)ADDR_CMD_CAN1 = PELI_SR;
    TempU8 = *(U8 *)ADDR_DAT_CAN1;
    if((TempU8&0x80) == 0)
    {
      MsgLen = msgQReceive (MsgCan1Tx, ((I8 *)&MsgCan),OS_MSG_LEN_MAX4B,0);
      if((MsgLen == OS_MSG_LEN_MAX4B)&&(MsgCan->Sta == CxBUF_RDY))
      {
        MsgCan->Sta             = CxBUF_RDING;
        MsgCan->TimStamp        = tickGet();
        pTempU8 = (U8*)&MsgCan->FrmInfo;
        *(U8 *)ADDR_CMD_CAN1 = PELI_TXB0;
        *(U8 *)ADDR_DAT_CAN1 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN1 = PELI_TXB4;
        *(U8 *)ADDR_DAT_CAN1 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN1 = PELI_TXB3;
        *(U8 *)ADDR_DAT_CAN1 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN1 = PELI_TXB2;
        *(U8 *)ADDR_DAT_CAN1 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN1 = PELI_TXB1;
        *(U8 *)ADDR_DAT_CAN1 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN1 = PELI_TXB5;
        *(U8 *)ADDR_DAT_CAN1 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN1 = PELI_TXB6;
        *(U8 *)ADDR_DAT_CAN1 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN1 = PELI_TXB7;
        *(U8 *)ADDR_DAT_CAN1 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN1 = PELI_TXB8;
        *(U8 *)ADDR_DAT_CAN1 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN1 = PELI_TXB9;
        *(U8 *)ADDR_DAT_CAN1 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN1 = PELI_TXB10;
        *(U8 *)ADDR_DAT_CAN1 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN1 = PELI_TXB11;
        *(U8 *)ADDR_DAT_CAN1 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN1 = PELI_TXB12;
        *(U8 *)ADDR_DAT_CAN1 = *pTempU8;pTempU8++;
        *(U8 *)ADDR_CMD_CAN1 = PELI_CMR;
        *(U8 *)ADDR_DAT_CAN1 = 0x01;
        MsgCan->Sta          = CxBUF_NULL;
      }
      else
      {
        //CAN1 Txd Idle Frame
      }
    }
  }
}

void TaskCanRx(void)
{
  U8 TempU8;
  U32 TempU32;
  U32 ErrRxMax;
  sCxBUF *pMsgCan;
  sCxID32 *pCxID32;
  I32 MsgLen;
  ErrRxMax      = ERR_CAN_RX_MAX;

  taskDelay(OS_TIM_100Ms);
  while(1)
  {
    MsgLen = msgQReceive (MsgCanRx, ((I8 *)&pMsgCan),OS_MSG_LEN_MAX4B,OS_TIM_50Ms);
    if(MsgLen != ERROR)
    {
      if(pMsgCan->FrmInfo == 0x88)
      {
        TempU32 = pMsgCan->ID32;
        TempU32 = TempU32>>3;

        pCxID32 = (sCxID32*)&TempU32;

        if(pCxID32->DAddr == ExSAD_HOST)
        {
          ErrRxMax= ERR_CAN_RX_MAX;
          switch(pCxID32->SAddr)
          {
          	case ExSAD_GEN_DIESEL:
          	case ExSAD_GEN_SHAFT:
          	case ExSAD_POW_CTRL:
							msgQSend(MsgPower,  (I8 *)&pMsgCan,OS_MSG_LEN_MAX4B,NO_WAIT,MSG_PRI_NORMAL);
            	break;


          	case ExSAD_RAISE_LF:
          	case ExSAD_RAISE_LB:
          	case ExSAD_RAISE_RF:                                          ;
          	case ExSAD_RAISE_RB:
							msgQSend (MsgRaise, (I8 *)&pMsgCan,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_NORMAL);
   						break;


         	 	case ExSAD_SW_HLF:         
          	case ExSAD_SW_HLB:           
          	case ExSAD_SW_HRF:           
          	case ExSAD_SW_HRB:
							msgQSend (MsgSwingH, (I8 *)&pMsgCan,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_NORMAL);

           	 	break;


         		case ExSAD_SW_VLF:           
          	case ExSAD_SW_VLB:          
          	case ExSAD_SW_VRF:            
          	case ExSAD_SW_VRB:
							msgQSend (MsgSwingV, (I8 *)&pMsgCan,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_NORMAL);
            	break;

          	case ExSAD_PROP_LF:          
          	case ExSAD_PROP_LB:           
          	case ExSAD_PROP_RF:            
          	case ExSAD_PROP_RB:
							msgQSend (MsgProp, (I8 *)&pMsgCan,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_NORMAL);
            	break;

          	case ExSAD_MOV_FX:
						case ExSAD_MOV_FY0:           
          	case ExSAD_MOV_FY1:
				  	case ExSAD_MOV_FZ:
							msgQSend (MsgMovFxyz, (I8 *)&pMsgCan,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_NORMAL);
            	break;

          	case ExSAD_MOV_BX:					
          	case ExSAD_MOV_BY0:           
          	case ExSAD_MOV_BY1:
						case ExSAD_MOV_BZ:
							msgQSend (MsgMovBxyz, (I8 *)&pMsgCan,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_NORMAL);
            	break;


          	case ExSAD_LEVEL0:           
          	case ExSAD_LEVEL1:
							msgQSend (MsgLeve, (I8 *)&pMsgCan,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_NORMAL);
            	break;

          	case ExSAD_VISL_F:           
         	 	case ExSAD_VISL_B:
							msgQSend (MsgVisl, (I8 *)&pMsgCan,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_NORMAL);
            	break;        
      
          	case ExSAD_NM_LF:            
          	case ExSAD_NM_LB:           
          	case ExSAD_NM_RF:            
          	case ExSAD_NM_RB:
							msgQSend (MsgRaise, (I8 *)&pMsgCan,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_NORMAL);
            	break;

          	case ExSAD_TOP:
							msgQSend (MsgTop, (I8 *)&pMsgCan,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_NORMAL);
            	break;

          	case ExSAD_SIDE_LF:           
          	case ExSAD_SIDE_LB:           
          	case ExSAD_SIDE_RF:            
          	case ExSAD_SIDE_RB:
							msgQSend (MsgShed, (I8 *)&pMsgCan,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_NORMAL);
            	break;

          	default:
            	break;
          }
          //Rxd Deal With complete          
        }
      }
    }
    if(ErrRxMax)
    {
      ErrRxMax--;
    }
    else
    {
      CanInit(ADDR_CMD_CAN0,IRQ5_CAN0,C0RxIsr);
      CanInit(ADDR_CMD_CAN1,IRQ7_CAN1,C1RxIsr);
    }
    /*Rxd TimeOver      */
  }
}


void C0RxIsr (void)
{
  U8   TempU8;
  U8 *pTempU8;

  *(U8 *)ADDR_CMD_CAN0 = PELI_IR;
  TempU8 = *(U8 *)ADDR_DAT_CAN0;
  if(TempU8 != 0x01){return;}

  *(U8 *)ADDR_CMD_CAN0 = PELI_SR;
  TempU8 = *(U8 *)ADDR_DAT_CAN0 ;
  if((TempU8 & 0x03) != 0x01){return;}

  if(pC0RxNodeIn->Sta == CxBUF_NULL)
  {
    pC0RxNodeIn->Sta            = CxBUF_WRING;
    pC0RxNodeIn->TimStamp       = tickGet();
    pTempU8 = (U8*)&pC0RxNodeIn->FrmInfo;

    *(U8 *)ADDR_CMD_CAN0        = PELI_RXB0;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN0;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN0        = PELI_RXB4;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN0;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN0        = PELI_RXB3;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN0;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN0        = PELI_RXB2;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN0;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN0        = PELI_RXB1;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN0;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN0        = PELI_RXB5;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN0;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN0        = PELI_RXB6;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN0;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN0        = PELI_RXB7;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN0;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN0        = PELI_RXB8;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN0;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN0        = PELI_RXB9;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN0;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN0        = PELI_RXB10;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN0;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN0        = PELI_RXB11;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN0;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN0        = PELI_RXB12;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN0;
    pTempU8++;

    *(U8 *)ADDR_CMD_CAN0 = PELI_CMR;
    *(U8 *)ADDR_DAT_CAN0 = (U8)0x04;

    pC0RxNodeIn->Sta     = CxBUF_RDY;
    msgQSend (MsgCanRx, (I8 *)&pC0RxNodeIn,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_NORMAL);
    pC0RxNodeIn = pC0RxNodeIn->Next;
  }
  else
  {
    *(U8 *)ADDR_CMD_CAN0 = PELI_CMR;
    *(U8 *)ADDR_DAT_CAN0 = (U8)0x04;
  }
}


void C1RxIsr (void)
{
  U8   TempU8;
  U8 *pTempU8;

  *(U8 *)ADDR_CMD_CAN1 = PELI_IR;
  TempU8 = *(U8 *)ADDR_DAT_CAN1;
  if(TempU8 != 0x01){return;}

  *(U8 *)ADDR_CMD_CAN1 = PELI_SR;
  TempU8 = *(U8 *)ADDR_DAT_CAN1 ;
  if((TempU8 & 0x03) != 0x01){return;}
  if(pC1RxNodeIn->Sta == CxBUF_NULL)
  {
    pC1RxNodeIn->Sta                    = CxBUF_WRING;
    pC1RxNodeIn->TimStamp       = tickGet();

    pTempU8 = (U8*)&pC1RxNodeIn->FrmInfo;

    *(U8 *)ADDR_CMD_CAN1        = PELI_RXB0;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN1;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN1        = PELI_RXB4;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN1;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN1        = PELI_RXB3;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN1;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN1        = PELI_RXB2;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN1;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN1        = PELI_RXB1;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN1;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN1        = PELI_RXB5;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN1;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN1        = PELI_RXB6;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN1;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN1        = PELI_RXB7;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN1;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN1        = PELI_RXB8;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN1;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN1        = PELI_RXB9;
    *pTempU8                   	= *(U8 *)ADDR_DAT_CAN1;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN1        = PELI_RXB10;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN1;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN1        = PELI_RXB11;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN1;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN1        = PELI_RXB12;
    *pTempU8                    = *(U8 *)ADDR_DAT_CAN1;
    pTempU8++;
    *(U8 *)ADDR_CMD_CAN1 = PELI_CMR;
    *(U8 *)ADDR_DAT_CAN1 = (U8)0x04;

    pC1RxNodeIn->Sta     = CxBUF_RDY;

    msgQSend (MsgCanRx, (I8 *)&pC1RxNodeIn,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_NORMAL);
    pC1RxNodeIn = pC1RxNodeIn->Next;
  }
  else
  {
    *(U8 *)ADDR_CMD_CAN1 = PELI_CMR;
    *(U8 *)ADDR_DAT_CAN1 = (U8)0x04;
  }
}


void CanInit (U32 AddrBase,I32 Irq, void (*Func)(void))
{
  sysIntDisablePIC (Irq);

  *(U8 *)(AddrBase) = PELI_MOD;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0x09;

  *(U8 *)(AddrBase) = PELI_CMR;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0x0C;

  *(U8 *)(AddrBase) = PELI_CDR;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0x88;

  *(U8 *)(AddrBase) = PELI_IER;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0x09;

  *(U8 *)(AddrBase) = PELI_ACR0;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0xFF;

  *(U8 *)(AddrBase) = PELI_ACR1;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0xFF;

  *(U8 *)(AddrBase) = PELI_ACR2;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0xFF;

  *(U8 *)(AddrBase) = PELI_ACR3;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0xFF;

  *(U8 *)(AddrBase) = PELI_AMR0;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0xFF;

  *(U8 *)(AddrBase) = PELI_AMR1;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0xFF;

  *(U8 *)(AddrBase) = PELI_AMR2;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0xFF;

  *(U8 *)(AddrBase) = PELI_AMR3;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0xFF;

  *(U8 *)(AddrBase) = PELI_BTR0;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0x04;

  *(U8 *)(AddrBase) = PELI_BTR1;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0x1C;

  *(U8 *)(AddrBase) = PELI_EWLR;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0x60;

  *(U8 *)(AddrBase) = PELI_OCR;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0x1A;

  *(U8 *)(AddrBase) = PELI_MOD;
  *(U8 *)(AddrBase+((U32)0x1000)) = (U8)0x08;


  intConnect(INUM_TO_IVEC(sysInumTbl[(int)Irq]),(VOIDFUNCPTR)Func,0);
  sysIntEnablePIC (Irq);
}


void CxRxBufInit(void)
{
  U32 Loop;
  for(Loop=0;Loop<(CRxBUF_LEN-1);Loop++)
  {
    C0RxBuf[Loop].Next = &C0RxBuf[Loop+1];
    C1RxBuf[Loop].Next = &C1RxBuf[Loop+1];
    C0RxBuf[Loop].Sta  = CxBUF_NULL;
    C1RxBuf[Loop].Sta  = CxBUF_NULL;
  }
  C0RxBuf[Loop].Next = &C0RxBuf[0];
  C1RxBuf[Loop].Next = &C1RxBuf[0];
  C0RxBuf[Loop].Sta  = CxBUF_NULL;
  C1RxBuf[Loop].Sta  = CxBUF_NULL;

  pC0RxNodeIn        = &C0RxBuf[0];
  pC1RxNodeIn        = &C1RxBuf[0];
}

void CxTxBufInit(sCxBUF *pCTxBuf,U32 BufLen,U32 Id32)
{
  U32 Loop;
	struct sCxBUF *pTemp;
	pTemp = pCTxBuf;
 	for(Loop=0;Loop<(BufLen-1);Loop++)
	{
  	pTemp->Sta  		= CxBUF_NULL;
		pTemp->Rec1U8		= (U8)0;
		pTemp->Rec2U8		= (U8)0;
		pTemp->Rec3U8		= (U8)0;
		pTemp->ID32		  = Id32; 
		pTemp->Next			= pTemp++;
		pTemp++;
  }
		pTemp->Sta  		= CxBUF_NULL;
		pTemp->Rec1U8		= (U8)0;
		pTemp->Rec2U8		= (U8)0;
		pTemp->Rec3U8		= (U8)0;
		pTemp->ID32		  = (U32)0; 
		pTemp->Next			= pTemp++;
}	




void LLTabSevoInit(sCRxSERVO *pSevDat,U32 BufLen)
{
	U32 Loop;
	sCRxSERVO *pTemp;
	pTemp = pSevDat;	
  for(Loop=0;Loop<(BufLen-1);Loop++)
  {		
    pTemp->Pos     	= (I16)0;
    pTemp->Spd     	= (I16)0;
    pTemp->Amp     	= (I16)0;
    pTemp->ErrCode 	= (U8)0;
   	pTemp->Sta     	= (U8)0;
    pTemp->Next 		= pTemp++; 
  	pTemp++; 
  }
		pTemp->Pos     	= (I16)0;
    pTemp->Spd     	= (I16)0;
    pTemp->Amp     	= (I16)0;
    pTemp->ErrCode 	= (U8)0;
   	pTemp->Sta     	= (U8)0;
    pTemp->Next 		= pSevDat;     
}

void CxTxSevCmdSpd(sCTxSEV_SPD *pCTxSev,U32 Id32,I16 Speed,I16 Acc,U8 Dly,U8 En)
{
	pCTxSev->ID32		= Id32;
	pCTxSev->RecU16	= (U8)0x1122;
	pCTxSev->Spd		= Speed;
	pCTxSev->Acc		= Acc;
	pCTxSev->Dly		= Dly;
	pCTxSev->En			= En;
}





