#include "vxWorks.h"
#include "taskLib.h"
#include "tickLib.h"
#include "msgQLib.h"

#include "Default.h"
#include "Can.h"
#include "Power.h"


extern MSG_Q_ID MsgPower;
extern MSG_Q_ID MsgCanRx;
extern MSG_Q_ID MsgCan0Tx;

extern struct sCxBUF C0BufTx[CTxBUF_LEN][4];

const U16 Pow24AmpMax[5] = {100,200,300,400};/*0.01A*/
const U16 Pow24AmpMin[5] = {100,100,100,100};
const U16 Pow500AmpMax[5]= {100,200,300,400};/*0.01A*/
const U16 Pow500AmpMin[5]= {100,200,300,400};

void TaskPowCtrl(void)
{
  U32 Mode,Loop;
	U32 MsgLen;
	U32 TickNext,TickDly,TickCur;
  U32 TempU32;
  U32 ErrRxMax;
	U32 P24Ctrl,P50Ctrl;
	U32 P24VolAvg,P24AmpAvg,P50VolAvg,P50AmpAvg;
	U32 P24VolSum,P24AmpSum,P50VolSum,P50AmpSum;
  struct sMSG_USER UMsgPow;
  struct sCxBUF 			*pCanRx;
  struct sCxBUF 			*pC0TxNodeIn;
  struct sCRxPWN_VOL *pCanRxPwnVol;
	struct sCRxPWN_IO 	*pCanRxPwnIo;

	struct sPWN_DAT   P24VolBuf[LLT_PWN_LEN];
	struct sPWN_DAT *pP24VolBufNodeIn;
	struct sPWN_DAT   P24AmpBuf[LLT_PWN_LEN];
	struct sPWN_DAT *pP24AmpBufNodeIn;
	struct sPWN_DAT   P50VolBuf[LLT_PWN_LEN];
	struct sPWN_DAT *pP50VolBufNodeIn;
	struct sPWN_DAT   P50AmpBuf[LLT_PWN_LEN];
	struct sPWN_DAT *pP50AmpBufNodeIn;

  ErrRxMax   = ERR_CAN_RX_MAX;
	P24Ctrl 	= (U32)0x00000000;
	P50Ctrl 	= (U32)0x00000000;


	for(Loop=0;Loop<LLT_PWN_LEN;Loop++)
	{
	//	P24VolBuf[Loop].Next = &P24VolBuf[Loop++];
//		P24AmpBuf[Loop].Next = &P24AmpBuf[Loop++];
	//	P50VolBuf[Loop].Next = &P50VolBuf[Loop++];
	//	P50AmpBuf[Loop].Next = &P50AmpBuf[Loop++];

//		P24VolBuf[Loop].Dat = ((U16)0);
//		P24AmpBuf[Loop].Dat = ((U16)0);
//		P50VolBuf[Loop].Dat = ((U16)0);
//		P50AmpBuf[Loop].Dat = ((U16)0);
	}

//	P24VolBuf[Loop].Next = &P24VolBuf[0];
//	P24AmpBuf[Loop].Next = &P24AmpBuf[0];
//	P50VolBuf[Loop].Next = &P50VolBuf[0];
//	P50AmpBuf[Loop].Next = &P50AmpBuf[0];

//	P24VolBuf[Loop].Dat = ((U16)0);
//	P24AmpBuf[Loop].Dat = ((U16)0);
//	P50VolBuf[Loop].Dat = ((U16)0);
//	P50AmpBuf[Loop].Dat = ((U16)0);

//	pP24VolBufNodeIn = &P24VolBuf[0];
//	pP24AmpBufNodeIn = &P24AmpBuf[0];
//	pP50VolBufNodeIn = &P50VolBuf[0];
//	pP50AmpBufNodeIn = &P50AmpBuf[0];

	P24VolSum = (U32)0;
	P24AmpSum = (U32)0;
	P50VolSum = (U32)0;
	P50AmpSum = (U32)0;
  taskDelay(OS_TIM_100Ms); 

//	pC0TxNodeIn = C0BufTx[C0BUF_POW_CTRL];

	TickDly    = CAN_PERO0_40MS;
  TickNext   = tickGet()+TickDly;

  while(1)
  {
    TickCur = TickNext - tickGet();
    if(TickCur > TickDly)
    {
      TickCur = 0;
    }
    MsgLen = msgQReceive (MsgPower, ((I8 *)&UMsgPow.Typ),OS_MSG_LEN_MAX8B,TickCur);
    switch(MsgLen)
    {
    	case OS_MSG_LEN_MAX8B://Rx Main CMD				
		/*		switch(UMsgPow.Typ)
				{
					case PCMD_V24_ON:
						P24Ctrl |= UMsgPow.Dat;
 						break;
 					case PCMD_V24_OFF:
						P24Ctrl ^= UMsgPow.Dat;
						break;
 					case PCMD_V50_ON:
						P50Ctrl |= UMsgPow.Dat;
						break;
 					case PCMD_V50_OFF:
						P50Ctrl ^= UMsgPow.Dat;
						break;
					default:
						break;
				}*/
      	break;


			case OS_MSG_LEN_MAX4B://Rx Can Data,Judge 24V and 500V  Vol and Amp
				pCanRx = (sCxBUF*)UMsgPow.Typ;
      	pCanRx->Sta = CxBUF_RDING;      
				switch(pCanRx->ID32)
				{
					case C0RxID32_PWN_VOL://VOL
						pCanRxPwnVol = (sCRx_PWN_VOL*)&pCanRx->ID32;
						if((pCanRxPwnVol->P24Vol < P24_VOL_MAX)&&(pCanRxPwnVol->P24Vol > P24_VOL_MIN))
						{
							P24VolSum 					 -= pP24VolBufNodeIn->Dat;
						  pP24VolBufNodeIn->Dat = (U32)pCanRxPwnVol->P24Vol;
							P24VolSum 					 += pP24VolBufNodeIn->Dat;
							P24VolAvg						 	= P24VolSum/16;
						}

						pCanRxPwnVol = (sCRx_PWN_VOL*)&pCanRx->ID32;
						if((pCanRxPwnVol->P24Amp < P24_AMP_MAX)&&(pCanRxPwnVol->P24Amp > P24_AMP_MIN))
						{
							P24AmpSum 					 -= pP24AmpBufNodeIn->Dat;
						  pP24AmpBufNodeIn->Dat = pCanRxPwnVol->P24Amp;
							P24AmpSum 					 += pP24AmpBufNodeIn->Dat;
							P24AmpAvg						 	= P24AmpSum/16;
						}

						pCanRxPwnVol = (sCRx_PWN_VOL*)&pCanRx->ID32;
						if((pCanRxPwnVol->P50Vol < P50_VOL_MAX)&&(pCanRxPwnVol->P50Vol > P50_VOL_MIN))
						{
							P50VolSum 					 -= pP50VolBufNodeIn->Dat;
						  pP50VolBufNodeIn->Dat = pCanRxPwnVol->P50Vol;
							P50VolSum 					 += pP50VolBufNodeIn->Dat;
							P50VolAvg						 	= P50VolSum/16;
						}

						pCanRxPwnVol = (sCRx_PWN_VOL*)&pCanRx->ID32;
						if((pCanRxPwnVol->P50Amp < P50_AMP_MAX)&&(pCanRxPwnVol->P50Amp > P50_AMP_MIN))
						{
							P50AmpSum 					 -= pP50AmpBufNodeIn->Dat;
						  pP50AmpBufNodeIn->Dat = pCanRxPwnVol->P50Amp;
							P50AmpSum 					 += pP50AmpBufNodeIn->Dat;
							P50AmpAvg						 	= P50AmpSum/16;
						}
						break;

					case C0RxID32_PWN_IO://IO
						pCanRxPwnIo = (sCRx_PWN_IO*)&pCanRx->ID32;


						break;
				}
				pCanRx->Sta = CxBUF_NULL;
      
				break;
			default://TimeSlot CanTx
 				TickNext = tickGet() + TickDly;
 				pC0TxNodeIn->Sta 		= CxBUF_WRING;
        pC0TxNodeIn->ID32 	= C0TxID32_PWN_Q;
        pC0TxNodeIn->Dat03 	= 0x00112233;
        pC0TxNodeIn->Dat47 	= 0x44556677;
        pC0TxNodeIn->Sta 		= CxBUF_RDY;
        msgQSend (MsgCan0Tx, (I8 *)&pC0TxNodeIn,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_URGENT);
        pC0TxNodeIn = pC0TxNodeIn->Next;
				break;

		}   
  }
}
