#include "vxWorks.h"
#include "rngLib.h"
#include "taskLib.h"
#include "semLib.h"
#include "msgQLib.h"
#include "wdLib.h"
#include "eventLib.h"

#include "Default.h"
#include "Power.h"

extern void TaskSwingH(void);
extern void TaskCanTx(void);
extern void TaskCanRx(void);

MSG_Q_ID MsgMain;

MSG_Q_ID MsgCanRx;
MSG_Q_ID MsgCan0Tx;
MSG_Q_ID MsgCan1Tx;
MSG_Q_ID MsgEth;
MSG_Q_ID MsgPower;

MSG_Q_ID MsgRaise;
MSG_Q_ID MsgSwingH;
MSG_Q_ID MsgSwingV;

MSG_Q_ID MsgMovFxyz;
MSG_Q_ID MsgMovBxyz;


MSG_Q_ID MsgTop;
MSG_Q_ID MsgShed;
MSG_Q_ID MsgMono;
MSG_Q_ID MsgLeve;
MSG_Q_ID MsgProp;
MSG_Q_ID MsgVisl;



void TaskMain(void)
{

  U32 Mode;
  U32 MsgRx;
	U32 MsgLen;
	U32 TimeIdle;
	U32 StaTask,FlgTemp;
  struct sMSG_USER UMsgTx;  
  struct sMSG_USER UMsgRx; 
  MsgMain     = msgQCreate(OS_MSG_QUEU_MAX256, OS_MSG_LEN_MAX8B, MSG_Q_FIFO);  
  MsgPower    = msgQCreate(OS_MSG_QUEU_MAX32, OS_MSG_LEN_MAX8B, MSG_Q_FIFO);

  MsgCanRx    = msgQCreate(OS_MSG_QUEU_MAX128,OS_MSG_LEN_MAX4B, MSG_Q_FIFO);
  MsgCan0Tx 	= msgQCreate(OS_MSG_QUEU_MAX64,	OS_MSG_LEN_MAX4B, MSG_Q_FIFO);
  MsgCan1Tx	 	= msgQCreate(OS_MSG_QUEU_MAX64,	OS_MSG_LEN_MAX4B, MSG_Q_FIFO);

  MsgSwingH   = msgQCreate(OS_MSG_QUEU_MAX32, OS_MSG_LEN_MAX8B, MSG_Q_FIFO);
	MsgSwingV		= msgQCreate(OS_MSG_QUEU_MAX32, OS_MSG_LEN_MAX8B, MSG_Q_FIFO);
	MsgRaise		= msgQCreate(OS_MSG_QUEU_MAX32, OS_MSG_LEN_MAX8B, MSG_Q_FIFO);
	

	MsgMovFxyz	= msgQCreate(OS_MSG_QUEU_MAX32, OS_MSG_LEN_MAX8B, MSG_Q_FIFO);
	MsgMovBxyz	= msgQCreate(OS_MSG_QUEU_MAX32, OS_MSG_LEN_MAX8B, MSG_Q_FIFO);

	
	MsgTop			= msgQCreate(OS_MSG_QUEU_MAX32, OS_MSG_LEN_MAX8B, MSG_Q_FIFO);
	MsgShed			= msgQCreate(OS_MSG_QUEU_MAX32, OS_MSG_LEN_MAX8B, MSG_Q_FIFO);
	MsgMono			= msgQCreate(OS_MSG_QUEU_MAX32, OS_MSG_LEN_MAX8B, MSG_Q_FIFO);
	MsgLeve			= msgQCreate(OS_MSG_QUEU_MAX32, OS_MSG_LEN_MAX8B, MSG_Q_FIFO);
	MsgProp			= msgQCreate(OS_MSG_QUEU_MAX32, OS_MSG_LEN_MAX8B, MSG_Q_FIFO);
	MsgVisl			= msgQCreate(OS_MSG_QUEU_MAX32, OS_MSG_LEN_MAX8B, MSG_Q_FIFO);

  taskSpawn ("TaskCanTx",	OS_PRIO_CANTX,	VX_FP_TASK,OS_STACK_SIZE,(FUNCPTR)TaskCanTx,	0,0,0,0,0,0,0,0,0,0);
  taskSpawn ("TaskCanRx",	OS_PRIO_CANRX,	VX_FP_TASK,OS_STACK_SIZE,(FUNCPTR)TaskCanRx,	0,0,0,0,0,0,0,0,0,0);
  taskSpawn ("TaskSwingH",OS_PRIO_SWINGH,	VX_FP_TASK,OS_STACK_SIZE,(FUNCPTR)TaskSwingH,	0,0,0,0,0,0,0,0,0,0);


	taskDelay(OS_TIM_100Ms);
	TimeIdle 	= (U32)0;
	Mode = MMOD_IDLE;
	StaTask = (U32)0;

  while(1)
  {
    MsgLen = msgQReceive (MsgMain, ((I8 *)&UMsgRx.Typ),OS_MSG_LEN_MAX8B,OS_TIM_100Ms);
    if(MsgLen == OS_MSG_LEN_MAX8B)
    {	
			//Task Status Judge
			FlgTemp	= (U32)0;
			switch(UMsgRx.Typ&TASK_NAME_MASK)
			{
				case TASK_NAME_ETH:
					FlgTemp = FLG_ETH;	
					break;	
				case TASK_NAME_GEN:	
					FlgTemp = FLG_PWN;					
					break;			
				case TASK_NAME_PWN:
					FlgTemp = FLG_GENs;	
					break;			

				case TASK_NAME_SWH:
					FlgTemp = FLG_SWh;	
					break;
				case TASK_NAME_RAS:
					FlgTemp = FLG_RAS;	
					break;
				case TASK_NAME_SWV:
					FlgTemp = FLG_SWv;	
					break;

				case TASK_NAME_PROP:
					FlgTemp = FLG_PROP;	
					break;
				case TASK_NAME_TOP:
					FlgTemp = FLG_TOP;
					break;
				case TASK_NAME_SHED:
					FlgTemp = FLG_SHED;
					break;

				case TASK_NAME_LEVE:
					FlgTemp = FLG_LEVE;
					break;
				case TASK_NAME_VISL:
					FlgTemp = FLG_VISL;
					break;

				case TASK_NAME_MOVF:
					FlgTemp = FLG_MOVF;
					break;
				case TASK_NAME_MOVB:
					FlgTemp = FLG_MOVB;			
					break;
				default:
					break;
			}

			if(FlgTemp)
			{
				if((UMsgRx.Typ&TASK_STA_MASK)==TASK_STA_RDY)
				{
					StaTask |=  FlgTemp;
				}
				else
				{
					StaTask &= ~FlgTemp;
				}
			}


			//Main Task Mode+Command 
      switch(Mode)
			{	
				case MMOD_IDLE:
					switch(UMsgRx.Typ)
					{
						case ECMD_GEN:					
						case ECMD_PWN:					
							Mode = MMOD_PWN;
							break;
						default:
							break;
					}
				case MMOD_PWN:
					switch(UMsgRx.Typ&ECMDx_ACTx_MASK)
					{				
						case ECMD_GEN:			
							if((UMsgRx.Typ&ECMDx_DIRx_MASK)==0)
							{//Start General
								UMsgTx.Typ = (U32)0;
								UMsgTx.Dat = (U32)0;

								if((UMsgRx.Dat&GDAT_DIESEL) == GDAT_DIESEL)
								{
									if((StaTask&FLG_GENd) == 0)
									{//Stop => Start
										UMsgTx.Typ = PCMD_GEN_ON;
										UMsgTx.Dat |= GDAT_DIESEL;
									}
								}
								if((UMsgRx.Dat&GDAT_SHAFT) == GDAT_SHAFT)
								{//Stop => Start
									if((StaTask&FLG_GENs) == 0)
									{
										UMsgTx.Typ = PCMD_GEN_ON;
										UMsgTx.Dat |= GDAT_SHAFT;
									}
								}
								if(UMsgTx.Typ)
								{							
									msgQSend (MsgPower, 	(I8 *)&UMsgTx.Typ,OS_MSG_LEN_MAX8B, NO_WAIT, MSG_PRI_NORMAL);
								}
							}
							else
							{//Stop General			
								if((StaTask&FLG_BUSY)==0)
								{								
									UMsgTx.Typ = PCMD_GEN_OFF;
									UMsgTx.Dat = UMsgRx.Dat;
									msgQSend (MsgPower, 	(I8 *)&UMsgTx.Typ,OS_MSG_LEN_MAX8B, NO_WAIT, MSG_PRI_NORMAL);
								}
								else
								{//All Servo Stop


								}
							}
							break;


						case ECMD_PWN:
							if((UMsgRx.Typ&ECMDx_DIRx_MASK)==0)
							{//Power On
								if((StaTask&FLGx_PWN)==FLGx_PWN)
								{//24V and 500V is Ready
									UMsgTx.Typ = PCMD_PWN_ON;
									UMsgTx.Dat = UMsgRx.Dat;
									msgQSend (MsgPower, 	(I8 *)&UMsgTx.Typ,OS_MSG_LEN_MAX8B, NO_WAIT, MSG_PRI_NORMAL);
								}
							}
							else
							{//Power Off					
								if((StaTask&FLG_BUSY)==0)
								{	
									UMsgTx.Typ = PCMD_PWN_OFF;
									UMsgTx.Dat = UMsgRx.Dat;
									msgQSend (MsgPower,(I8 *)&UMsgTx.Typ,OS_MSG_LEN_MAX8B, NO_WAIT, MSG_PRI_NORMAL);
								}
							}
							break;

						case ECMD_SWH:
							if((StaTask&FLGx_SWH) == FLGx_SWH)
							{
								Mode = MMOD_SWH;
							}
							break;						

						case ECMD_RAS:
							if((StaTask&FLGx_RAS) == FLGx_RAS)
							{
								Mode = MMOD_RAS;
							}
							break;

						case ECMD_SWH_RAS:
							if((StaTask&FLGx_SWH_RAS) == FLGx_SWH)
							{
								Mode = MMOD_SWH_RAS;
							}
							break;

						case ECMD_SWV:
							if((StaTask&FLGx_SWV) == FLGx_SWV)
							{
								Mode = MMOD_SWV;
							}
							break;
						case ECMD_PROP:
							if((StaTask&FLGx_PROP) == FLGx_PROP)
							{
								Mode = MMOD_PROP;
							}
							break;
						case ECMD_TOP:
							if((StaTask&FLGx_TOP) == FLGx_TOP)
							{
								Mode = MMOD_TOP;
							}
							break;
						case ECMD_SHED:
							if((StaTask&FLGx_SHED) == FLGx_SHED)
							{
								Mode = MMOD_SHED;
							}
							break;
						case ECMD_LEVE:
							if((StaTask&FLGx_LEVE) == FLGx_LEVE)
							{
								Mode = MMOD_LEVE;
							}
							break;
						case ECMD_MOVFB:
							if((StaTask&FLGx_MOVxyz) == FLGx_MOVxyz)
							{
								Mode = MMOD_SWH;
							}
							break;					
						default:
							break;
					}			

      	case MMOD_SWH:
					switch(UMsgRx.Typ&ECMDx_ACTx_MASK)
					{
						case ECMD_GEN:	
						case ECMD_PWN:
							if((StaTask&FLG_BUSY) == 0)
							{//Exit SWH => Enter PWN
								Mode = MMOD_PWN;
							}
							else
							{//Tx Stop Command
								UMsgTx.Typ = SCMD_STOP;
								UMsgTx.Dat = (U32)0;
								msgQSend (MsgSwingH, 	(I8 *)&UMsgTx.Typ,OS_MSG_LEN_MAX8B, NO_WAIT, MSG_PRI_NORMAL);
							}
							break;
						case ECMD_SWH:
							if((UMsgRx.Typ&ECMDx_MODx_MASK)==0)
							{//SWh Manual							
								//UMsgTx.Typ  = SCMD_MRUN;
								UMsgTx.Typ |= (UMsgRx.Typ&ECMDx_DIRx_MASK);
								UMsgTx.Dat  = (U32)0;
								msgQSend (MsgSwingH, 	(I8 *)&UMsgTx.Typ,OS_MSG_LEN_MAX8B, NO_WAIT, MSG_PRI_NORMAL);
							}
							else
							{//SWh Auto 
							//	UMsgTx.Typ = SCMD_ARUN;
								UMsgTx.Typ |= (UMsgRx.Typ&ECMDx_DIRx_MASK);
								UMsgTx.Dat = (U32)0;
								msgQSend (MsgSwingH, 	(I8 *)&UMsgTx.Typ,OS_MSG_LEN_MAX8B, NO_WAIT, MSG_PRI_NORMAL);
							}
							break;
						case ECMD_RAS:		
							if((StaTask&FLG_BUSY)==0)
							{//Exit SWH Mod => Enter RAS Mod
								if((StaTask&FLGx_RAS) == FLGx_RAS)
								{//Exit MMOD_SWH ,Enter  MMOD_RAS
									Mode = MMOD_RAS;
								}
							}
							else
							{//Tx Stop Command


							}
							break;
					}
					break;
				case MMOD_RAS:
					switch(UMsgRx.Typ&ECMDx_ACTx_MASK)
					{				
						case ECMD_GEN:	
						case ECMD_PWN:
							if((StaTask&FLG_BUSY) == 0)
							{
								Mode = MMOD_SWH;
							}
							else
							{


							}
							break;
						case ECMD_SWV:
							if((StaTask&FLG_BUSY) == 0)
							{
								if((StaTask&FLGx_RAS) == FLGx_RAS)
								{
									Mode = MMOD_SWH;
								}
							}
							else
							{


							}
						case ECMD_RAS:
							if((UMsgRx.Typ&ECMDx_DIRx_MASK)==0)						
							{//SWVp
								
							}
							else
							{//SWVn



							}
							break;
					}
				case MMOD_SWH_RAS:
					switch(UMsgRx.Typ&ECMDx_ACTx_MASK)
					{				
						case ECMD_GEN:	
						case ECMD_PWN:
							if((StaTask&FLG_BUSY) == 0)
							{
								Mode = MMOD_SWH;
							}
							else
							{


							}
							break;
						case ECMD_SWV:
							if((StaTask&FLG_BUSY) == 0)
							{
								if((StaTask&FLGx_RAS) == FLGx_RAS)
								{
									Mode = MMOD_SWH;
								}
							}
							else
							{


							}
						case ECMD_RAS:
							if((UMsgRx.Typ&ECMDx_DIRx_MASK)==0)						
							{//SWVp
								
							}
							else
							{//SWVn



							}
							break;
					}

					

					break;
				default:						
					break;	
      	
			}
    }
    else
    {//TimeOver Do
			TimeIdle++;			
    }
  }
}
