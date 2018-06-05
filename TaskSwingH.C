#include "vxWorks.h"
#include "rngLib.h"
#include "semLib.h"
#include "msgQLib.h"
#include "taskLib.h"
#include "tickLib.h"

#include "Default.h"
#include "Can.h"
#include "Servo.h"

extern MSG_Q_ID MsgSwingH;
extern MSG_Q_ID MsgCan0Tx;
extern MSG_Q_ID MsgCan1Tx;

extern void LLTabSevoInit(sCRxSERVO *pSevDat,U32 BufLen);
extern void CxTxBufInit(sCxBUF *pCTxBuf,U32 BufLen,U32 Id32);
extern void CxTxSevCmdSpd(sCTxSEV_SPD *pCTxSev,U32 Id32,I16 Speed,I16 Acc,U8 Dly,U8 En);


void LoadSWhLMT(void);
I32 SpdPlanSWH(I32 SpdCur,I32 AccDt,I32 LenRun,I32 LenRem);
struct sCxBUF  C0TxBufSW[CTxBUF_LEN];
struct sCxBUF  C1TxBufSW[CTxBUF_LEN];

struct sCxBUF *pC0TxNodeInSW;
struct sCxBUF *pC1TxNodeInSW;

struct sCRxSERVO CRxSevBufSWLF[LLT_SEV_LEN];
struct sCRxSERVO CRxSevBufSWLB[LLT_SEV_LEN];
struct sCRxSERVO CRxSevBufSWRF[LLT_SEV_LEN];
struct sCRxSERVO CRxSevBufSWRB[LLT_SEV_LEN];

struct sCRxSERVO *pCRxSevNodeInSWLF;
struct sCRxSERVO *pCRxSevNodeInSWLB;
struct sCRxSERVO *pCRxSevNodeInSWRF;
struct sCRxSERVO *pCRxSevNodeInSWRB;

struct sPOS_LMT	PosLmtSWh[4];


const U32 CxID32SWHSpd[4]={C0TxID32_SWH_Q_LF,C0TxID32_SWH_Q_LB,C1TxID32_SWH_Q_RF,C1TxID32_SWH_Q_RB};
const U32 CxID32SWHAmp[4]={C0TxID32_SWH_Q_LF,C0TxID32_SWH_Q_LB,C1TxID32_SWH_Q_RF,C1TxID32_SWH_Q_RB};

void TaskSwingH(void)
{ 

  U32 Mode,RunDir,Loop,MsgLen;
  U32 TickNext,TickDly,TickCur; 
	U32 TimWaitStop;
	U32 TempSn;
	I32 TempI32;
	U32 SevSta[5];
	U32 SpdZeroFilt[5];
	I32 SpdNextPlan,SpdCurPlan,LenRunPlan,LenRemPlan;
	U32 ModePlan;

	struct sSTA_FILT StaFiltCon[4];
	struct sSTA_FILT StaFiltPos[4];
	struct sSTA_FILT StaFiltSpd[4];
	struct sSTA_FILT StaFiltAmp[4];
	struct sSTA_FILT StaFiltCod[4];
	struct sSTA_FILT StaFiltSyn[4];


  struct sCxBUF *pCxRxTemp;
  struct sCxBUF *pCxTxTemp;


  struct sCRxSERVO *pCRxSevTemp;
  struct sCRxSERVO *pCRxSevNodeInTemp;

  struct sSERVO_DAT SevDatSpd[4];
  struct sSERVO_DAT SevDatPos[4];
  struct sSERVO_DAT SevDatAmp[4];
  struct sSERVO_ERR SevErr[4];
  struct sCTxCMD 		CTxSevCmd[4];

  struct sCTxSEV_SPD *pSevCmdSpd;
  struct sCTxSEV_AMP *pSevCmdAmp;

  struct sMSG_USER    UMsgRx;
  struct sMSG_USER    UMsgTx;

	LoadSWhLMT();

	LLTabSevoInit(&CRxSevBufSWLF[0],LLT_SEV_LEN);
	LLTabSevoInit(&CRxSevBufSWLB[0],LLT_SEV_LEN);
	LLTabSevoInit(&CRxSevBufSWRF[0],LLT_SEV_LEN);
	LLTabSevoInit(&CRxSevBufSWRB[0],LLT_SEV_LEN);

	pCRxSevNodeInSWLF = &CRxSevBufSWLF[0];
	pCRxSevNodeInSWLB = &CRxSevBufSWLB[0];
	pCRxSevNodeInSWRF = &CRxSevBufSWRF[0];
	pCRxSevNodeInSWRB = &CRxSevBufSWRB[0];

  //Init Data Struct and variables;
  for(Loop=0;Loop<4;Loop++)
  {
    SevDatSpd[Loop].SumI32 = (I32)0;
    SevDatSpd[Loop].CurI32 = (I32)0;
    SevDatSpd[Loop].AvgI32 = (I32)0;

    SevDatPos[Loop].SumI32 = (I32)0;
    SevDatPos[Loop].CurI32 = (I32)0;
    SevDatPos[Loop].AvgI32 = (I32)0;

    SevDatAmp[Loop].SumI32 = (I32)0;
    SevDatAmp[Loop].CurI32 = (I32)0;
    SevDatAmp[Loop].AvgI32 = (I32)0;
	
		StaFiltCon[Loop].Err 	= SEV_SWhLR_CON_FILT_MAX;
		StaFiltCon[Loop].Noml = SEV_SWhLR_CON_FILT_MAX;
		StaFiltPos[Loop].Err 	= SEV_SWhLR_POS_FILT_MAX;
		StaFiltPos[Loop].Noml = SEV_SWhLR_POS_FILT_MAX;
		StaFiltSpd[Loop].Err 	= SEV_SWhLR_SPD_FILT_MAX;
		StaFiltSpd[Loop].Noml = SEV_SWhLR_SPD_FILT_MAX;
		StaFiltAmp[Loop].Err 	= SEV_SWhLR_AMP_FILT_MAX;		
		StaFiltAmp[Loop].Noml = SEV_SWhLR_AMP_FILT_MAX;
		StaFiltCod[Loop].Err 	= SEV_SWhLR_COD_FILT_MAX;
		StaFiltCod[Loop].Noml = SEV_SWhLR_COD_FILT_MAX;

		StaFiltSyn[Loop].Err 	= SEV_SWhLR_SYNC_FILT_MAX;
		StaFiltSyn[Loop].Noml = SEV_SWhLR_SYNC_FILT_MAX;	
		SpdZeroFilt[Loop] 		= SEV_SWLR_SPD_ZERO_FILT_MAX;

		SevSta[Loop] 			=	(U32)0;
	
		pSevCmdSpd = (sCTxSEV_SPD*)&CTxSevCmd[Loop].ID32;
 		pSevCmdSpd->ID32		= CxID32SWHSpd[Loop];
		pSevCmdSpd->RecU16	= SEV_SPD_RECU16;
		pSevCmdSpd->Spd			= 0;
		pSevCmdSpd->Acc			= SEV_SPD_ACC;
		pSevCmdSpd->Dly			= SEV_SPD_DLY0;	
		pSevCmdSpd->En 			= SEV_BRAKE_CLOSE;
		
  }

	SevSta[4]   =	(U32)0;
	SpdNextPlan = (U32)0;
	SpdCurPlan	= (U32)0;
	LenRunPlan	= (U32)0;
	LenRemPlan	= (U32)0;
	ModePlan  	= SEV_PLAN_MOD_IDLE;
		
	//TimWaitStop		= TIM_WAIT_STOP_5S;
	Mode = SMOD_STOP;
	
  taskDelay(OS_TIM_100Ms);	
  TickDly       = CAN_PERO0_40MS;//1s
  TickNext      = tickGet()+TickDly;
  while(1)
  {
    TickCur = TickNext - tickGet();
    if(TickCur > TickDly)
    {
      TickCur = 0;
    }
    MsgLen = msgQReceive (MsgSwingH, ((I8 *)&UMsgRx.Typ),OS_MSG_LEN_MAX8B,TickCur);
    switch(MsgLen)
    {
    	case OS_MSG_LEN_MAX8B://Rx TaskMain Command
				switch(Mode)
				{
					case SMOD_STOP:
						switch(UMsgRx.Typ)
						{
							case SCMD_STOP:
								for(Loop=0;Loop<4;Loop++)
								{
									ModePlan  	= SEV_PLAN_MOD_IDLE;
									pSevCmdSpd = (sCTxSEV_SPD*)&CTxSevCmd[Loop].ID32;
 									pSevCmdSpd->ID32		= CxID32SWHSpd[Loop];
									pSevCmdSpd->RecU16	= SEV_SPD_RECU16;
									pSevCmdSpd->Spd			= 0;
									pSevCmdSpd->Acc			= SEV_SPD_ACC;
									pSevCmdSpd->Dly			= SEV_SPD_DLY0;								
	
									if(SevSta[Loop]&FLG_SEV_STOP)
									{									
										pSevCmdSpd->En 	= SEV_BRAKE_CLOSE;
									}								
								}
								break;						
							case SCMD_ARUN:
								if(((SevSta[SnLF]&FLG_SEV_ERR_MASK) || (SevSta[SnLB]&FLG_SEV_ERR_MASK)||
										(SevSta[SnRF]&FLG_SEV_ERR_MASK) || (SevSta[SnRB]&FLG_SEV_ERR_MASK)||
										(SevSta[SnSYN]&FLG_SEV_ERR_SYNC_MASK))	== 0	)
								{												
									RunDir 	= UMsgRx.Typ &ECMDx_DIRx_MASK;
									if(RunDir == ECMDx_DIRp)	
									{
										Mode 	= SMOD_ARUNp;
										ModePlan = SEV_PLAN_MOD_ARUNp;										
									}
									else
									{
										Mode 	= SMOD_ARUNn;
										ModePlan = SEV_PLAN_MOD_ARUNn;
									}										
									SpdCurPlan 	= 0;		
									SpdNextPlan = 0;														
								}
								break;
							case SCMD_MRUN:
								RunDir 	= UMsgRx.Typ &ECMDx_DIRx_MASK;
								if(RunDir == ECMDx_DIRp)	
								{
									Mode 	= SMOD_MRUNp;
									ModePlan = SEV_PLAN_MOD_MRUNp;										
								}
								else
								{
									Mode 	= SMOD_MRUNn;
									ModePlan = SEV_PLAN_MOD_MRUNn;
								}				
								SpdCurPlan 	= 0;		
								SpdNextPlan = 0;	
							
								break;
							default:
								break;
						}
						break;

					case SMOD_ARUNp:	
						switch(UMsgRx.Typ)
						{
						
							case SCMD_ARUN:
								if(((SevSta[SnLF]&FLG_SEV_ERR_MASK) || (SevSta[SnLB]&FLG_SEV_ERR_MASK)||
										(SevSta[SnRF]&FLG_SEV_ERR_MASK) || (SevSta[SnRB]&FLG_SEV_ERR_MASK)||
										(SevSta[SnSYN]&FLG_SEV_ERR_SYNC_MASK)) == 0)
								{	 								
									//if(1)
									{//
										//Pos
									}

								}
								else
								{
									Mode 			= SMOD_STOP;
									ModePlan 	= SEV_PLAN_MOD_IDLE;			
								}
								break;

							case SCMD_STOP:
							case SCMD_MRUN:	
							default:
								Mode 			= SMOD_STOP;
								ModePlan 	= SEV_PLAN_MOD_IDLE;		
								break;
						}
					case SMOD_ARUNn:	
						switch(UMsgRx.Typ)						
						{
							case SCMD_STOP:
								Mode 			= SMOD_STOP;
								ModePlan  = SEV_PLAN_MOD_IDLE;				
								break;
							case SCMD_ARUN:
								if(((SevSta[SnLF]&FLG_SEV_ERR_MASK) || (SevSta[SnLB]&FLG_SEV_ERR_MASK)||
										(SevSta[SnRF]&FLG_SEV_ERR_MASK) || (SevSta[SnRB]&FLG_SEV_ERR_MASK)||
										(SevSta[SnSYN]&FLG_SEV_ERR_SYNC_MASK)) == 0)
								{	 								
									//amp
								}
								else
								{
									Mode 			= SMOD_STOP;
									ModePlan 	= SEV_PLAN_MOD_IDLE;			
								}																
								break;
							default:
								Mode 			= SMOD_STOP;
								ModePlan 	= SEV_PLAN_MOD_IDLE;			
								break;
						}
						break;		

					case SMOD_MRUNp:
						switch(UMsgRx.Typ)
						{
							case SCMD_STOP:								
								break;						

							case SCMD_MRUN:	
							
				
								break;

							default:
								break;
						}	
						break;
					case SMOD_MRUNn:	
						break;
					default:
						break;
				}  	
				break;

    	case OS_MSG_LEN_MAX4B://Rx CanBus Data
      	pCxRxTemp = (sCxBUF*)&UMsgRx.Typ;
				if(pCxRxTemp->Sta == CxBUF_RDY)
				{      	
      		pCRxSevTemp		= (sCRxSERVO*)&pCxRxTemp->ID32;
					if(pCRxSevTemp->PForm == PF_STA_SEVO_DAT)
					{
						TempSn = SnNULL;
      			switch(pCRxSevTemp->SAddr)
      			{
      				case ExSAD_SW_HLF:   
								TempSn 						= SnLF;   						
								pCRxSevNodeInTemp = pCRxSevNodeInSWLF;          
        				break;
      				case ExSAD_SW_HLB:
								TempSn 						= SnLB;   						
								pCRxSevNodeInTemp = pCRxSevNodeInSWLF;         				
								break;

      				case ExSAD_SW_HRF:
								TempSn 						= SnRF;   						
								pCRxSevNodeInTemp = pCRxSevNodeInSWLF;    
        				break;
      				case ExSAD_SW_HRB:
								TempSn 						= SnRB;   						
								pCRxSevNodeInTemp = pCRxSevNodeInSWLF; 
        				break;
      				default:
        				break;
      			}
						if(TempSn != SnNULL)
						{//Calc Position,Speed,Current Average											
							SevDatPos[TempSn].SumI32 -= (I32)pCRxSevNodeInTemp->Pos;
          		SevDatSpd[TempSn].SumI32 -= (I32)pCRxSevNodeInTemp->Spd;
          		SevDatAmp[TempSn].SumI32 -= (I32)pCRxSevNodeInTemp->Amp;
									
							pCxRxTemp->Sta 	= CxBUF_RDING;	

            	pCRxSevNodeInTemp->SAddr     	= pCRxSevTemp->SAddr;
            	pCRxSevNodeInTemp->DAddr     	= pCRxSevTemp->DAddr;
            	pCRxSevNodeInTemp->PForm     	= pCRxSevTemp->PForm;
            	pCRxSevNodeInTemp->Page     	= pCRxSevTemp->Page;
            	pCRxSevNodeInTemp->Pos       	= pCRxSevTemp->Pos;
            	pCRxSevNodeInTemp->Spd       	= pCRxSevTemp->Spd;
            	pCRxSevNodeInTemp->Amp     		= pCRxSevTemp->Amp;
            	pCRxSevNodeInTemp->ErrCode   	= pCRxSevTemp->ErrCode;
            	pCRxSevNodeInTemp->Sta       	= pCRxSevTemp->Sta;

            	SevDatPos[TempSn].SumI32 += (I32)pCRxSevNodeInTemp->Pos;
            	SevDatSpd[TempSn].SumI32 += (I32)pCRxSevNodeInTemp->Spd;
            	SevDatAmp[TempSn].SumI32 += (I32)pCRxSevNodeInTemp->Amp;

           		SevDatPos[TempSn].AvgI32 	= SevDatPos[TempSn].SumI32/((I32)LLT_SEV_LEN);
            	SevDatSpd[TempSn].AvgI32 	= SevDatSpd[TempSn].SumI32/((I32)LLT_SEV_LEN);
            	SevDatAmp[TempSn].AvgI32 	= SevDatAmp[TempSn].SumI32/((I32)LLT_SEV_LEN);

           		pCRxSevNodeInTemp = pCRxSevNodeInTemp->Next;
      				pCxRxTemp->Sta = CxBUF_NULL;

							//Position Filter,Err Judge
							if((SevDatPos[TempSn].AvgI32 < PosLmtSWh[TempSn].Max)&&(SevDatPos[TempSn].AvgI32 > PosLmtSWh[TempSn].Min))
          		{            			
								StaFiltPos[TempSn].Err 		= SEV_SWhLR_POS_FILT_MAX;
								if(StaFiltPos[TempSn].Noml)
								{
									StaFiltPos[TempSn].Noml--;
								}
								else
								{
									SevSta[TempSn] 				&= ~FLG_SEV_ERR_POS;
									StaFiltPos[TempSn].Noml  = SEV_SWhLR_POS_FILT_MAX<<4;
								}										
          		}
          		else
          		{            	
								StaFiltPos[TempSn].Noml 	= SEV_SWhLR_POS_FILT_MAX;
								if(StaFiltPos[TempSn].Err)
								{
									StaFiltPos[TempSn].Err--;
								}
								else
								{
									SevSta[TempSn] 			|= FLG_SEV_ERR_POS;
									StaFiltPos[TempSn].Err = SEV_SWhLR_POS_FILT_MAX<<4;
								}
          		}
							//Speed Filter,Err Judge
          		if((SevDatSpd[TempSn].AvgI32 < SEV_SWhLR_SPD_MAXp) && (SevDatSpd[TempSn].AvgI32 > SEV_SWhLR_SPD_MAXn))
          		{
								StaFiltSpd[TempSn].Err = SEV_SWhLR_SPD_FILT_MAX;
								if(StaFiltSpd[TempSn].Noml)
								{
									StaFiltSpd[TempSn].Noml--;
								}
								else
								{
									SevSta[TempSn] 					&= ~FLG_SEV_ERR_SPD;
									StaFiltSpd[TempSn].Noml  = SEV_SWhLR_SPD_FILT_MAX<<4;
								}
								if(SevDatSpd[TempSn].AvgI32 > SEV_SWLR_SPD_ZERO)
								{
									SpdZeroFilt[TempSn] = SEV_SWLR_SPD_ZERO_FILT_MAX;
									SevSta[TempSn] 		&= ~FLG_SEV_STOP;
								}
								else
								{
									if(SpdZeroFilt[TempSn])
									{
										SpdZeroFilt[TempSn]--;
									}
									else
									{
										SevSta[TempSn] |= FLG_SEV_STOP;
									}
								}							
							}
            	else
           		{
								StaFiltSpd[TempSn].Noml = SEV_SWhLR_SPD_FILT_MAX;
								if(StaFiltSpd[TempSn].Err)
								{
									StaFiltSpd[TempSn].Err--;
								}
								else
								{
									SevSta[TempSn] 				|= FLG_SEV_ERR_SPD;
									StaFiltSpd[TempSn].Err 	 = SEV_SWhLR_SPD_FILT_MAX<<4;
								}
            	}

							//Current Filter,Err Judge
            	if((SevDatAmp[TempSn].AvgI32 < SEV_SWhLR_AMP_MAXp)&&(SevDatAmp[TempSn].AvgI32 >SEV_SWhLR_AMP_MAXn))
            	{             	
								StaFiltAmp[TempSn].Err  = SEV_SWhLR_AMP_FILT_MAX;
								if(StaFiltAmp[TempSn].Noml)
								{
									StaFiltAmp[TempSn].Noml--;
								}
								else
								{
									SevSta[TempSn] 				&= ~FLG_SEV_ERR_AMP;
									StaFiltAmp[TempSn].Noml  = SEV_SWhLR_AMP_FILT_MAX<<4;
								}		
           		}
           		else
           		{	
								StaFiltAmp[TempSn].Noml  = SEV_SWhLR_AMP_FILT_MAX;
								if(StaFiltAmp[TempSn].Err)
								{
									StaFiltAmp[TempSn].Err--;
								}
								else
								{
									SevSta[TempSn] 			|= FLG_SEV_ERR_AMP;
									StaFiltAmp[TempSn].Err = SEV_SWhLR_AMP_FILT_MAX<<4;
								}
           		}

							//ErrCode  Filter,Err Judge
							if(pCRxSevNodeInTemp->ErrCode == ((U8)0))
							{
								StaFiltCod[TempSn].Err 		= SEV_SWhLR_COD_FILT_MAX;
								if(StaFiltCod[TempSn].Noml)
								{
									StaFiltCod[TempSn].Noml--;
								}
								else
								{
									SevSta[TempSn] 				&= ~FLG_SEV_ERR_COD;
									StaFiltCod[TempSn].Noml  = SEV_SWhLR_COD_FILT_MAX<<4;
								}	
							}
							else
							{
								StaFiltCod[TempSn].Noml 	= SEV_SWhLR_COD_FILT_MAX;
								if(StaFiltCod[TempSn].Err)
								{
									StaFiltCod[TempSn].Err--;
								}
								else
								{
									SevSta[TempSn] 			|= FLG_SEV_ERR_COD;
									StaFiltCod[TempSn].Err = SEV_SWhLR_COD_FILT_MAX<<4;
								}
							}
						
							//Position Lock Condition  Filter,Judge						
							if(SevDatPos[TempSn].AvgI32 < PosLmtSWh[TempSn].Con)
          		{            			
								StaFiltCon[TempSn].Err 		= SEV_SWhLR_CON_FILT_MAX;
								if(StaFiltCon[TempSn].Noml)
								{
									StaFiltCon[TempSn].Noml--;
								}
								else
								{
									SevSta[TempSn] 				&= ~FLG_SEV_LenLMT;
									StaFiltCon[TempSn].Noml  = SEV_SWhLR_CON_FILT_MAX<<4;
								}															
          		}
          		else
          		{            	
								StaFiltCon[TempSn].Noml 	= SEV_SWhLR_CON_FILT_MAX;
								if(StaFiltCon[TempSn].Err)
								{
									StaFiltCon[TempSn].Err--;
								}
								else
								{
									SevSta[TempSn] 			|= FLG_SEV_LenLMT;
									StaFiltCon[TempSn].Err = SEV_SWhLR_CON_FILT_MAX<<4;
								}
          		}

							//Position Sync Filter	
							//Single Left Side ,Front and Behind Compare
							TempI32 = SevDatPos[SnLF].AvgI32 - SevDatPos[SnLB].AvgI32;
							if(TempI32 < (I32)0)
							{
								TempI32 = -TempI32;
							}					
							if(TempI32 < SEV_SWh_SYNC_FB_ERR_MAX)
							{
								StaFiltSyn[SnLFB].Err 		= SEV_SWhLR_SYNC_FILT_MAX;
								if(StaFiltSyn[SnLFB].Noml)
								{
									StaFiltSyn[SnLFB].Noml--;
								}
								else
								{
									SevSta[SnSYN] 				&= ~FLG_SEV_ERR_SYNC_L;
									StaFiltSyn[SnLFB].Noml = SEV_SWhLR_SYNC_FILT_MAX<<4;
								}
							}						
							else
							{
								StaFiltSyn[SnLFB].Noml 	= SEV_SWhLR_SYNC_FILT_MAX;
								if(StaFiltSyn[SnLFB].Err)
								{
									StaFiltSyn[SnLFB].Err--;
								}
								else
								{
									SevSta[SnSYN] 			|= FLG_SEV_ERR_SYNC_L;
									StaFiltSyn[SnLFB].Err = SEV_SWhLR_SYNC_FILT_MAX<<4;
								}
							}

							//Single  Right Side ,Front and Behind Compare
							TempI32 = SevDatPos[SnRF].AvgI32 - SevDatPos[SnRB].AvgI32;
							if(TempI32 < (I32)0)
							{
								TempI32 = -TempI32;
							}					
							if(TempI32 < SEV_SWh_SYNC_FB_ERR_MAX)
							{
								StaFiltSyn[SnRFB].Err 		= SEV_SWhLR_SYNC_FILT_MAX;
								if(StaFiltSyn[SnRFB].Noml)
								{
									StaFiltSyn[SnRFB].Noml--;
								}
								else
								{
									SevSta[SnSYN] 				&= ~FLG_SEV_ERR_SYNC_R;
									StaFiltSyn[SnRFB].Noml = SEV_SWhLR_SYNC_FILT_MAX<<4;
								}
							}						
							else
							{
								StaFiltSyn[SnRFB].Noml 	= SEV_SWhLR_SYNC_FILT_MAX;
								if(StaFiltSyn[SnRFB].Err)
								{
									StaFiltSyn[SnRFB].Err--;
								}
								else
								{
									SevSta[SnSYN] 			|= FLG_SEV_ERR_SYNC_R;
									StaFiltSyn[SnRFB].Err = SEV_SWhLR_SYNC_FILT_MAX<<4;
								}
							}

							//Double  Side ,Left and Right Compare

							TempI32 = ((SevDatPos[SnLF].AvgI32 + SevDatPos[SnLB].AvgI32)/2) - ((SevDatPos[SnRF].AvgI32 + SevDatPos[SnRB].AvgI32)/2);
							if(TempI32 < (I32)0)
							{
								TempI32 = -TempI32;
							}					
							if(TempI32 < SEV_SWh_SYNC_LR_ERR_MAX)
							{
								StaFiltSyn[SnLRx].Err 		= SEV_SWhLR_SYNC_FILT_MAX;
								if(StaFiltSyn[SnLRx].Noml)
								{
									StaFiltSyn[SnLRx].Noml--;
								}
								else
								{
									SevSta[SnSYN] 		   &= ~FLG_SEV_ERR_SYNC_LR;
									StaFiltSyn[SnLRx].Noml = SEV_SWhLR_SYNC_FILT_MAX<<4;
								}
							}
							else
							{
								StaFiltSyn[SnLRx].Noml 	= SEV_SWhLR_SYNC_FILT_MAX;
								if(StaFiltSyn[SnLRx].Err)
								{
									StaFiltSyn[SnLRx].Err--;
								}
								else
								{
									SevSta[SnSYN] 			|= FLG_SEV_ERR_SYNC_LR;
									StaFiltSyn[SnLRx].Err = SEV_SWhLR_SYNC_FILT_MAX<<4;
								}
							}
							//Calc Plan Run Spd Need for Paramer;
							if(ModePlan != SEV_PLAN_MOD_IDLE)
							{
								if(RunDir == ECMDx_DIRp)
								{
									TempI32 = SevDatPos[0].AvgI32;
									for(Loop=1;Loop<4;Loop++)
									{
										if(SevDatPos[Loop].AvgI32 < TempI32)
										{//Select Slow ,Len Min
											TempI32 = SevDatPos[Loop].AvgI32;		
										}
									}										
								
									if(TempI32>SEV_PLAN_SWH_POS_MAX)
									{
										TempI32 = SEV_PLAN_SWH_POS_MAX;		
									}
									else if(TempI32 < SEV_PLAN_SWH_POS_MIN)
									{
										TempI32 = SEV_PLAN_SWH_POS_MIN;
									}
										
									LenRunPlan = TempI32;
									LenRemPlan = SEV_PLAN_SWH_POS_MAX - TempI32;
								}
								else
								{
									TempI32 = SevDatPos[0].AvgI32;
									for(Loop=1;Loop<4;Loop++)
									{
										if(SevDatPos[Loop].AvgI32 > TempI32)
										{//Select Slow, Len Max
											TempI32 = SevDatPos[Loop].AvgI32;		
										}
									}										
									
									if(TempI32>SEV_PLAN_SWH_POS_MAX)
									{
										TempI32 = SEV_PLAN_SWH_POS_MAX;		
									}
									else if(TempI32 < SEV_PLAN_SWH_POS_MIN)
									{
										TempI32 = SEV_PLAN_SWH_POS_MIN;
									}									
									LenRunPlan = TempI32;
									LenRemPlan = TempI32 - SEV_PLAN_SWH_POS_MIN;
								}
							}

						}
					}
				}
      	break;

    default://TimeOver: Txd CanData
			
      TickNext = tickGet()+TickDly;//Calc Next Delay Period
		
			//Plan Route ,Calc PID parameter //Add PID DSZ
			switch(ModePlan)
			{
				case SEV_PLAN_MOD_ARUNp:								
					SpdNextPlan = SpdPlanSWH(SpdCurPlan,SEV_PLAN_SWHM_ACCp_Dt,LenRunPlan,LenRemPlan);								
					for(Loop=0;Loop<4;Loop++)
					{
						pSevCmdSpd = (sCTxSEV_SPD*)&CTxSevCmd[Loop].ID32;
 						pSevCmdSpd->ID32		= CxID32SWHSpd[Loop];
						pSevCmdSpd->RecU16	= SEV_SPD_RECU16;
						pSevCmdSpd->Spd			= SpdNextPlan;
						pSevCmdSpd->Acc			= SEV_SPD_ACC;
						pSevCmdSpd->Dly			= SEV_SPD_DLY0;
						pSevCmdSpd->En			= SEV_BRAKE_OPEN;		
					}
					break;
				
				case SEV_PLAN_MOD_ARUNn:
					if(LenRemPlan > SEV_PLAN_SWH_LEN_MOMENT_3mm)
					{
						SpdNextPlan = SpdPlanSWH(SpdCurPlan,SEV_PLAN_SWHM_ACCn_Dt,LenRunPlan,LenRemPlan);
						for(Loop=0;Loop<4;Loop++)
						{
							pSevCmdSpd = (sCTxSEV_SPD*)&CTxSevCmd[Loop].ID32;
 							pSevCmdSpd->ID32		= CxID32SWHSpd[Loop];
							pSevCmdSpd->RecU16	= SEV_SPD_RECU16;
							pSevCmdSpd->Spd			= -SpdNextPlan;
							pSevCmdSpd->Acc			= SEV_SPD_ACC;
							pSevCmdSpd->Dly			= SEV_SPD_DLY0;
							pSevCmdSpd->En			= SEV_BRAKE_OPEN;								
						}					
					}
					else
					{
						for(Loop=0;Loop<4;Loop++)
						{
							pSevCmdAmp = (sCTxSEV_AMP*)&CTxSevCmd[Loop].ID32;
 							pSevCmdAmp->ID32		= CxID32SWHAmp[Loop];
							pSevCmdAmp->RecU32	= SEV_SPD_RECU32;
							pSevCmdAmp->Amp			= -SEV_PLAN_SWH_MOMENT_AMP;						
							pSevCmdAmp->Dly			= SEV_SPD_DLY0;
							pSevCmdAmp->En			= SEV_BRAKE_OPEN;		
						}						
					}
					break;			
				case SEV_PLAN_MOD_MRUNp:


					break;

				case SEV_PLAN_MOD_MRUNn:


					break;			
				default:
					for(Loop=0;Loop<4;Loop++)
					{
						pSevCmdSpd = (sCTxSEV_SPD*)&CTxSevCmd[Loop].ID32;
 						pSevCmdSpd->ID32		= CxID32SWHSpd[Loop];
						pSevCmdSpd->RecU16	= SEV_SPD_RECU16;
						pSevCmdSpd->Spd			= 0;
						pSevCmdSpd->Acc			= SEV_SPD_ACC;
						pSevCmdSpd->Dly			= SEV_SPD_DLY0;					
					}							
					break;
			}
      if(pC0TxNodeInSW->Sta == CxBUF_NULL)
      {
        pC0TxNodeInSW->Sta 		= CxBUF_WRING;
        pC0TxNodeInSW->ID32 	= CTxSevCmd[SnLF].ID32;
        pC0TxNodeInSW->Dat03 	= CTxSevCmd[SnLF].Dat03;
        pC0TxNodeInSW->Dat47 	= CTxSevCmd[SnLF].Dat47;
        pC0TxNodeInSW-> Sta 	= CxBUF_RDY;
        msgQSend (MsgCan0Tx, (I8 *)pC0TxNodeInSW,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_URGENT);
        pC0TxNodeInSW = pC0TxNodeInSW->Next;
      }

      if(pC0TxNodeInSW->Sta == CxBUF_NULL)
      {
        pC0TxNodeInSW-> Sta 	= CxBUF_WRING;
        pC0TxNodeInSW->ID32 	= CTxSevCmd[SnLB].ID32;
        pC0TxNodeInSW->Dat03 	= CTxSevCmd[SnLB].Dat03;
        pC0TxNodeInSW->Dat47 	= CTxSevCmd[SnLB].Dat47;
        pC0TxNodeInSW-> Sta 	= CxBUF_RDY;
        msgQSend (MsgCan0Tx, (I8 *)pC0TxNodeInSW,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_URGENT);
        pC0TxNodeInSW = pC0TxNodeInSW->Next;
      }

      if(pC1TxNodeInSW->Sta == CxBUF_NULL)
      {
        pC1TxNodeInSW-> Sta 	= CxBUF_WRING;
        pC1TxNodeInSW->ID32 	= CTxSevCmd[SnRF].ID32;
        pC1TxNodeInSW->Dat03 	= CTxSevCmd[SnRF].Dat03;
        pC1TxNodeInSW->Dat47 	= CTxSevCmd[SnRF].Dat47;
        pC1TxNodeInSW-> Sta 	= CxBUF_RDY;
        msgQSend (MsgCan1Tx, (I8 *)pC1TxNodeInSW,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_URGENT);
        pC1TxNodeInSW = pC1TxNodeInSW->Next;
      }

      if(pC1TxNodeInSW->Sta == CxBUF_NULL)
      {
        pC1TxNodeInSW-> Sta 	= CxBUF_WRING;
        pC1TxNodeInSW->ID32 	= CTxSevCmd[SnRB].ID32;
        pC1TxNodeInSW->Dat03 	= CTxSevCmd[SnRB].Dat03;
        pC1TxNodeInSW->Dat47 	= CTxSevCmd[SnRB].Dat47;
        pC1TxNodeInSW-> Sta 	= CxBUF_RDY;
        msgQSend (MsgCan1Tx, (I8 *)pC1TxNodeInSW,OS_MSG_LEN_MAX4B, NO_WAIT, MSG_PRI_URGENT);
        pC1TxNodeInSW = pC1TxNodeInSW->Next;
      }			

      break;
    }
  }
}


//S = (Vt*Vt -Vo*Vo)/2a = Vo*t+((a*T*T)/2)
I32 SpdPlanSWH(I32 SpdCur,I32 AccDt,I32 LenRun,I32 LenRem)
{
	I32 LenStop,SpdPlan;
	if(SpdCur > SEV_PLAN_SWH_SPD_SLOW)
	{
		LenStop = (((SpdCur * SpdCur) - SEV_PLAN_SWH_SPD_SLOW*SEV_PLAN_SWH_SPD_SLOW) / (2*SEV_PLAN_SWH_ACCp)) + SEV_PLAN_SWH_LEN_SLOW_20mm;
	}
	else
	{
		LenStop = 0;
	}

	if(LenRem > LenStop)
	{
		if(LenRun > SEV_PLAN_SWH_LEN_SLOW_20mm)
		{
			if(SpdCur < SEV_PLAN_SWH_SPD_FAST)
			{
				SpdPlan = SpdCur+ AccDt;
			}
			else
			{
				SpdPlan =  SEV_PLAN_SWH_SPD_FAST;
			}
		}
		else
		{
			SpdPlan = SEV_PLAN_SWH_SPD_SLOW;
		}
	}
	else 
	{
		if(LenStop > SEV_PLAN_SWH_LEN_SLOW_20mm)
		{
			if(SpdPlan > SEV_PLAN_SWH_SPD_SLOW)
			{
				SpdPlan = SpdCur - AccDt;
			}
			else
			{
				SpdPlan = SEV_PLAN_SWH_SPD_SLOW;
			}
		}
		else
		{
			if(LenRem>0)
			{
				SpdPlan = SEV_PLAN_SWH_SPD_SLOW;
			}
			else
			{
				SpdPlan = 0;
			}		
		}
	}
	return(SpdPlan);
}



void LoadSWhLMT(void)
{
	PosLmtSWh[SnLF].Max 	= SEV_SWhLF_POS_MAX;
	PosLmtSWh[SnLF].Con 	= SEV_SWhLF_POS_CON;
	PosLmtSWh[SnLF].Min 	= SEV_SWhLF_POS_MIN;

	PosLmtSWh[SnLB].Max 	= SEV_SWhLB_POS_MAX;
	PosLmtSWh[SnLB].Con 	= SEV_SWhLB_POS_CON;
	PosLmtSWh[SnLB].Min 	= SEV_SWhLB_POS_MIN;

	PosLmtSWh[SnRF].Max 	= SEV_SWhRF_POS_MAX;
	PosLmtSWh[SnRF].Con 	= SEV_SWhRF_POS_CON;
	PosLmtSWh[SnRF].Min 	= SEV_SWhRF_POS_MIN;

	PosLmtSWh[SnRB].Max 	= SEV_SWhRB_POS_MAX;
	PosLmtSWh[SnRB].Con 	= SEV_SWhRB_POS_CON;
	PosLmtSWh[SnRB].Min 	= SEV_SWhRB_POS_MIN;
}
