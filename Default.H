typedef  double D64;
typedef  unsigned int U32;
typedef       		int I32;
typedef  unsigned short U16;
typedef  					short I16;
typedef  unsigned char U8;
typedef          	char I8;

#define OS_TICK_SEC ((U32)200)

#define OS_TIM_5Ms   (OS_TICK_SEC/200)
#define OS_TIM_50Ms  (OS_TICK_SEC/20)
#define OS_TIM_100Ms (OS_TICK_SEC/10)
#define OS_TIM_200Ms (OS_TICK_SEC/5)
#define OS_TIM_500Ms (OS_TICK_SEC/2)

#define OS_TIM_5s        (OS_TICK_SEC*5)

#define CAN_PERO0_40MS   (OS_TICK_SEC/25)
#define CAN_PERO1_1S     (OS_TICK_SEC)


#define OS_PRIO_TEST    ((I32)5)
#define OS_PRIO_MAIN    ((I32)6)

#define OS_PRIO_CANTX   ((I32)8)
#define OS_PRIO_CANRX   ((I32)9)


#define OS_PRIO_MOVB   	((I32)11)
#define OS_PRIO_MOVF   	((I32)12)

#define OS_PRIO_PROP    ((I32)20)
#define OS_PRIO_SHED    ((I32)21)
#define OS_PRIO_SWINGH  ((I32)22)
#define OS_PRIO_SWINGV  ((I32)23)
#define OS_PRIO_POW     ((I32)45)
#define OS_PRIO_MONO    ((I32)46)
#define OS_PRIO_REC     ((I32)48)

#define OS_PRIO_ETH     ((I32)49)

#define OS_STACK_SIZE   ((I32)512)


#define OS_MSG_QUEU_MAX16       ((I32)16)
#define OS_MSG_QUEU_MAX32      	((I32)32)
#define OS_MSG_QUEU_MAX64       ((I32)64)
#define OS_MSG_QUEU_MAX128      ((I32)128)
#define OS_MSG_QUEU_MAX256      ((I32)256)
#define OS_MSG_LEN_MAX4B        ((I32)4)
#define OS_MSG_LEN_MAX8B        ((I32)8)


#define CxBUF_NULL  ((U32)0xCA000000)
#define CxBUF_WRING ((U32)0xCA07E109)
#define CxBUF_RDING ((U32)0xCA07D109)
#define CxBUF_RDY   ((U32)0xCA07EAD4)

#define CRxBUF_LEN  ((U32)256)
#define CTxBUF_LEN  ((U32)32)
#define TEMP_BUF_LEN ((U32)8) 


#define MMOD_IDLE  			((U32)0xA10D1001)
#define MMOD_PWN   			((U32)0xA10D1002)

#define MMOD_SWH    		((U32)0xA10D2001)
#define MMOD_RAS    		((U32)0xA10D2002)
#define MMOD_SWH_RAS 		((U32)0xA10D2003)
#define MMOD_SWV    		((U32)0xA10D2004)
#define MMOD_PROP 			((U32)0xA10D2005)
#define MMOD_TOP 				((U32)0xA10D2006)
#define MMOD_SHED 			((U32)0xA10D2007)
#define MMOD_LEVE 			((U32)0xA10D3001)
#define MMOD_MOVFB			((U32)0xA10D3002)


#define ECMD_GEN				((U32)0xEC001001)
#define ECMD_PWN 				((U32)0xEC001002)


#define ECMD_SWH				((U32)0xEC002001)
#define ECMD_RAS 				((U32)0xEC002002)

#define ECMD_SWH_RAS 		((U32)0xEC002003)

#define ECMD_SWV 				((U32)0xEC002004)
#define ECMD_PROP 			((U32)0xEC002005)
#define ECMD_TOP 				((U32)0xEC002011)
#define ECMD_SHED 			((U32)0xEC002012)
#define ECMD_LEVE 			((U32)0xEC002013)

#define ECMD_MOVFB			((U32)0xEC003001)
#define ECMDx_ACTx_MASK	((U32)0xFF00F00F)

#define ECMDx_MODa 			((U32)0x00000000)
#define ECMDx_MODm 			((U32)0x00E00000)
#define ECMDx_MODx_MASK ((U32)0x00F00000)

#define ECMDx_DIRp 			((U32)0x00000000)
#define ECMDx_DIRn 			((U32)0x000C0000)
#define ECMDx_DIRx_MASK ((U32)0x000F0000)



#define SCMD_ARUN 	((U32)0x5C0000A0)
#define SCMD_MRUN 	((U32)0x5C0000E0)
#define SCMD_STOP  	((U32)0x5C000050)


#define SCMD_SET  	((U32) 0x5CDE7000)
#define SCMD_SAVE  	((U32) 0x5CDAE000)
#define SCMD_IDLE  	((U32) 0x5CD1D1E0)





#define PCMD_PWN_ON   ((U32)0x9CD24055)
#define PCMD_PWN_OFF  ((U32)0x9CD240FF)
#define PCMD_V50_ON   ((U32)0x9CD50055)
#define PCMD_V50_OFF  ((U32)0x9CD500FF)
#define PCMD_GEN_ON   ((U32)0x9CD50055)
#define PCMD_GEN_OFF  ((U32)0x9CD500FF)


#define STA_SEV_SWH		((U32)0x95A00055)
#define STA_SEV_SWV		((U32)0x95A00055)


#define TASK_STA_IDLE				((U32)0x00000000)
#define TASK_STA_BUSY				((U32)0x000000B5)
#define TASK_STA_RDY				((U32)0x000000D7)
#define TASK_STA_ERR				((U32)0x000000E0)

#define TASK_STA_MASK				((U32)0x0000FFFF)

#define TASK_NAME_ETH   		((U32)0x5A010000)
#define TASK_NAME_GEN   		((U32)0x5A020000)
#define TASK_NAME_PWN   		((U32)0x5A030000)

#define TASK_NAME_SWH   		((U32)0x5A100000)
#define TASK_NAME_RAS   		((U32)0x5A110000)
#define TASK_NAME_SWV   		((U32)0x5A120000)
#define TASK_NAME_PROP  		((U32)0x5A130000)

#define TASK_NAME_TOP   		((U32)0x5A220000)
#define TASK_NAME_SHED   		((U32)0x5A230000)

#define TASK_NAME_LEVE   		((U32)0x5A310000)
#define TASK_NAME_VISL  		((U32)0x5A320000)

#define TASK_NAME_MOVF   		((U32)0x5A610000)
#define TASK_NAME_MOVB  		((U32)0x5A620000)

#define TASK_NAME_MASK  		((U32)0xFFFF0000)


#define FLG_PWN   	((U32)0x00000001)
#define FLG_GENd  	((U32)0x00000002)
#define FLG_GENs  	((U32)0x00000004)
#define FLG_ETH  		((U32)0x00000008)
#define FLG_SWh   	((U32)0x00000010)
#define FLG_RAS   	((U32)0x00000020)
#define FLG_SWv   	((U32)0x00000040)
#define FLG_PROP  	((U32)0x00000080)

#define FLG_TOP   	((U32)0x00000100)
#define FLG_SHED  	((U32)0x00000200)
#define FLG_LEVE  	((U32)0x00001000)
#define FLG_VISL  	((U32)0x00002000)

#define FLG_MOVF  	((U32)0x00010000)
#define FLG_MOVB  	((U32)0x00100000)

#define FLG_BUSY 		((U32)0x00200000)

#define FLGx_ALL  	((U32)0x001133FB)


#define FLGx_PWN	(FLG_ETH|FLG_GENd)
#define FLGx_SWH	(FLG_GENd|FLG_PWN|FLG_ETH|FLG_SWh)
#define FLGx_RAS	(FLG_GENd|FLG_PWN|FLG_ETH|FLG_SWh)
#define FLGx_SWH_RAS	(FLG_GENd|FLG_PWN|FLG_ETH|FLG_SWh)
#define FLGx_SWV	(FLG_GENd|FLG_PWN|FLG_ETH|FLG_SWh)
#define FLGx_PROP	(FLG_GENd|FLG_PWN|FLG_ETH|FLG_SWh)
#define FLGx_TOP	(FLG_GENd|FLG_PWN|FLG_ETH|FLG_SWh)
#define FLGx_SHED	(FLG_GENd|FLG_PWN|FLG_ETH|FLG_SWh)
#define FLGx_LEVE	(FLG_GENd|FLG_PWN|FLG_ETH|FLG_SWh)
#define FLGx_MOVxyz	(FLG_GENd|FLG_PWN|FLG_ETH|FLG_SWh)



#define ERR_CAN_RX_MAX  ((U32)1000)


typedef struct sMSG_USER{
  U32 Typ;
  U32 Dat;
}sMSG_USER;



typedef struct sCxBUF{
  struct sCxBUF *Next;
  U32   Sta;
  U32   TimStamp;
  U8    Rec1U8;
  U8    Rec2U8;
  U8    Rec3U8;
  U8    FrmInfo;
  U32   ID32;
  U32   Dat03;
  U32   Dat47;
}sCxBUF;


typedef struct sCxID32{
  U8 SAddr;
  U8 DAddr;
  U8 PForm;
  U8 Page;
}sCxID32;

typedef struct sCTxCMD{
  U32   ID32;
  U32   Dat03;
  U32   Dat47;
}sCTxCMD;




typedef struct sCRxPWN_IO{
  U8 	SAddr;
  U8 	DAddr;
  U8 	PForm;
  U8 	Page;
  U32 P24Ctrl;
  U16 P50Ctrl;
  I16 Amp;
  U8  ErrCode;
  U8  Xor;
}sCRx_PWN_IO;


typedef struct sCRxPWN_VOL{
  U8 SAddr;
  U8 DAddr;
  U8 PForm;
  U8 Page;
  U16 P24Vol; 
  U16 P24Amp;
 	U16 P50Vol;
  U16 P50Amp; 
}sCRx_PWN_VOL;

#define LLT_PWN_LEN  ((U32)16)

typedef struct sPWN_DAT{
struct sPWN_DAT *Next;
  					U32 Dat;					
}sPWN_DAT;

#define LLT_SEV_LEN  ((U32)4)
#define SnLF ((U32)0)
#define SnLB ((U32)1)
#define SnRF ((U32)2)
#define SnRB ((U32)3)

#define SnLFB ((U32)0)
#define SnRFB ((U32)1)
#define SnLRx	((U32)2)

#define SnSYN ((U32)4)

#define SnNULL ((U32)0x80)

#define TIM_WAIT_STOP_5S ((U32)125)//125/25 = 5S
#define TIM_WAIT_MOMENT_300S ((U32)7500)//125/25 = 300S

typedef struct sCRxSERVO{
  struct sCRxSERVO *Next; 
	U8  SAddr;
  U8  DAddr;
  U8  PForm;
  U8  Page;
  I16 Pos;
  I16 Spd;
  I16 Amp;
  U8  ErrCode;
  U8  Sta;
}sCRxSERVO;


#define SEV_SPD_RECU16	((U16)0x0011)
#define SEV_SPD_RECU32	((U32)0x00112233)
#define SEV_SPD_ACC 		((U16)1000)//1000ms run up to V
#define SEV_BRAKE_CLOSE ((U8)0x3C)
#define SEV_BRAKE_OPEN 	((U8)0xC3)
#define SEV_SPD_DLY0 		((U8)0x00)

typedef struct sCTxSEV_SPD{ 
	U32 ID32; 
  U16 RecU16;
  I16 Spd;
  I16 Acc; 
  U8  Dly;
  U8  En;
}sCTxSEV_SPD;


typedef struct sCTxSEV_AMP{ 
	U32 ID32; 
  U32 RecU32;
  I16 Amp;
  U8  Dly;
  U8  En;
}sCTxSEV_AMP;




typedef struct sSERVO_DAT{ 
  I32 CurI32;
  I32 AvgI32;
	I32 SumI32;
}sSERVO_DAT;


typedef struct sSERVO_ERR{ 
 	U8 U8Code;
 	U8 U8Cur;
 	U8 U8Prev1;
 	U8 U8Prev2;
}sSERVO_ERR;



typedef struct sPOS_LMT{ 
 	I32 Max;//Position switch
	I32 Con;//Condition Dot
 	I32 Min;////Position switch
}sDAT_LMT;


typedef struct sSTA_FILT{ 
 	U32 Err;
	U32 Noml;
}sSTA_FILT;















/*

  typedef struct StrCanTx{
  struct StrCanTx* Next;
  struct StrCanBuf* C0Node;
  struct StrCanBuf* C1Node;
  }StrCanTx;





  typedef struct StrCanRx{
  struct StrCanRx* Next;
  U8            FrmInfo;
  U8            ID32[4];
  U8            Dat[8];
  U8            Status;
  U16   TimStamp;
  }StrCanRx;

*/
