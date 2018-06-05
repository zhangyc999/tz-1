typedef  double 			D64;
typedef  unsigned int U32;
typedef           int I32;
typedef  unsigned short U16;
typedef           short I16;
typedef  unsigned char U8;
typedef           char I8;


#define SEV_SWhLF_POS_MAX  	((I32)1000)
#define SEV_SWhLF_POS_CON 	((I32)200)
#define SEV_SWhLF_POS_MIN  	((I32)0)

#define SEV_SWhLB_POS_MAX  	((I32)1000)
#define SEV_SWhLB_POS_CON 	((I32)200)
#define SEV_SWhLB_POS_MIN  	((I32)0)

#define SEV_SWhRF_POS_MAX  	((I32)1000)
#define SEV_SWhRF_POS_CON 	((I32)200)
#define SEV_SWhRF_POS_MIN  	((I32)0)

#define SEV_SWhRB_POS_MAX  	((I32)1000)
#define SEV_SWhRB_POS_CON  	((I32)200)
#define SEV_SWhRB_POS_MIN  	((I32)0)

#define SEV_SWhLR_SPD_MAXp  ((I32)+1000)
#define SEV_SWhLR_SPD_MAXn  ((I32)-1000)

#define SEV_SWLR_SPD_ZERO 	((I32)2)


#define SEV_SWhLR_AMP_MAXp  ((I32)+100)
#define SEV_SWhLR_AMP_MAXn  ((I32)-100)

#define SEV_SWh_SYNC_FB_ERR_MAX ((I32)+100)
#define SEV_SWh_SYNC_LR_ERR_MAX ((I32)+200)


#define SEV_SWhLR_CON_FILT_MAX 	((U32)40)
#define SEV_SWhLR_POS_FILT_MAX  ((U32)20)
#define SEV_SWhLR_SPD_FILT_MAX  ((U32)20)
#define SEV_SWhLR_AMP_FILT_MAX  ((U32)20)
#define SEV_SWhLR_COD_FILT_MAX  ((U32)20)
#define SEV_SWhLR_SYNC_FILT_MAX ((U32)20)
#define SEV_SWLR_SPD_ZERO_FILT_MAX 	((I32)50)

#define SEV_SWhLR_REMN_LEN ((U32)20)


//Swing 0<=>330mm Unit = 0.2mm

//		      ====|| ||====
//	 ssss /				   	 	  \ ssss
//__|		 |							   |    |__
//	|		 |								 |	  |


//#define SEV_PLAN_SWH_SPD_UP_LEN 	((I32)1000)
//#define SEV_PLAN_SWH_SPD_DOWN_LEN ((I32)1000)

#define SEV_PLAN_SWH_LEN_SLOW_20mm		((I32) 100)//50*0.2=20.0mm Unit 0.2mm
#define SEV_PLAN_SWH_LEN_MOMENT_3mm		((I32)  15)//15*0.2=3.0mm Unit 0.2mm

#define SEV_PLAN_SWH_SPD_SLOW 		((I32) 100)// 1.0mm/s 	Unit 0.01mm/s
#define SEV_PLAN_SWH_SPD_FAST 		((I32)1000)//10.0mm/s 	Unit 0.01mm/s

#define SEV_PLAN_SWH_ACCp 				((I32) 1000)//5.0mm/(s^2) 	Unit 0.01mm/s^2
#define SEV_PLAN_SWH_ACCn 				((I32) 1000)//5.0mm/(s^2) 	Unit 0.01mm/s^2


#define SEV_PLAN_SWHA_ACCp_Dt 			((I32) 20)//
#define SEV_PLAN_SWHA_ACCn_Dt 			((I32) 20)//

#define SEV_PLAN_SWHM_ACCp_Dt 			((I32) 10)//
#define SEV_PLAN_SWHM_ACCn_Dt 			((I32) 10)//





#define SEV_PLAN_SWH_MOMENT_AMP		((I32) 200)//200*0.01 = 2A	Unit 0.01a

#define SEV_PLAN_SWH_POS_MAX  	((I32)1000)
#define SEV_PLAN_SWH_POS_MIN  	((I32)0)

#define SEV_PLAN_EN  		((I32)0x00E0AB0E)
#define SEV_PLAN_DIS 		((I32)0xD1E0AB0E)



#define SEV_PLAN_MOD_ARUNp  	((U32)0x53910D01)
#define SEV_PLAN_MOD_ARUNpF  	((U32)0x53910D02)
#define SEV_PLAN_MOD_ARUNn  	((U32)0x53910D03)
#define SEV_PLAN_MOD_ARUNnF  	((U32)0x53910D04)

#define SEV_PLAN_MOD_MRUNp  	((U32)0x53910D10)
#define SEV_PLAN_MOD_MRUNpF  	((U32)0x53910D20)
#define SEV_PLAN_MOD_MRUNn  	((U32)0x53910D30)
#define SEV_PLAN_MOD_MRUNnF  	((U32)0x53910D40)

#define SEV_PLAN_MOD_IDLE  		((U32)0x53910D00)


#define SEV_CMD_AUTOp  	((U32)0x5ECA0000)
#define SEV_CMD_AUTOn  	((U32)0x5ECA1000)

#define SEV_CMD_MANUp  	((U32)0x5ECD0000)
#define SEV_CMD_MANUn  	((U32)0x5ECD0000)


#define SEV_DAT_POS  		((U32)0x5ED95000)
#define SEV_DAT_ERR  		((U32)0x5EDE1000)
#define SEV_DAT_SPD  		((U32)0x5ED59000)
#define SEV_DAT_AMP  		((U32)0x5EDA9000)
#define SEV_DAT_RDY  		((U32)0x5EDDA000)
#define SEV_DAT_BUSY 		((U32)0x5EDB5000)

#define SEV_DAT_IDLE 		((U32)0x5ED1D000)


#define SMOD_STOP 		((U32)0x50D00001)
#define SMOD_ARUNp 		((U32)0x50D000A0)
#define SMOD_ARUNn 		((U32)0x50D000A8)
#define SMOD_MRUNp 		((U32)0x50D000B0)
#define SMOD_MRUNn 		((U32)0x50D000B8)



#define FLG_SEV_LenMAX  			((U32)0x00000001)
#define FLG_SEV_LenLMT				((U32)0x00000002)
#define FLG_SEV_LenMIN				((U32)0x00000004)

#define FLG_SEV_STOP					((U32)0x00000010)
#define FLG_SEV_RUN						((U32)0x00000020)
#define FLG_SEV_EN						((U32)0x00000040)


#define FLG_SEV_ERR_POS				((U32)0x00010000)
#define FLG_SEV_ERR_SPD				((U32)0x00020000)
#define FLG_SEV_ERR_AMP				((U32)0x00040000)
#define FLG_SEV_ERR_COD				((U32)0x00080000)
#define FLG_SEV_ERR_MASK			((U32)0x000F0000)


#define FLG_SEV_ERR_SYNC_L		((U32)0x00100000)
#define FLG_SEV_ERR_SYNC_R		((U32)0x00200000)
#define FLG_SEV_ERR_SYNC_LR		((U32)0x00400000)
#define FLG_SEV_ERR_SYNC_MASK	((U32)0x00700000)

