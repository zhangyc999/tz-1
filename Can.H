typedef  double D64;
typedef  unsigned int U32;
typedef           int I32;
typedef  unsigned short U16;
typedef           short I16;
typedef  unsigned char U8;
typedef           char I8;



#define CSEV_IMM    ((U8)0x9A)
#define CSEV_DLY    ((U8)0xA9)
#define CSEV_EN     ((U8)0xC3)
#define CSEV_DIS    ((U8)0x3C)

#define ExPF_SPD   	((U8)0xA5)
#define ExPF_POS   	((U8)0xA6)
#define ExPF_AMP   	((U8)0xA3)
#define ExPAGE     	((U8)0x08)


#define ExSAD_HOST      ((U8)0x0C)
#define ExSAD_VISL_F    ((U8)0x13)
#define ExSAD_VISL_B    ((U8)0x15)

#define ExSAD_LEVEL0    ((U8)0x16)
#define ExSAD_LEVEL1    ((U8)0x19)


#define ExSAD_MOV_FX    ((U8)0x2C)
#define ExSAD_MOV_FY0   ((U8)0x25)
#define ExSAD_MOV_FY1   ((U8)0x26)
#define ExSAD_MOV_FZ    ((U8)0x20)

#define ExSAD_MOV_BX    ((U8)0x2F)
#define ExSAD_MOV_BY0   ((U8)0x29)
#define ExSAD_MOV_BY1   ((U8)0x2A)
#define ExSAD_MOV_BZ    ((U8)0x23)

#define ExSAD_SW_HLF    ((U8)0x49)
#define ExSAD_SW_HLB    ((U8)0x4A)
#define ExSAD_SW_HRF    ((U8)0x4C)
#define ExSAD_SW_HRB    ((U8)0x4F)

#define ExSAD_SW_VLF    ((U8)0x40)
#define ExSAD_SW_VLB    ((U8)0x43)
#define ExSAD_SW_VRF    ((U8)0x45)
#define ExSAD_SW_VRB    ((U8)0x46)

#define ExSAD_PROP_LF   ((U8)0x30)
#define ExSAD_PROP_LB   ((U8)0x33)
#define ExSAD_PROP_RF   ((U8)0x35)
#define ExSAD_PROP_RB   ((U8)0x36)

#define ExSAD_RAISE_LF  ((U8)0x39)
#define ExSAD_RAISE_LB  ((U8)0x3A)
#define ExSAD_RAISE_RF  ((U8)0x3C)
#define ExSAD_RAISE_RB  ((U8)0x3F)

#define ExSAD_NM_LF     ((U8)0x83)
#define ExSAD_NM_LB     ((U8)0x85)
#define ExSAD_NM_RF     ((U8)0x86)
#define ExSAD_NM_RB     ((U8)0x89)


#define ExSAD_GEN_DIESEL   ((U8)0x52)

#define ExSAD_GEN_SHAFT    ((U8)0x66)

#define ExSAD_POW_CTRL     ((U8)0x73)


#define ExSAD_TOP           ((U8)0x93)

#define ExSAD_SIDE_LF       ((U8)0xA3)
#define ExSAD_SIDE_LB       ((U8)0xA5)
#define ExSAD_SIDE_RF       ((U8)0xA6)
#define ExSAD_SIDE_RB       ((U8)0xA9)

#define ExSAD_END_F1     ((U8)0xB0)
#define ExSAD_END_F2     ((U8)0xB3)
#define ExSAD_END_F3     ((U8)0xB5)
#define ExSAD_END_F4     ((U8)0xB6)
#define ExSAD_END_B1     ((U8)0xB9)
#define ExSAD_END_B2     ((U8)0xBA)
#define ExSAD_END_B3     ((U8)0xBC)
#define ExSAD_END_B4     ((U8)0xBF)

#define ExSAD_Recor      ((U8)0xEC)
#define ExSAD_NetAdmin   ((U8)0xFE)
#define ExSAD_Broadcast  ((U8)0xFF)



#define C0BUF_VIS1F       ((U32)0)
#define C0BUF_VIS1B       ((U32)1)

#define C0BUF_LEVEL0      ((U32)2)
#define C0BUF_LEVEL1      ((U32)3)


#define C0BUF_MOV_XF      ((U32)5)
#define C0BUF_MOV_YF0     ((U32)6)
#define C0BUF_MOV_YF1     ((U32)7)
#define C0BUF_MOV_ZF      ((U32)8)

#define C1BUF_MOV_XB      ((U32)1)
#define C1BUF_MOV_YB0     ((U32)2)
#define C1BUF_MOV_YB1     ((U32)3)
#define C1BUF_MOV_ZB      ((U32)4)

#define C0BUF_SW_HLF      ((U32)9)
#define C0BUF_SW_HLB      ((U32)10)
#define C1BUF_SW_HRF      ((U32)5)
#define C1BUF_SW_HRB      ((U32)6)

#define C0BUF_SW_VLF      ((U32)11)
#define C0BUF_SW_VLB      ((U32)12)
#define C1BUF_SW_VRF      ((U32)7)
#define C1BUF_SW_VRB      ((U32)8)

#define C0BUF_PROP_LF      ((U32)13)
#define C0BUF_PROP_LB      ((U32)14)
#define C1BUF_PROP_RF      ((U32)9)
#define C1BUF_PROP_RB      ((U32)10)

#define C1BUF_RAISE_LF     ((U32)11)
#define C1BUF_RAISE_LB     ((U32)12)
#define C1BUF_RAISE_RF     ((U32)13)
#define C1BUF_RAISE_RB     ((U32)14)

#define C1BUF_GEN_DIESEL   ((U32)15)
#define C1BUF_GEN_SHAFT    ((U32)16)

#define C0BUF_POW_CTRL      ((U32)15)


#define C1BUF_NM_LF   ((U32)17)
#define C1BUF_NM_LB   ((U32)18)
#define C1BUF_NM_RF   ((U32)19)
#define C1BUF_NM_RB   ((U32)20)

#define C1BUF_TOP        ((U32)21)

#define C0BUF_SIDE_LF     ((U32)17)
#define C0BUF_SIDE_LB     ((U32)18)
#define C0BUF_SIDE_RF     ((U32)19)
#define C0BUF_SIDE_RB     ((U32)20)

#define C1BUF_END_F1     ((U8)22)
#define C1BUF_END_F2     ((U8)23)
#define C1BUF_END_F3     ((U8)24)
#define C1BUF_END_F4     ((U8)25)

#define C0BUF_END_B1     ((U8)21)
#define C0BUF_END_B2     ((U8)22)
#define C0BUF_END_B3     ((U8)23)
#define C0BUF_END_B4     ((U8)24)

#define C1BUF_Recor       ((U32)0xEC)
#define C1BUF_NetAdmin    ((U32)0xFE)
#define C1BUF_Broadcast   ((U32)0xFF)


#define PF_STA_POW_VOL   	((U8)0xC0)
#define PF_STA_POW_IO   	((U8)0xC3)

#define PF_CMD_SEVO_ZERO 	((U8)0xA9)
#define PF_STA_SEVO_ZERO 	((U8)0xC0)
#define PF_CMD_SEVO_POS 	((U8)0xA6)
#define PF_CMD_SEVO_SPD 	((U8)0xA5)
#define PF_CMD_SEVO_AMP 	((U8)0xA3)
#define PF_CMD_SEVO_Dt  	((U8)0xA0)
#define PF_STA_SEVO_DAT 	((U8)0xC6)
#define PF_STA_SEVO_ERR 	((U8)0x7F)


#define C0TxID32_SWH_Q_LF   ((U32)0x62E24860)
#define C0TxID32_SWH_Q_LB   ((U32)0x62E25060)
#define C1TxID32_SWH_Q_RF   ((U32)0x62E26060)
#define C1TxID32_SWH_Q_RB   ((U32)0x62E27860)

#define C0TxID32_SWH_V_LF   ((U32)0x452A4860)
#define C0TxID32_SWH_V_LB   ((U32)0x452A5060)
#define C1TxID32_SWH_V_RF   ((U32)0x452A6060)
#define C1TxID32_SWH_V_RB   ((U32)0x452A7860)


#define C0TxID32_SWH_P_LF   ((U32)0x45324860)
#define C0TxID32_SWH_P_LB   ((U32)0x45325060)
#define C1TxID32_SWH_P_RF   ((U32)0x45326060)
#define C1TxID32_SWH_P_RB   ((U32)0x45327860)


#define C0TxID32_SWH_I_LF   ((U32)0x451A4860)
#define C0TxID32_SWH_I_LB   ((U32)0x451A5060)
#define C1TxID32_SWH_I_RF   ((U32)0x451A6060)
#define C1TxID32_SWH_I_RB   ((U32)0x451A7860)

#define C0TxID32_SWH_Dt_LF   ((U32)0x45024860)
#define C0TxID32_SWH_Dt_LB   ((U32)0x45025060)
#define C1TxID32_SWH_Dt_RF   ((U32)0x45026060)
#define C1TxID32_SWH_Dt_RB   ((U32)0x45027860)


#define C0RxID32_PWN_VOL   	((U32)0x14C30C73)//=>3BIT
#define C0RxID32_PWN_IO   	((U32)0x18C00C73)//=>3BIT
#define C0TxID32_PWN_Q  		((U32)0x62E39860)
#define C0TxID32_PWN_IO			((U32)0x45039860)




#define ADDR_GPIO_OUT 	 	((U32)0x000DA000)
#define ADDR_GPIO_IN 	 		((U32)0x000D9000)


#define ADDR_CMD_CAN0  ((U32)0x000D1000)
#define ADDR_DAT_CAN0  ((U32)0x000D2000)
#define ADDR_CMD_CAN1  ((U32)0x000D3000)
#define ADDR_DAT_CAN1  ((U32)0x000D4000)

#define IRQ5_CAN0  ((I32)5)
#define IRQ7_CAN1  ((I32)7)

#define PELI_MOD   ((U8)0x00)
#define PELI_CMR   ((U8)0x01)
#define PELI_SR    ((U8)0x02)
#define PELI_IR    ((U8)0x03)
#define PELI_IER   ((U8)0x04)
#define PELI_BTR0  ((U8)0x06)
#define PELI_BTR1  ((U8)0x07)
#define PELI_OCR   ((U8)0x08)
#define PELI_ALC   ((U8)0x0B)
#define PELI_ECC   ((U8)0x0C)
#define PELI_EWLR  ((U8)0x0D)
#define PELI_RXERR ((U8)0x0E)
#define PELI_TXERR ((U8)0x0F)
#define PELI_TXB0  ((U8)0x10)
#define PELI_TXB1  ((U8)0x11)
#define PELI_TXB2  ((U8)0x12)
#define PELI_TXB3  ((U8)0x13)
#define PELI_TXB4  ((U8)0x14)
#define PELI_TXB5  ((U8)0x15)
#define PELI_TXB6  ((U8)0x16)
#define PELI_TXB7  ((U8)0x17)
#define PELI_TXB8  ((U8)0x18)
#define PELI_TXB9  ((U8)0x19)
#define PELI_TXB10 ((U8)0x1A)
#define PELI_TXB11 ((U8)0x1B)
#define PELI_TXB12 ((U8)0x1C)
#define PELI_RXB0  ((U8)0x10)
#define PELI_RXB1  ((U8)0x11)
#define PELI_RXB2  ((U8)0x12)
#define PELI_RXB3  ((U8)0x13)
#define PELI_RXB4  ((U8)0x14)
#define PELI_RXB5  ((U8)0x15)
#define PELI_RXB6  ((U8)0x16)
#define PELI_RXB7  ((U8)0x17)
#define PELI_RXB8  ((U8)0x18)
#define PELI_RXB9  ((U8)0x19)
#define PELI_RXB10 ((U8)0x1A)
#define PELI_RXB11 ((U8)0x1B)
#define PELI_RXB12 ((U8)0x1C)
#define PELI_ACR0  ((U8)0x10)
#define PELI_ACR1  ((U8)0x11)
#define PELI_ACR2  ((U8)0x12)
#define PELI_ACR3  ((U8)0x13)
#define PELI_AMR0  ((U8)0x14)
#define PELI_AMR1  ((U8)0x15)
#define PELI_AMR2  ((U8)0x16)
#define PELI_AMR3  ((U8)0x17)
#define PELI_RMC   ((U8)0x1D)
#define PELI_RBSA  ((U8)0x1E)
#define PELI_CDR   ((U8)0x1F)







/*
  +-----------------------+
  | BPS     | BTR0 | BTR1 |
  +---------+------+------+
  | 5Kbps   | 0xBF | 0xFF |
  | 10Kbps  | 0x31 | 0x1C |
  | 20Kbps  | 0x18 | 0x1C |
  | 40Kbps  | 0x87 | 0xFF |
  | 50Kbps  | 0x09 | 0x1C |
  | 80Kbps  | 0x83 | 0Xff |
  | 100Kbps | 0x04 | 0x1C |
  | 125Kbps | 0x03 | 0x1C |
  | 200Kbps | 0x81 | 0xFA |
  | 250Kbps | 0x01 | 0x1C |
  | 400Kbps | 0x80 | 0xFA |
  | 500Kbps | 0x00 | 0x1C |
  | 666Kbps | 0x80 | 0xB6 |
  | 800Kbps | 0x00 | 0x16 |
  | 1Mbps   | 0x00 | 0x14 |
  +---------+------+------+
*/
