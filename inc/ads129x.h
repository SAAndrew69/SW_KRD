#ifndef __ADS1298_H
#define __ADS1298_H

#ifdef __cplusplus
 extern "C" {
#endif

typedef enum {
  ADS_CONVERT_16_8,
  ADS_CONVERT_16_16,
  ADS_CONVERT_24_16,
  ADS_CONVERT_24_24,
  ADS_CONVERT_NONE
} ads_convert_t;

/* ADS129x Configuration */

#define ADS1298_R_CONFIG1  HIGH_RES_500_SPS /* (0x86) = (HR  | DR2 | DR1 )     */

#define ADS1298_R_CONFIG3  (     /* ( 0xDC )  */  \
  ADS129X_CONFIG3_PD_REFBUF   |  /* (1 << 7)  */  \
  ADS129X_CONFIG3_RLD_MEAS    |  /* (1 << 4)  */  \
  ADS129X_CONFIG3_RLDREF_INT  |  /* (1 << 3)  */  \
  ADS129X_CONFIG3_PD_RLD         /* (1 << 2), */  \
)

#define ADS1298_WORKING_CONFIG {      \
  .CONFIG1       = ADS1298_R_CONFIG1, \
  .CONFIG3	 = ADS1298_R_CONFIG3, \
  .LOFF	         = 0x03,              \
  .CH1SET        = GAIN_1X,           \
  .CH2SET        = GAIN_1X,           \
  .CH3SET        = GAIN_1X,           \
  .CH4SET        = GAIN_1X,           \
  .CH5SET        = GAIN_1X,           \
  .CH6SET        = GAIN_1X,           \
  .CH7SET        = GAIN_1X,           \
  .CH8SET        = GAIN_1X,           \
  .LOFF_SENSP    = 0xFF,              \
  .LOFF_SENSN    = 0x02,              \
  .RESP	         = 0xF0,              \
  .CONFIG4	 = 0x22,              \
  .WCT1	         = 0x0A,              \
  .WCT2	         = 0xE3               \
}

#if !0 
  #define ADS1298_DEFAULT_CONFIG {    \
    .ID          = 0x92,              \
    .CONFIG1     = HIGH_RES_32k_SPS,  \
    .CONFIG3     = CONFIG3_CONST,     \
    .GPIO        = GPIO_CONST,        \
  }
#else
  #define ADS1298_CONFIG {            \
    .ID          = 0x92,              \
    .CONFIG1     = 0x06,              \
    .CONFIG2     = 0x00,              \
    .CONFIG3     = 0x40,              \
    .LOFF        = 0x00,              \
    .CH1SET      = 0x00,              \
    .CH2SET      = 0x00,              \
    .CH3SET      = 0x00,              \
    .CH4SET      = 0x00,              \
    .CH5SET      = 0x00,              \
    .CH6SET      = 0x00,              \
    .CH7SET      = 0x00,              \
    .CH8SET      = 0x00,              \
    .RLD_SENSP   = 0x00,              \
    .RLD_SENSN   = 0x00,              \
    .LOFF_SENSP  = 0x00,              \
    .LOFF_SENSN  = 0x00,              \
    .LOFF_FLIP   = 0x00,              \
    .LOFF_STATP  = 0x00,              \
    .LOFF_STATN  = 0x00,              \
    .GPIO        = 0x0F,              \
    .PACE        = 0x00,              \
    .RESP        = 0x00,              \
    .CONFIG4     = 0x00,              \
    .WCT1        = 0x00,              \
    .WCT2        = 0x00               \
  }
#endif

#define ADS1298_CONFIG ADS1298_WORKING_CONFIG

enum ADS129X_REGISTER_POOL_SIZE {
  ADS129X_REG_POOL_SIZE = 26
};

enum ADS129X_SPI_COMMAND {
  /* System Commands */
  ADS129X_WAKEUP     = 0x02,
  ADS129X_STANDBY    = 0x04,
  ADS129X_RESET      = 0x06,
  ADS129X_START      = 0x08,
  ADS129X_STOP       = 0x0a,
  /* Read Commands */
  ADS129X_RDATAC     = 0x10,
  ADS129X_SDATAC     = 0x11,
  ADS129X_RDATA      = 0x12,
  /* Register Commands */
  ADS129X_RREG       = 0x20,
  ADS129X_WREG       = 0x40
};

enum ADS129X_REG {
  /* Device Settings */
  ADS129X_ID         = 0x00,
  /* Global Settings */
  ADS129X_CONFIG1    = 0x01,
  ADS129X_CONFIG2    = 0x02,
  ADS129X_CONFIG3    = 0x03,
  ADS129X_LOFF       = 0x04,
  /* Channel Specific Settings */
  ADS129X_CH_SET     = 0x04,
  ADS129X_CH1SET     = ADS129X_CH_SET + 1,
  ADS129X_CH2SET     = ADS129X_CH_SET + 2,
  ADS129X_CH3SET     = ADS129X_CH_SET + 3,
  ADS129X_CH4SET     = ADS129X_CH_SET + 4,
  ADS129X_CH5SET     = ADS129X_CH_SET + 5,
  ADS129X_CH6SET     = ADS129X_CH_SET + 6,
  ADS129X_CH7SET     = ADS129X_CH_SET + 7,
  ADS129X_CH8SET     = ADS129X_CH_SET + 8,
  ADS129X_RLD_SENSP  = 0x0D,
  ADS129X_RLD_SENSN  = 0x0E,
  ADS129X_LOFF_SENSP = 0x0F,
  ADS129X_LOFF_SENSN = 0x10,
  ADS129X_LOFF_FLIP  = 0x11,
  /* Lead Off Status */
  ADS129X_LOFF_STATP = 0x12,
  ADS129X_LOFF_STATN = 0x13,
  /* Other */
  ADS129X_GPIO       = 0x14,
  ADS129X_PACE       = 0x15,
  ADS129X_RESP       = 0x16,
  ADS129X_CONFIG4    = 0x17,
  ADS129X_WCT1       = 0x18,
  ADS129X_WCT2       = 0x19
};

#if 0
enum ADS129X_CONFIG1_BIT {
  ADS129X_CONFIG1_HR     = (1 << 7),
  ADS129X_CONFIG1_NDAISY = (1 << 6),
  ADS129X_CONFIG1_CLK    = (1 << 5),
  ADS129X_CONFIG1_DR     = (7 << 0)
};

enum ADS129X_CONFIG2_BIT {
  ADS129X_CONFIG2_WCT_CHOP  = (1 << 5),
  ADS129X_CONFIG2_INT_TEST  = (1 << 4),
  ADS129X_CONFIG2_TEST_AMP  = (1 << 2),
  ADS129X_CONFIG2_TEST_FREQ = (3 << 0)
};

enum ADS129X_CONFIG3_BIT {
  ADS129X_CONFIG3_PD_REFBUF     = (1 << 7),
  ADS129X_CONFIG3_VREF_4V       = (1 << 5),
  ADS129X_CONFIG3_RLD_MEAS      = (1 << 4),
  ADS129X_CONFIG3_RLDREF_INT    = (1 << 3),
  ADS129X_CONFIG3_PD_RLD        = (1 << 2),
  ADS129X_CONFIG3_RLD_LOFF_SENS = (1 << 1),
  ADS129X_CONFIG3_RLD_STAT      = (1 << 0)
};

// LOFF: Lead-Off Control Register (address = 04h) (reset = 00h)
enum ADS129X_LOFF_BIT {
  ADS129X_LOFF_COMP_TH      = (7 << 5),
  ADS129X_LOFF_VLEAD_OFF_EN = (1 << 4),
  ADS129X_LOFF_ILEAD_OFF    = (3 << 2),
  ADS129X_LOFF_FLEAD_OFF    = (3 << 0)
};

// CHnSET: Individual Channel Settings (n = 1 to 8) (address = 05h to 0Ch) (reset = 00h)
//enum ADS129X_LOFF_BIT {
#endif

enum ADS129X_ID_BIT {
  DEV_ID7          = 0x80,
  DEV_ID6          = 0x40,
  DEV_ID5          = 0x20,
  DEV_ID4          = 0x10,
  DEV_ID2          = 0x04,
  DEV_ID1          = 0x02,
  DEV_ID0          = 0x01,

  ID_CONST         = 0x10,
  ID_ADS129x       = (DEV_ID7 | DEV_ID4),
  ID_ADS129xR      = (DEV_ID7 | DEV_ID6),

  ID_4CHAN         = 0,
  ID_6CHAN         = DEV_ID0,
  ID_8CHAN         = DEV_ID1,

  ID_ADS1294       = (ID_ADS129x  | ID_4CHAN),
  ID_ADS1296       = (ID_ADS129x  | ID_6CHAN),
  ID_ADS1298       = (ID_ADS129x  | ID_8CHAN),
  ID_ADS1294R      = (ID_ADS129xR | ID_4CHAN),
  ID_ADS1296R      = (ID_ADS129xR | ID_6CHAN),
  ID_ADS1298R      = (ID_ADS129xR | ID_8CHAN)
};

enum ADS129X_CONFIG1_BIT {
  HR               = 0x80,
  DAISY_EN         = 0x40,
  CLK_EN           = 0x20,
  DR2              = 0x04,
  DR1              = 0x02,
  DR0              = 0x01,
  
  HIGH_RES_32k_SPS = HR,
  HIGH_RES_16k_SPS = (HR  | DR0       ),
  HIGH_RES_8k_SPS  = (HR  | DR1       ),
  HIGH_RES_4k_SPS  = (HR  | DR1 | DR0 ),
  HIGH_RES_2k_SPS  = (HR  | DR2       ),
  HIGH_RES_1k_SPS  = (HR  | DR2 | DR0 ),
  HIGH_RES_500_SPS = (HR  | DR2 | DR1 ),
  LOW_PWR_250_SPS  = (DR2 | DR1       ),

  CONFIG1_CONST    = LOW_PWR_250_SPS
};

enum ADS129X_CONFIG2_BIT {
  WCT_CHOP         = 0x20,
  INT_TEST         = 0x10,
  TEST_AMP         = 0x04,
  TEST_FREQ1       = 0x02,
  TEST_FREQ0       = 0x01,
  
  CONFIG2_CONST    = 0x00,

  INT_TEST_4HZ     = INT_TEST, 
  INT_TEST_8HZ     = (INT_TEST | TEST_FREQ0),
  INT_TEST_DC      = (INT_TEST | TEST_FREQ1 | TEST_FREQ0)
};

enum ADS129X_CONFIG3_BIT {
  ADS129X_CONFIG3_PD_REFBUF        = 0x80,
  ADS129X_CONFIG3_VREF_4V          = 0x20,
  ADS129X_CONFIG3_RLD_MEAS         = 0x10,
  ADS129X_CONFIG3_RLDREF_INT       = 0x08,
  ADS129X_CONFIG3_PD_RLD           = 0x04,
  ADS129X_CONFIG3_RLD_LOFF_SENS    = 0x02,
  ADS129X_CONFIG3_RLD_STAT         = 0x01,

  CONFIG3_CONST    = 0x40
};

enum ADS129X_LOFF_BIT {
  COMP_TH2         = 0x80,
  COMP_TH1         = 0x40,
  COMP_TH0         = 0x20,
  VLEAD_OFF_EN     = 0x10,
  ILEAD_OFF1       = 0x08,
  ILEAD_OFF0       = 0x04,
  FLEAD_OFF1       = 0x02,
  FLEAD_OFF0       = 0x01,
                   
  LOFF_CONST       = 0x00,
                   
  COMP_TH_95       = 0x00,
  COMP_TH_92_5     = COMP_TH0,
  COMP_TH_90       = COMP_TH1,
  COMP_TH_87_5     = (COMP_TH1 | COMP_TH0),
  COMP_TH_85       = COMP_TH2,
  COMP_TH_80       = (COMP_TH2 | COMP_TH0),
  COMP_TH_75       = (COMP_TH2 | COMP_TH1),
  COMP_TH_70       = (COMP_TH2 | COMP_TH1 | COMP_TH0),
                   
  ILEAD_OFF_6NA    = 0x00,
  ILEAD_OFF_12NA   = ILEAD_OFF0,
  ILEAD_OFF_18NA   = ILEAD_OFF1,
  ILEAD_OFF_24NA   = (ILEAD_OFF1 | ILEAD_OFF0),
                   
  FLEAD_OFF_AC     = FLEAD_OFF0,
  FLEAD_OFF_DC     = (FLEAD_OFF1 | FLEAD_OFF0)
};

enum ADS129X_CHNSET_BIT {
  PDN              = 0x80,
  PD_N             = 0x80,
  GAINN2           = 0x40,
  GAINN1           = 0x20,
  GAINN0           = 0x10,
  MUXN2            = 0x04,
  MUXN1            = 0x02,
  MUXN0            = 0x01,
                   
  CHNSET_CONST     = 0x00,
                   
  GAIN_1X          = GAINN0,
  GAIN_2X          = GAINN1,
  GAIN_3X          = (GAINN1 | GAINN0),
  GAIN_4X          = GAINN2,
  GAIN_6X          = 0x00,
  GAIN_8X          = (GAINN2 | GAINN0),
  GAIN_12X         = (GAINN2 | GAINN1),
  
  ELECTRODE_INPUT  = 0x00,
  SHORTED          = MUXN0,
  RLD_INPUT        = MUXN1,
  MVDD             = (MUXN1 | MUXN0),
  TEMP             = MUXN2,
  TEST_SIGNAL      = (MUXN2 | MUXN0),
  RLD_DRP          = (MUXN2 | MUXN1),
  RLD_DRN          = (MUXN2 | MUXN1 | MUXN0)
};

enum ADS129X_CH1SET_BIT {
  PD_1             = 0x80,
  GAIN12           = 0x40,
  GAIN11           = 0x20,
  GAIN10           = 0x10,
  MUX12            = 0x04,
  MUX11            = 0x02,
  MUX10            = 0x01,
                   
  CH1SET_CONST     = 0x00
};

enum ADS129X_CH2SET_BIT {
  PD_2             = 0x80,
  GAIN22           = 0x40,
  GAIN21           = 0x20,
  GAIN20           = 0x10,
  MUX22            = 0x04,
  MUX21            = 0x02,
  MUX20            = 0x01,
                   
  CH2SET_CONST     = 0x00
};

enum ADS129X_CH3SET_BIT {
  PD_3             = 0x80,
  GAIN32           = 0x40,
  GAIN31           = 0x20,
  GAIN30           = 0x10,
  MUX32            = 0x04,
  MUX31            = 0x02,
  MUX30            = 0x01,
                   
  CH3SET_CONST     = 0x00
};

enum ADS129X_CH4SET_BIT {
  PD_4             = 0x80,
  GAIN42           = 0x40,
  GAIN41           = 0x20,
  GAIN40           = 0x10,
  MUX42            = 0x04,
  MUX41            = 0x02,
  MUX40            = 0x01,
                   
  CH4SET_CONST     = 0x00
};

enum ADS129X_CH5SET_BIT {
  PD_5             = 0x80,
  GAIN52           = 0x40,
  GAIN51           = 0x20,
  GAIN50           = 0x10,
  MUX52            = 0x04,
  MUX51            = 0x02,
  MUX50            = 0x01,
                   
  CH5SET_CONST     = 0x00
};

enum ADS129X_CH6SET_BIT {
  PD_6             = 0x80,
  GAIN62           = 0x40,
  GAIN61           = 0x20,
  GAIN60           = 0x10,
  MUX62            = 0x04,
  MUX61            = 0x02,
  MUX60            = 0x01,
                   
  CH6SET_CONST     = 0x00
};

enum ADS129X_CH7SET_BIT {
  PD_7             = 0x80,
  GAIN72           = 0x40,
  GAIN71           = 0x20,
  GAIN70           = 0x10,
  MUX72            = 0x04,
  MUX71            = 0x02,
  MUX70            = 0x01,
                   
  CH7SET_CONST     = 0x00
};

enum ADS129X_CH8SET_BIT {
  PD_8             = 0x80,
  GAIN82           = 0x40,
  GAIN81           = 0x20,
  GAIN80           = 0x10,
  MUX82            = 0x04,
  MUX81            = 0x02,
  MUX80            = 0x01,
                   
  CH8SET_CONST     = 0x00
};

enum ADS129X_RLD_SENSP_BIT {
  RLD8P            = 0x80,
  RLD7P            = 0x40,
  RLD6P            = 0x20,
  RLD5P            = 0x10,
  RLD4P            = 0x08,
  RLD3P            = 0x04,
  RLD2P            = 0x02,
  RLD1P            = 0x01,
  
  RLD_SENSP_CONST  = 0x00
};

enum ADS129X_RLD_SENSN_BIT {
  RLD8N            = 0x80,
  RLD7N            = 0x40,
  RLD6N            = 0x20,
  RLD5N            = 0x10,
  RLD4N            = 0x08,
  RLD3N            = 0x04,
  RLD2N            = 0x02,
  RLD1N            = 0x01,
  
  RLD_SENSN_CONST  = 0x00
};

enum ADS129X_LOFF_SENSP_BIT {
  LOFF8P           = 0x80,
  LOFF7P           = 0x40,
  LOFF6P           = 0x20,
  LOFF5P           = 0x10,
  LOFF4P           = 0x08,
  LOFF3P           = 0x04,
  LOFF2P           = 0x02,
  LOFF1P           = 0x01,
  
  LOFF_SENSP_CONST = 0x00
};

enum ADS129X_LOFF_SENSN_BIT {
  LOFF8N           = 0x80,
  LOFF7N           = 0x40,
  LOFF6N           = 0x20,
  LOFF5N           = 0x10,
  LOFF4N           = 0x08,
  LOFF3N           = 0x04,
  LOFF2N           = 0x02,
  LOFF1N           = 0x01,
  
  LOFF_SENSN_CONST = 0x00
};

enum ADS129X_LOFF_FLIP_BIT {
  LOFF_FLIP8       = 0x80,
  LOFF_FLIP7       = 0x40,
  LOFF_FLIP6       = 0x20,
  LOFF_FLIP5       = 0x10,
  LOFF_FLIP4       = 0x08,
  LOFF_FLIP3       = 0x04,
  LOFF_FLIP2       = 0x02,
  LOFF_FLIP1       = 0x01,
                  
  LOFF_FLIP_CONST  = 0x00
};

enum ADS129X_LOFF_STATP_BIT {
  IN8P_OFF         = 0x80,
  IN7P_OFF         = 0x40,
  IN6P_OFF         = 0x20,
  IN5P_OFF         = 0x10,
  IN4P_OFF         = 0x08,
  IN3P_OFF         = 0x04,
  IN2P_OFF         = 0x02,
  IN1P_OFF         = 0x01,
  
  LOFF_STATP_CONST = 0x00
};

enum ADS129X_LOFF_STATN_BIT {
  IN8N_OFF         = 0x80,
  IN7N_OFF         = 0x40,
  IN6N_OFF         = 0x20,
  IN5N_OFF         = 0x10,
  IN4N_OFF         = 0x08,
  IN3N_OFF         = 0x04,
  IN2N_OFF         = 0x02,
  IN1N_OFF         = 0x01,
  
  LOFF_STATN_CONST = 0x00
};

enum ADS129X_GPIO_BIT {
  GPIOD4           = 0x80,
  GPIOD3           = 0x40,
  GPIOD2           = 0x20,
  GPIOD1           = 0x10,
  GPIOC4           = 0x08,
  GPIOC3           = 0x04,
  GPIOC2           = 0x02,
  GPIOC1           = 0x01,
  
  GPIO_CONST       = 0x0F
};

enum ADS129X_PACE_BIT {
  PACEE1           = 0x10,
  PACEE0           = 0x08,
  PACEO1           = 0x04,
  PACEO0           = 0x02,
  PD_PACE          = 0x01,
  
  PACE_CONST       = 0x00,
  
  PACEE_CHAN2      = 0x00,
  PACEE_CHAN4      = PACEE0,
  PACEE_CHAN6      = PACEE1,
  PACEE_CHAN8      = (PACEE1 | PACEE0),
                   
  PACEO_CHAN1      = 0x00,
  PACEO_CHAN3      = PACEE0,
  PACEO_CHAN5      = PACEE1,
  PACEO_CHAN7      = (PACEE1 | PACEE0)
};

enum ADS129X_RESP_BIT {
  RESP_DEMOD_EN1   = 0x80,
  RESP_MOD_EN1     = 0x40,
  RESP_PH2         = 0x10,
  RESP_PH1         = 0x08,
  RESP_PH0         = 0x04,
  RESP_CTRL1       = 0x02,
  RESP_CTRL0       = 0x01,
                   
  RESP_CONST       = 0x20,
                   
  RESP_PH_22_5     = 0x00,
  RESP_PH_45       = RESP_PH0,
  RESP_PH_67_5     = RESP_PH1,
  RESP_PH_90       = (RESP_PH1 | RESP_PH0),
  RESP_PH_112_5    = RESP_PH2,
  RESP_PH_135      = (RESP_PH2 | RESP_PH0),
  RESP_PH_157_5    = (RESP_PH2 | RESP_PH1),
                  
  RESP_NONE        = 0x00,
  RESP_EXT         = RESP_CTRL0,
  RESP_INT_SIG_INT = RESP_CTRL1,
  RESP_INT_SIG_EXT = (RESP_CTRL1 | RESP_CTRL0)
};

enum ADS129X_CONFIG4_BIT {
  RESP_FREQ2       = 0x80,
  RESP_FREQ1       = 0x40,
  RESP_FREQ0       = 0x20,
  SINGLE_SHOT      = 0x08,
  WCT_TO_RLD       = 0x04,
  PD_LOFF_COMP     = 0x02,
  
  CONFIG4_CONST    = 0x00,
  
  RESP_FREQ_64k_Hz = 0x00,
  RESP_FREQ_32k_Hz = RESP_FREQ0,
  RESP_FREQ_16k_Hz = RESP_FREQ1,
  RESP_FREQ_8k_Hz  = (RESP_FREQ1 | RESP_FREQ0),
  RESP_FREQ_4k_Hz  = RESP_FREQ2,
  RESP_FREQ_2k_Hz  = (RESP_FREQ2 | RESP_FREQ0),
  RESP_FREQ_1k_Hz  = (RESP_FREQ2 | RESP_FREQ1),
  RESP_FREQ_500_Hz = (RESP_FREQ2 | RESP_FREQ1 | RESP_FREQ0)
};

enum ADS129X_WCT1_BIT {
  AVF_CH6          = 0x80,
  AVL_CH5          = 0x40,
  AVR_CH7          = 0x20,
  AVR_CH4          = 0x10,
  PD_WCTA          = 0x08,
  WCTA2            = 0x04,
  WCTA1            = 0x02,
  WCTA0            = 0x01,
                   
  WCT1_CONST       = 0x00,
                   
  WCTA_CH1P        = 0x00,
  WCTA_CH1N        = WCTA0,
  WCTA_CH2P        = WCTA1,
  WCTA_CH2N        = (WCTA1 | WCTA0),
  WCTA_CH3P        = WCTA2,
  WCTA_CH3N        = (WCTA2 | WCTA0),
  WCTA_CH4P        = (WCTA2 | WCTA1),
  WCTA_CH4N        = (WCTA2 | WCTA1 | WCTA0)
};

enum ADS129X_WCT2_BIT {
  PD_WCTC          = 0x80,
  PD_WCTB          = 0x40,
  WCTB2            = 0x20,
  WCTB1            = 0x10,
  WCTB0            = 0x08,
  WCTC2            = 0x04,
  WCTC1            = 0x02,
  WCTC0            = 0x01,
                   
  WCT2_CONST       = 0x00,
                   
  WCTB_CH1P        = 0x00,
  WCTB_CH1N        = WCTB0,
  WCTB_CH2P        = WCTB1,
  WCTB_CH2N        = (WCTB1 | WCTB0),
  WCTB_CH3P        = WCTB2,
  WCTB_CH3N        = (WCTB2 | WCTB0),
  WCTB_CH4P        = (WCTB2 | WCTB1),
  WCTB_CH4N        = (WCTB2 | WCTB1 | WCTB0),
                   
  WCTC_CH1P        = 0x00,
  WCTC_CH1N        = WCTC0,
  WCTC_CH2P        = WCTC1,
  WCTC_CH2N        = (WCTC1 | WCTC0),
  WCTC_CH3P        = WCTC2,
  WCTC_CH3N        = (WCTC2 | WCTC0),
  WCTC_CH4P        = (WCTC2 | WCTC1),
  WCTC_CH4N        = (WCTC2 | WCTC1 | WCTC0)
};

typedef struct {

  uint8_t ID;
  /* Global Settings */
  uint8_t CONFIG1;
  uint8_t CONFIG2;
  uint8_t CONFIG3;
  uint8_t LOFF;
  /* Channel Specific Settings */
  uint8_t CH1SET;
  uint8_t CH2SET;
  uint8_t CH3SET;
  uint8_t CH4SET;
  uint8_t CH5SET;
  uint8_t CH6SET;
  uint8_t CH7SET;
  uint8_t CH8SET;
  uint8_t RLD_SENSP;
  uint8_t RLD_SENSN;
  uint8_t LOFF_SENSP;
  uint8_t LOFF_SENSN;
  uint8_t LOFF_FLIP;
  /* Lead Off Status */
  uint8_t LOFF_STATP;
  uint8_t LOFF_STATN;
  /* Other */
  uint8_t GPIO;
  uint8_t PACE;
  uint8_t RESP;
  uint8_t CONFIG4;
  uint8_t WCT1;
  uint8_t WCT2;

} register_pool_t;

typedef struct {
  uint8_t GPIO;
  uint8_t LOFF_STATN;
  uint8_t LOFF_STATP;
} status_reg_t;

typedef struct {
  uint8_t data[27];
} rdata_t;


__STATIC_FORCEINLINE void set_acquiring_mode(unsigned mode);
__STATIC_FORCEINLINE unsigned get_acquiring_mode(void);

__STATIC_INLINE void ads_read_reg(uint8_t reg_no, uint8_t count, uint8_t *data);
__STATIC_INLINE void ads_write_reg(uint8_t reg_no, uint8_t count, uint8_t *data);


#ifdef __cplusplus
}
#endif

#endif
