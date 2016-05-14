/**
  ******************************************************************************
  * @file    cSPIN.h 
  * @author  IPC Rennes
  * @version V2.1
  * @date    October 15, 2013
  * @brief   Header for cspin.c module
  * @note    (C) COPYRIGHT 2013 STMicroelectronics
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CSPIN_H
#define __CSPIN_H

/* Includes ------------------------------------------------------------------*/
#include "cspin_config.h"

/* Exported constants --------------------------------------------------------*/

/** @defgroup cSPIN FW library interface
  *
  * {
  */

#define FALSE (0)
#define TRUE  (1)

/* cSPIN maximum number of bytes of command and arguments to set a parameter */
#define cSPIN_CMD_ARG_MAX_NB_BYTES (4)
/* cSPIN command + argument bytes number */
#define cSPIN_CMD_ARG_NB_BYTES_NOP              (1)
#define cSPIN_CMD_ARG_NB_BYTES_RUN              (4)
#define cSPIN_CMD_ARG_NB_BYTES_STEP_CLOCK       (1)
#define cSPIN_CMD_ARG_NB_BYTES_MOVE             (4)
#define cSPIN_CMD_ARG_NB_BYTES_GO_TO            (4)
#define cSPIN_CMD_ARG_NB_BYTES_GO_TO_DIR        (4)
#define cSPIN_CMD_ARG_NB_BYTES_GO_UNTIL         (4)
#define cSPIN_CMD_ARG_NB_BYTES_RELEASE_SW       (1)
#define cSPIN_CMD_ARG_NB_BYTES_GO_HOME          (1)
#define cSPIN_CMD_ARG_NB_BYTES_GO_MARK          (1)
#define cSPIN_CMD_ARG_NB_BYTES_RESET_POS        (1)
#define cSPIN_CMD_ARG_NB_BYTES_RESET_DEVICE     (1)
#define cSPIN_CMD_ARG_NB_BYTES_SOFT_STOP        (1)
#define cSPIN_CMD_ARG_NB_BYTES_HARD_STOP        (1)
#define cSPIN_CMD_ARG_NB_BYTES_SOFT_HIZ         (1)
#define cSPIN_CMD_ARG_NB_BYTES_HARD_HIZ         (1)
#define cSPIN_CMD_ARG_NB_BYTES_GET_STATUS       (1)
/* cSPIN response bytes number */
#define cSPIN_RSP_NB_BYTES_GET_STATUS           (2)    
/* Daisy chain command mask */
#define DAISY_CHAIN_COMMAND_MASK (0xFA)
    

/** cSPIN parameter min and max values
  */
#define cSPIN_CONF_PARAM_STALL_TH_MV_MAX           ((uint16_t)(1000)) /* mV */

/** Register bits / masks
  */

/* cSPIN electrical position register masks */
#define cSPIN_ELPOS_STEP_MASK			((uint8_t)0xC0)
#define cSPIN_ELPOS_MICROSTEP_MASK		((uint8_t)0x3F)

/* cSPIN min speed register bit / mask */
#define cSPIN_LSPD_OPT		((uint16_t) ((0x1) << 12))
#define cSPIN_MIN_SPEED_MASK	((uint16_t)0x0FFF)

/* cSPIN full step boost bit */
#define cSPIN_BOOST_MODE        ((uint16_t) ((0x1) << 10))
#define cSPIN_FS_SPD_MASK	((uint16_t)0x03FF)

/* cSPIN Sync Output frequency enabling bit */
#define cSPIN_SYNC_EN           ((0x1) << 7)

/* cSPIN clock source monitoring enabling bit */
#define cSPIN_WD_EN     	((0x1) << 11)

/* Exported types ------------------------------------------------------------*/

//typedef u8  bool;

/** 
  * @brief cSPIN Init structure definition
  */
typedef struct
{
  uint32_t ABS_POS;
  uint16_t EL_POS;
  uint32_t MARK;
  uint32_t SPEED;
  uint16_t ACC;
  uint16_t DEC;
  uint16_t MAX_SPEED;
  uint16_t MIN_SPEED;
  uint16_t FS_SPD;
  uint8_t  KVAL_HOLD;
  uint8_t  KVAL_RUN;
  uint8_t  KVAL_ACC;
  uint8_t  KVAL_DEC;
  uint16_t INT_SPD;
  uint8_t  ST_SLP;
  uint8_t  FN_SLP_ACC;
  uint8_t  FN_SLP_DEC;
  uint8_t  K_THERM;
  uint8_t  ADC_OUT;
  uint8_t  OCD_TH;
  uint8_t  STALL_TH;
  uint8_t  STEP_MODE;
  uint8_t  ALARM_EN;
  uint16_t GATECFG1;
  uint8_t  GATECFG2;  
  uint16_t CONFIG;
  uint16_t STATUS;
}cSPIN_RegsStruct_TypeDef;

/* cSPIN Low speed optimization */
typedef enum {
	cSPIN_LSPD_OPT_OFF		=((uint16_t)0x0000),
	cSPIN_LSPD_OPT_ON		=((uint16_t)cSPIN_LSPD_OPT)
} cSPIN_LSPD_OPT_TypeDef;

/* cSPIN Full step boost */
typedef enum {
	cSPIN_BOOST_MODE_OFF		=((uint16_t)0x0000),
	cSPIN_BOOST_MODE_ON		=((uint16_t)cSPIN_BOOST_MODE)
} cSPIN_BOOST_MODE_TypeDef;

/* cSPIN overcurrent threshold options */
typedef enum {
	cSPIN_OCD_TH_31_25mV		=((uint8_t)0x00),
	cSPIN_OCD_TH_62_5mV		=((uint8_t)0x01),
	cSPIN_OCD_TH_93_75mV		=((uint8_t)0x02),
	cSPIN_OCD_TH_125mV		=((uint8_t)0x03),
	cSPIN_OCD_TH_156_25mV		=((uint8_t)0x04),
	cSPIN_OCD_TH_187_50mV		=((uint8_t)0x05),
	cSPIN_OCD_TH_218_75mV		=((uint8_t)0x06),
	cSPIN_OCD_TH_250mV		=((uint8_t)0x07),
	cSPIN_OCD_TH_281_25mV		=((uint8_t)0x08),
	cSPIN_OCD_TH_312_5mV		=((uint8_t)0x09),
	cSPIN_OCD_TH_343_75mV		=((uint8_t)0x0A),
	cSPIN_OCD_TH_375mV		=((uint8_t)0x0B),
	cSPIN_OCD_TH_406_25mV		=((uint8_t)0x0C),
	cSPIN_OCD_TH_437_5mV		=((uint8_t)0x0D),
	cSPIN_OCD_TH_468_75mV		=((uint8_t)0x0E),
	cSPIN_OCD_TH_500mV		=((uint8_t)0x0F),
        cSPIN_OCD_TH_531_25mV		=((uint8_t)0x10),
        cSPIN_OCD_TH_562_5mV		=((uint8_t)0x11),
        cSPIN_OCD_TH_593_75mV		=((uint8_t)0x12),
        cSPIN_OCD_TH_625mV		=((uint8_t)0x13),
        cSPIN_OCD_TH_656_25mV		=((uint8_t)0x14),
        cSPIN_OCD_TH_687_5mV		=((uint8_t)0x15),
        cSPIN_OCD_TH_718_75mV		=((uint8_t)0x16),
        cSPIN_OCD_TH_750mV		=((uint8_t)0x17),
        cSPIN_OCD_TH_781_25mV		=((uint8_t)0x18),
        cSPIN_OCD_TH_812_5mV		=((uint8_t)0x19),
        cSPIN_OCD_TH_843_75mV		=((uint8_t)0x1A),
        cSPIN_OCD_TH_875mV		=((uint8_t)0x1B),
        cSPIN_OCD_TH_906_25mV		=((uint8_t)0x1C),
        cSPIN_OCD_TH_937_75mV		=((uint8_t)0x1D),
        cSPIN_OCD_TH_968_75mV		=((uint8_t)0x1E),
        cSPIN_OCD_TH_1V		        =((uint8_t)0x1F)        
} cSPIN_OCD_TH_TypeDef;

/* cSPIN STEP_MODE register masks */
typedef enum {
	cSPIN_STEP_MODE_STEP_SEL		=((uint8_t)0x07),
	cSPIN_STEP_MODE_SYNC_SEL		=((uint8_t)0x70),
	cSPIN_STEP_MODE_SYNC_EN			=((uint8_t)0x80)
} cSPIN_STEP_MODE_Masks_TypeDef;

 /* cSPIN STEP_MODE register options */
/* cSPIN STEP_SEL options */
typedef enum {
	cSPIN_STEP_SEL_1		=((uint8_t)0x00),
	cSPIN_STEP_SEL_1_2		=((uint8_t)0x01),
	cSPIN_STEP_SEL_1_4		=((uint8_t)0x02),
	cSPIN_STEP_SEL_1_8		=((uint8_t)0x03),
	cSPIN_STEP_SEL_1_16		=((uint8_t)0x04),
	cSPIN_STEP_SEL_1_32		=((uint8_t)0x05),
	cSPIN_STEP_SEL_1_64		=((uint8_t)0x06),
	cSPIN_STEP_SEL_1_128	        =((uint8_t)0x07)
} cSPIN_STEP_SEL_TypeDef;

/* cSPIN SYNC_SEL options */
typedef enum {
	cSPIN_SYNC_SEL_DISABLED		=((uint8_t)0x00),
        cSPIN_SYNC_SEL_1_2		=((uint8_t)(cSPIN_SYNC_EN|0x00)),
	cSPIN_SYNC_SEL_1		=((uint8_t)(cSPIN_SYNC_EN|0x10)),
	cSPIN_SYNC_SEL_2		=((uint8_t)(cSPIN_SYNC_EN|0x20)),
	cSPIN_SYNC_SEL_4		=((uint8_t)(cSPIN_SYNC_EN|0x30)),
	cSPIN_SYNC_SEL_8		=((uint8_t)(cSPIN_SYNC_EN|0x40)),
	cSPIN_SYNC_SEL_16		=((uint8_t)(cSPIN_SYNC_EN|0x50)),
	cSPIN_SYNC_SEL_32		=((uint8_t)(cSPIN_SYNC_EN|0x60)),
	cSPIN_SYNC_SEL_64		=((uint8_t)(cSPIN_SYNC_EN|0x70))
} cSPIN_SYNC_SEL_TypeDef;

/* cSPIN ALARM_EN register options */
typedef enum {
	cSPIN_ALARM_EN_OVERCURRENT		=((uint8_t)0x01),
	cSPIN_ALARM_EN_THERMAL_SHUTDOWN		=((uint8_t)0x02),
	cSPIN_ALARM_EN_THERMAL_WARNING		=((uint8_t)0x04),
	cSPIN_ALARM_EN_UVLO     		=((uint8_t)0x08),
	cSPIN_ALARM_EN_ADC_UVLO 		=((uint8_t)0x10),
	cSPIN_ALARM_EN_STALL_DETECTION	        =((uint8_t)0x20),
	cSPIN_ALARM_EN_SW_TURN_ON		=((uint8_t)0x40),
	cSPIN_ALARM_EN_WRONG_NPERF_CMD		=((uint8_t)0x80)
} cSPIN_ALARM_EN_TypeDef;

/* cSPIN TCC (Control current Time) options*/
typedef enum {
	cSPIN_TCC_125ns		=((uint8_t)0x00),
        cSPIN_TCC_250ns		=((uint8_t)0x01),
	cSPIN_TCC_375ns		=((uint8_t)0x02),
	cSPIN_TCC_500ns		=((uint8_t)0x03),
	cSPIN_TCC_625ns		=((uint8_t)0x04),
	cSPIN_TCC_750ns		=((uint8_t)0x05),
	cSPIN_TCC_875ns		=((uint8_t)0x06),
	cSPIN_TCC_1000ns	=((uint8_t)0x07),
	cSPIN_TCC_1125ns	=((uint8_t)0x08),
	cSPIN_TCC_1250ns	=((uint8_t)0x09),
	cSPIN_TCC_1375ns	=((uint8_t)0x0A),
	cSPIN_TCC_1500ns	=((uint8_t)0x0B),
	cSPIN_TCC_1625ns	=((uint8_t)0x0C),
	cSPIN_TCC_1750ns	=((uint8_t)0x0D),
	cSPIN_TCC_1875ns	=((uint8_t)0x0E),
	cSPIN_TCC_2000ns	=((uint8_t)0x0F),
	cSPIN_TCC_2125ns	=((uint8_t)0x10),
	cSPIN_TCC_2250ns	=((uint8_t)0x11),
	cSPIN_TCC_2375ns	=((uint8_t)0x12),
	cSPIN_TCC_2500ns	=((uint8_t)0x13),
	cSPIN_TCC_2625ns	=((uint8_t)0x14),
	cSPIN_TCC_2750ns	=((uint8_t)0x15),
	cSPIN_TCC_2875ns	=((uint8_t)0x16),
	cSPIN_TCC_3000ns	=((uint8_t)0x17),
	cSPIN_TCC_3125ns	=((uint8_t)0x18),
	cSPIN_TCC_3250ns	=((uint8_t)0x19),
	cSPIN_TCC_3375ns	=((uint8_t)0x1A),
	cSPIN_TCC_3500ns	=((uint8_t)0x1B),
	cSPIN_TCC_3625ns	=((uint8_t)0x1C),
	cSPIN_TCC_3750ns	=((uint8_t)0x1D),
        cSPIN_TCC_3750ns_bis	=((uint8_t)0x1E),
        cSPIN_TCC_3750ns_ter	=((uint8_t)0x1F)
} cSPIN_TCC_TypeDef;

/* cSPIN IGate options*/
typedef enum {
	cSPIN_IGATE_4mA		        =(((uint8_t)0x00)<<5),
        cSPIN_IGATE_4mA_Bis		=(((uint8_t)0x01)<<5),
	cSPIN_IGATE_8mA		        =(((uint8_t)0x02)<<5),
	cSPIN_IGATE_16mA		=(((uint8_t)0x03)<<5),
	cSPIN_IGATE_24mA		=(((uint8_t)0x04)<<5),
	cSPIN_IGATE_32mA		=(((uint8_t)0x05)<<5),
	cSPIN_IGATE_64mA		=(((uint8_t)0x06)<<5),
	cSPIN_IGATE_96mA		=(((uint8_t)0x07)<<5),
} cSPIN_IGATE_TypeDef;

/* cSPIN TBOOST (Turn off boost time) options*/
typedef enum {
	cSPIN_TBOOST_0ns		=(((uint8_t)0x00)<<8),
        cSPIN_TBOOST_62_5__83_3__125ns	=(((uint8_t)0x01)<<8),
	cSPIN_TBOOST_125ns		=(((uint8_t)0x02)<<8),
	cSPIN_TBOOST_250ns		=(((uint8_t)0x03)<<8),
	cSPIN_TBOOST_375ns		=(((uint8_t)0x04)<<8),
	cSPIN_TBOOST_500ns		=(((uint8_t)0x05)<<8),
	cSPIN_TBOOST_750ns		=(((uint8_t)0x06)<<8),
	cSPIN_TBOOST_1000ns		=(((uint8_t)0x07)<<8),
} cSPIN_TBOOST_TypeDef;

/* cSPIN external clock watchdog */
typedef enum {
	cSPIN_WD_EN_DISABLE		=((uint16_t)0x0000),
	cSPIN_WD_EN_ENABLE		=((uint16_t)cSPIN_WD_EN)
} cSPIN_WD_EN_TypeDef;

/* cSPIN TBLANK (Blanking time) options*/
typedef enum {
	cSPIN_TBLANK_125ns		=(((uint8_t)0x00)<<5),
        cSPIN_TBLANK_250ns      	=(((uint8_t)0x01)<<5),
	cSPIN_TBLANK_375ns		=(((uint8_t)0x02)<<5),
	cSPIN_TBLANK_500ns		=(((uint8_t)0x03)<<5),
	cSPIN_TBLANK_625ns		=(((uint8_t)0x04)<<5),
	cSPIN_TBLANK_750ns		=(((uint8_t)0x05)<<5),
	cSPIN_TBLANK_875ns		=(((uint8_t)0x06)<<5),
	cSPIN_TBLANK_1000ns		=(((uint8_t)0x07)<<5),
} cSPIN_TBLANK_TypeDef;

/* cSPIN TDT (Dead time) options*/
typedef enum {
	cSPIN_TDT_125ns		=((uint8_t)0x00),
        cSPIN_TDT_250ns		=((uint8_t)0x01),
	cSPIN_TDT_375ns		=((uint8_t)0x02),
	cSPIN_TDT_500ns		=((uint8_t)0x03),
	cSPIN_TDT_625ns		=((uint8_t)0x04),
	cSPIN_TDT_750ns		=((uint8_t)0x05),
	cSPIN_TDT_875ns		=((uint8_t)0x06),
	cSPIN_TDT_1000ns	=((uint8_t)0x07),
	cSPIN_TDT_1125ns	=((uint8_t)0x08),
	cSPIN_TDT_1250ns	=((uint8_t)0x09),
	cSPIN_TDT_1375ns	=((uint8_t)0x0A),
	cSPIN_TDT_1500ns	=((uint8_t)0x0B),
	cSPIN_TDT_1625ns	=((uint8_t)0x0C),
	cSPIN_TDT_1750ns	=((uint8_t)0x0D),
	cSPIN_TDT_1875ns	=((uint8_t)0x0E),
	cSPIN_TDT_2000ns	=((uint8_t)0x0F),
	cSPIN_TDT_2125ns	=((uint8_t)0x10),
	cSPIN_TDT_2250ns	=((uint8_t)0x11),
	cSPIN_TDT_2375ns	=((uint8_t)0x12),
	cSPIN_TDT_2500ns	=((uint8_t)0x13),
	cSPIN_TDT_2625ns	=((uint8_t)0x14),
	cSPIN_TDT_2750ns	=((uint8_t)0x15),
	cSPIN_TDT_2875ns	=((uint8_t)0x16),
	cSPIN_TDT_3000ns	=((uint8_t)0x17),
	cSPIN_TDT_3125ns	=((uint8_t)0x18),
	cSPIN_TDT_3250ns	=((uint8_t)0x19),
	cSPIN_TDT_3375ns	=((uint8_t)0x1A),
	cSPIN_TDT_3500ns	=((uint8_t)0x1B),
	cSPIN_TDT_3625ns	=((uint8_t)0x1C),
	cSPIN_TDT_3750ns	=((uint8_t)0x1D),
        cSPIN_TDT_3875ns        =((uint8_t)0x1E),
        cSPIN_TDT_4000ns        =((uint8_t)0x1F)
} cSPIN_TDT_TypeDef;


/* cSPIN Config register masks */
typedef enum {
	cSPIN_CONFIG_OSC_SEL		=((uint16_t)0x0007),
	cSPIN_CONFIG_EXT_CLK		=((uint16_t)0x0008),
	cSPIN_CONFIG_SW_MODE		=((uint16_t)0x0010),
	cSPIN_CONFIG_EN_VSCOMP		=((uint16_t)0x0020),
	cSPIN_CONFIG_OC_SD		=((uint16_t)0x0080),
	cSPIN_CONFIG_UVLOVAL		=((uint16_t)0x0100),
	cSPIN_CONFIG_VCCVAL		=((uint16_t)0x0200),
	cSPIN_CONFIG_F_PWM_DEC		=((uint16_t)0x1C00),
	cSPIN_CONFIG_F_PWM_INT		=((uint16_t)0xE000)
} cSPIN_CONFIG_Masks_TypeDef;

/* cSPIN Config register options */
typedef enum {
	cSPIN_CONFIG_INT_16MHZ			=((uint16_t)0x0000),
	cSPIN_CONFIG_INT_16MHZ_OSCOUT_2MHZ	=((uint16_t)0x0008),
	cSPIN_CONFIG_INT_16MHZ_OSCOUT_4MHZ	=((uint16_t)0x0009),
	cSPIN_CONFIG_INT_16MHZ_OSCOUT_8MHZ	=((uint16_t)0x000A),
	cSPIN_CONFIG_INT_16MHZ_OSCOUT_16MHZ	=((uint16_t)0x000B),
	cSPIN_CONFIG_EXT_8MHZ_XTAL_DRIVE	=((uint16_t)0x0004),
	cSPIN_CONFIG_EXT_16MHZ_XTAL_DRIVE	=((uint16_t)0x0005),
	cSPIN_CONFIG_EXT_24MHZ_XTAL_DRIVE	=((uint16_t)0x0006),
	cSPIN_CONFIG_EXT_32MHZ_XTAL_DRIVE	=((uint16_t)0x0007),
	cSPIN_CONFIG_EXT_8MHZ_OSCOUT_INVERT	=((uint16_t)0x000C),
	cSPIN_CONFIG_EXT_16MHZ_OSCOUT_INVERT	=((uint16_t)0x000D),
	cSPIN_CONFIG_EXT_24MHZ_OSCOUT_INVERT	=((uint16_t)0x000E),
	cSPIN_CONFIG_EXT_32MHZ_OSCOUT_INVERT	=((uint16_t)0x000F)
} cSPIN_CONFIG_OSC_MGMT_TypeDef;

typedef enum {
	cSPIN_CONFIG_SW_HARD_STOP		=((uint16_t)0x0000),
	cSPIN_CONFIG_SW_USER			=((uint16_t)0x0010)
} cSPIN_CONFIG_SW_MODE_TypeDef;

typedef enum {
	cSPIN_CONFIG_VS_COMP_DISABLE	=((uint16_t)0x0000),
	cSPIN_CONFIG_VS_COMP_ENABLE	=((uint16_t)0x0020)
} cSPIN_CONFIG_EN_VSCOMP_TypeDef;

typedef enum {
	cSPIN_CONFIG_OC_SD_DISABLE		=((uint16_t)0x0000),
	cSPIN_CONFIG_OC_SD_ENABLE		=((uint16_t)0x0080)
} cSPIN_CONFIG_OC_SD_TypeDef;

typedef enum {
	cSPIN_CONFIG_UVLOVAL_LOW        =((uint16_t)0x0000),
        cSPIN_CONFIG_UVLOVAL_HIGH	=((uint16_t)0x0100),
} cSPIN_CONFIG_UVLOVAL_TypeDef;

typedef enum {
	cSPIN_CONFIG_VCCVAL_7_5V        =((uint16_t)0x0200),
	cSPIN_CONFIG_VCCVAL_15V         =((uint16_t)0x0300)
} cSPIN_CONFIG_VCCVAL_TypeDef;

typedef enum {
	cSPIN_CONFIG_PWM_DIV_1		=(((uint16_t)0x00)<<13),
	cSPIN_CONFIG_PWM_DIV_2		=(((uint16_t)0x01)<<13),
	cSPIN_CONFIG_PWM_DIV_3		=(((uint16_t)0x02)<<13),
	cSPIN_CONFIG_PWM_DIV_4		=(((uint16_t)0x03)<<13),
	cSPIN_CONFIG_PWM_DIV_5		=(((uint16_t)0x04)<<13),
	cSPIN_CONFIG_PWM_DIV_6		=(((uint16_t)0x05)<<13),
	cSPIN_CONFIG_PWM_DIV_7		=(((uint16_t)0x06)<<13)
} cSPIN_CONFIG_F_PWM_INT_TypeDef;

typedef enum {
	cSPIN_CONFIG_PWM_MUL_0_625		=(((uint16_t)0x00)<<10),
	cSPIN_CONFIG_PWM_MUL_0_75		=(((uint16_t)0x01)<<10),
	cSPIN_CONFIG_PWM_MUL_0_875		=(((uint16_t)0x02)<<10),
	cSPIN_CONFIG_PWM_MUL_1			=(((uint16_t)0x03)<<10),
	cSPIN_CONFIG_PWM_MUL_1_25		=(((uint16_t)0x04)<<10),
	cSPIN_CONFIG_PWM_MUL_1_5		=(((uint16_t)0x05)<<10),
	cSPIN_CONFIG_PWM_MUL_1_75		=(((uint16_t)0x06)<<10),
	cSPIN_CONFIG_PWM_MUL_2			=(((uint16_t)0x07)<<10)
} cSPIN_CONFIG_F_PWM_DEC_TypeDef;

/* Status Register bit masks */
typedef enum {
	cSPIN_STATUS_HIZ		=(((uint16_t)0x0001)),
	cSPIN_STATUS_BUSY		=(((uint16_t)0x0002)),
	cSPIN_STATUS_SW_F		=(((uint16_t)0x0004)),
	cSPIN_STATUS_SW_EVN		=(((uint16_t)0x0008)),
	cSPIN_STATUS_DIR		=(((uint16_t)0x0010)),
	cSPIN_STATUS_MOT_STATUS		=(((uint16_t)0x0060)),
	cSPIN_STATUS_NOTPERF_CMD	=(((uint16_t)0x0080)),
	cSPIN_STATUS_STCK_MOD		=(((uint16_t)0x0100)),
	cSPIN_STATUS_UVLO		=(((uint16_t)0x0200)),
	cSPIN_STATUS_UVLO_ADC	        =(((uint16_t)0x0400)),
	cSPIN_STATUS_TH_SD		=(((uint16_t)0x1800)),
	cSPIN_STATUS_OCD		=(((uint16_t)0x2000)),
	cSPIN_STATUS_STEP_LOSS_A	=(((uint16_t)0x4000)),
	cSPIN_STATUS_STEP_LOSS_B	=(((uint16_t)0x8000))
} cSPIN_STATUS_Masks_TypeDef;

/* Status Register options */
typedef enum {
	cSPIN_STATUS_MOT_STATUS_STOPPED		=(((uint16_t)0x0000)<<5),
	cSPIN_STATUS_MOT_STATUS_ACCELERATION	=(((uint16_t)0x0001)<<5),
	cSPIN_STATUS_MOT_STATUS_DECELERATION	=(((uint16_t)0x0002)<<5),
	cSPIN_STATUS_MOT_STATUS_CONST_SPD	=(((uint16_t)0x0003)<<5)
} cSPIN_STATUS_TypeDef;

/* cSPIN internal register addresses */
typedef enum {
	cSPIN_ABS_POS			=((uint8_t)0x01),
	cSPIN_EL_POS			=((uint8_t)0x02),
	cSPIN_MARK			=((uint8_t)0x03),	
	cSPIN_SPEED			=((uint8_t)0x04),
	cSPIN_ACC			=((uint8_t)0x05),
	cSPIN_DEC			=((uint8_t)0x06),
	cSPIN_MAX_SPEED			=((uint8_t)0x07),
	cSPIN_MIN_SPEED			=((uint8_t)0x08),
	cSPIN_FS_SPD			=((uint8_t)0x15),
	cSPIN_KVAL_HOLD			=((uint8_t)0x09),
	cSPIN_KVAL_RUN			=((uint8_t)0x0A),
	cSPIN_KVAL_ACC			=((uint8_t)0x0B),
	cSPIN_KVAL_DEC			=((uint8_t)0x0C),
	cSPIN_INT_SPD			=((uint8_t)0x0D),
	cSPIN_ST_SLP			=((uint8_t)0x0E),
	cSPIN_FN_SLP_ACC		=((uint8_t)0x0F),
	cSPIN_FN_SLP_DEC		=((uint8_t)0x10),
	cSPIN_K_THERM			=((uint8_t)0x11),
	cSPIN_ADC_OUT			=((uint8_t)0x12),
	cSPIN_OCD_TH			=((uint8_t)0x13),
	cSPIN_STALL_TH			=((uint8_t)0x14),
	cSPIN_STEP_MODE			=((uint8_t)0x16),
	cSPIN_ALARM_EN			=((uint8_t)0x17),
	cSPIN_GATECFG1			=((uint8_t)0x18),
	cSPIN_GATECFG2			=((uint8_t)0x19),
	cSPIN_CONFIG			=((uint8_t)0x1A),
	cSPIN_STATUS			=((uint8_t)0x1B)
} cSPIN_Registers_TypeDef;

/* cSPIN command set */
typedef enum {
	cSPIN_NOP		=((uint8_t)0x00),
	cSPIN_SET_PARAM		=((uint8_t)0x00),
	cSPIN_GET_PARAM		=((uint8_t)0x20),
	cSPIN_RUN		=((uint8_t)0x50),
	cSPIN_STEP_CLOCK	=((uint8_t)0x58),
	cSPIN_MOVE		=((uint8_t)0x40),
	cSPIN_GO_TO		=((uint8_t)0x60),
	cSPIN_GO_TO_DIR		=((uint8_t)0x68),
	cSPIN_GO_UNTIL		=((uint8_t)0x82),
        cSPIN_GO_UNTIL_ACT_CPY  =((uint8_t)0x8A),
	cSPIN_RELEASE_SW	=((uint8_t)0x92),
	cSPIN_GO_HOME		=((uint8_t)0x70),
	cSPIN_GO_MARK		=((uint8_t)0x78),
	cSPIN_RESET_POS		=((uint8_t)0xD8),
	cSPIN_RESET_DEVICE	=((uint8_t)0xC0),
	cSPIN_SOFT_STOP		=((uint8_t)0xB0),
	cSPIN_HARD_STOP		=((uint8_t)0xB8),
	cSPIN_SOFT_HIZ		=((uint8_t)0xA0),
	cSPIN_HARD_HIZ		=((uint8_t)0xA8),
	cSPIN_GET_STATUS	=((uint8_t)0xD0),
	cSPIN_RESERVED_CMD1	=((uint8_t)0xEB),
	cSPIN_RESERVED_CMD2	=((uint8_t)0xF8)
} cSPIN_Commands_TypeDef;

/* cSPIN direction options */
typedef enum {
	FWD		=((uint8_t)0x01),
	REV		=((uint8_t)0x00)
} cSPIN_Direction_TypeDef;

/* cSPIN action options */
typedef enum {
	ACTION_RESET	=((uint8_t)0x00),
	ACTION_COPY	=((uint8_t)0x08)
} cSPIN_Action_TypeDef;
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
#define Speed_Steps_to_Par(steps) ((uint32_t)(((steps)*67.108864)+0.5))			/* Speed conversion, range 0 to 15625 steps/s */
#define AccDec_Steps_to_Par(steps) ((uint16_t)(((steps)*0.068719476736)+0.5))	/* Acc/Dec rates conversion, range 14.55 to 59590 steps/s2 */
#define MaxSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*0.065536)+0.5))			/* Max Speed conversion, range 15.25 to 15610 steps/s */
#define MinSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*4.194304)+0.5))			/* Min Speed conversion, range 0 to 976.3 steps/s */
#define FSSpd_Steps_to_Par(steps) ((uint16_t)((steps)*0.065536))				/* Full Step Speed conversion, range 7.63 to 15625 steps/s */
#define IntSpd_Steps_to_Par(steps) ((uint16_t)(((steps)*4.194304)+0.5))			/* Intersect Speed conversion, range 0 to 3906 steps/s */
#define Kval_Perc_to_Par(perc) ((uint8_t)(((perc)/0.390625)+0.5))				/* KVAL conversions, range 0.4% to 99.6% */
#define BEMF_Slope_Perc_to_Par(perc) ((uint8_t)(((perc)/0.00156862745098)+0.5))	/* BEMF compensation slopes, range 0 to 0.4% s/step */
#define KTherm_to_Par(KTherm) ((uint8_t)(((KTherm - 1)/0.03125)+0.5))			/* K_THERM compensation conversion, range 1 to 1.46875 */
#define StallTh_to_Par(StallTh) ((uint8_t)(((StallTh - 31.25)/31.25)+0.5))		/* Stall Threshold conversion, range 31.25mV to 1000mV */
#if defined(DEBUG)
#define CHECK_REG(reg, result) if (cSPIN_RegsStruct-> ## reg != cSPIN_Get_Param(cSPIN_ ## reg)) result |= 1 << ((uint8_t) cSPIN_ ## reg)
#define PRINT_REG(reg_name, read_reg, diff, reg_name_string)  strcpy(reg_name_string,"          "); \
  strncpy(reg_name_string, #reg_name, strlen(#reg_name)); \
  strcpy(diff," "); \
  read_reg = cSPIN_Get_Param(cSPIN_ ## reg_name); \
  if (read_reg != cSPIN_RegsStruct-> ## reg_name) strcpy(diff,"x"); \
  UARTprintf("%s %s mcu: 0x%-6X, cspin: 0x%-6X\n", diff, reg_name_string, cSPIN_RegsStruct-> ## reg_name, read_reg)
#endif /* defined(DEBUG) */
/* Exported functions ------------------------------------------------------- */

void cSPIN_Delay(volatile uint32_t nCount);
void cSPIN_Reset_And_Standby(void);
void cSPIN_Gpio_Toggle(uint32_t GPIOx, uint16_t GPIO_Pin);
void cSPIN_Led_Check(void);
void cSPIN_Peripherals_Init(void);
void cSPIN_PWM_Enable(uint16_t Period);
void cSPIN_PWM_DISABLE(void); 
void cSPIN_Interrupt_Channel_Config(void);
void cSPIN_Flag_Interrupt_GPIO_Config(void);
void cSPIN_Busy_Interrupt_GPIO_Config(void);
void cSPIN_Busy_Interrupt_GPIO_DeConfig(void);
void cSPIN_Buttons_Interrupts_GPIO_Config(void);
void cSPIN_Switch_Motor_Interrupt_Config(void);
void cSPIN_Regs_Struct_Reset(cSPIN_RegsStruct_TypeDef* cSPIN_RegsStruct);
void cSPIN_Registers_Set(cSPIN_RegsStruct_TypeDef* cSPIN_RegsStruct);
#if defined(DEBUG) 
uint32_t cSPIN_Registers_Check(cSPIN_RegsStruct_TypeDef* cSPIN_RegsStruct);
void cSPIN_Registers_Get(cSPIN_RegsStruct_TypeDef* cSPIN_RegsStruct);
#endif /* defined(DEBUG) */
void cSPIN_Nop(void);
void cSPIN_Set_Param(cSPIN_Registers_TypeDef param, uint32_t value);
uint32_t cSPIN_Get_Param(cSPIN_Registers_TypeDef param);
void cSPIN_Run(cSPIN_Direction_TypeDef direction, uint32_t speed);
void cSPIN_Step_Clock(cSPIN_Direction_TypeDef direction);
void cSPIN_Move(cSPIN_Direction_TypeDef direction, uint32_t n_step);
void cSPIN_Go_To(uint32_t abs_pos);
void cSPIN_Go_To_Dir(cSPIN_Direction_TypeDef direction, uint32_t abs_pos);
void cSPIN_Go_Until(cSPIN_Action_TypeDef action, cSPIN_Direction_TypeDef direction, uint32_t speed);
void cSPIN_Release_SW(cSPIN_Action_TypeDef action, cSPIN_Direction_TypeDef direction);
void cSPIN_Go_Home(void);
void cSPIN_Go_Mark(void);
void cSPIN_Reset_Pos(void);
void cSPIN_Reset_Device(void);
void cSPIN_Soft_Stop(void);
void cSPIN_Hard_Stop(void);
void cSPIN_Soft_HiZ(void);
void cSPIN_Hard_HiZ(void);
uint16_t cSPIN_Get_Status(void);
uint8_t cSPIN_Busy_HW(void);
uint8_t cSPIN_Busy_SW(void);
uint8_t cSPIN_Flag(void);
uint8_t cSPIN_Write_Byte(uint8_t byte);
void cSPIN_Write_Daisy_Chain_Bytes(uint8_t *pTxByte, uint8_t *pRxByte, uint8_t nBytes);
void cSPIN_All_Slaves_Registers_Set(uint8_t slaves_number, cSPIN_RegsStruct_TypeDef *cSPIN_RegsStructArray);
void cSPIN_All_Slaves_Set_Param(uint8_t slaves_number, uint8_t *pParam, uint32_t *pValue);
void cSPIN_All_Slaves_Get_Param(uint8_t slaves_number, uint8_t *pParam, uint32_t *pValue);
void cSPIN_One_Slave_Move(uint8_t slaveNumber, uint8_t slaves_number, cSPIN_Direction_TypeDef direction, uint32_t n_step);
void cSPIN_One_Slave_Run(uint8_t slaveNumber, uint8_t slaves_number, cSPIN_Direction_TypeDef direction, uint32_t speed);
void cSPIN_One_Slave_Send_Command(uint8_t slaveNumber, uint8_t slaves_number, uint8_t param, uint32_t value);
void cSPIN_All_Slaves_Send_Command(uint8_t slaves_number, uint8_t *pParam, uint32_t *pValue);
void cSPIN_All_Slaves_Get_Status(uint8_t slaves_number, uint32_t *pValue);
uint8_t cSPIN_One_Or_More_Slaves_Busy_SW(uint8_t slaves_number);
/** @} */  

#endif /* __CSPIN_H */

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
