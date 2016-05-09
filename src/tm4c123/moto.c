//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2012-2015 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.2.111 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "cspin.h"
#include "l6480.h"
cSPIN_RegsStruct_TypeDef cSPIN_RegsStructArray[NUMBER_OF_SLAVES];
double   MAX_SPEED[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_MAX_SPEED;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DELAY_COUNT    0x3FFFF

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t i;        
uint8_t commandArray[NUMBER_OF_SLAVES];
uint32_t argumentArray[NUMBER_OF_SLAVES];
uint32_t responseArray[NUMBER_OF_SLAVES];
/* straight forward definitions */
double   ACC[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_ACC;
double   DEC[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_DEC;
double   FS_SPD[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_FS_SPD;
uint8_t  ALARM_EN[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_ALARM_EN;
uint8_t  OCD_TH[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_OCD_TH;
/* OR-ed definitions */
uint8_t  STEP_MODE[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_STEP_MODE;
uint8_t  SYNC_MODE[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_SYNC_MODE;
uint16_t GATECFG1_WD_EN[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_WD_EN;
uint16_t GATECFG1_TBOOST[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_TBOOST;
uint16_t GATECFG1_IGATE[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_IGATE;
uint16_t GATECFG1_TCC[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_TCC;
uint8_t  GATECFG2_TBLANK[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_TBLANK;
uint8_t  GATECFG2_TDT[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_TDT;
uint16_t CONFIG_CLOCK_SETTING[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_CLOCK_SETTING;
uint16_t CONFIG_SW_MODE[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_SW_MODE;   
uint16_t CONFIG_OC_SD[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_OC_SD;
uint16_t CONFIG_UVLOVAL[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_UVLOVAL;
uint16_t CONFIG_VCCVAL[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_VCCVAL;
/* straight forward definitions */
double   KVAL_HOLD[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_KVAL_HOLD;
double   KVAL_RUN[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_KVAL_RUN;
double   KVAL_ACC[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_KVAL_ACC;
double   KVAL_DEC[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_KVAL_DEC;
double   INT_SPD[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_INT_SPD;
double   ST_SLP[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_ST_SLP;
double   FN_SLP_ACC[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_FN_SLP_ACC;
double   FN_SLP_DEC[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_FN_SLP_DEC;
double   K_THERM[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_K_THERM;
double   STALL_TH[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_STALL_TH;
/* OR-ed definitions */
double   MIN_SPEED[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_MIN_SPEED;
uint16_t LSPD_BIT[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_LSPD_BIT;
uint16_t CONFIG_VS_COMP[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_VS_COMP;
uint16_t CONFIG_PWM_DIV[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_PWM_DIV;
uint16_t CONFIG_PWM_MUL[NUMBER_OF_SLAVES] = cSPIN_DC_CONF_PARAM_PWM_MUL;
bool pOpenA[1];
bool pOpenB[1];
uint32_t cSPIN_rx_data = 0;
cSPIN_RegsStruct_TypeDef cSPIN_RegsStruct;

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE,
                                   ROM_UARTCharGetNonBlocking(UART0_BASE));

        //
        // Blink the LED to show a character transfer is occuring.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
int
main(void)
{
	initUart();
	InitDelay();
	cSPIN_Peripherals_Init();
	cSPIN_Reset_And_Standby();
	/* Structure initialization by default values, in order to avoid blank records */
	cSPIN_Regs_Struct_Reset(&cSPIN_RegsStruct);
	cSPIN_PWM_Enable(2000);
    cSPIN_Delay(0x00FFFFFF);
    cSPIN_PWM_DISABLE();
	/* Acceleration rate settings to cSPIN_CONF_PARAM_ACC in steps/s2, range 14.55 to 59590 steps/s2 */
	cSPIN_RegsStruct.ACC 		= AccDec_Steps_to_Par(cSPIN_CONF_PARAM_ACC);
	/* Deceleration rate settings to cSPIN_CONF_PARAM_DEC in steps/s2, range 14.55 to 59590 steps/s2 */
	cSPIN_RegsStruct.DEC 		= AccDec_Steps_to_Par(cSPIN_CONF_PARAM_DEC); 
	/* Maximum speed settings to cSPIN_CONF_PARAM_MAX_SPEED in steps/s, range 15.25 to 15610 steps/s */
	cSPIN_RegsStruct.MAX_SPEED 	= MaxSpd_Steps_to_Par(cSPIN_CONF_PARAM_MAX_SPEED);
	/* Full step speed settings cSPIN_CONF_PARAM_FS_SPD in steps/s, range 7.63 to 15625 steps/s */
	cSPIN_RegsStruct.FS_SPD 	= FSSpd_Steps_to_Par(cSPIN_CONF_PARAM_FS_SPD);
	/* Minimum speed settings to cSPIN_CONF_PARAM_MIN_SPEED in steps/s, range 0 to 976.3 steps/s */
	cSPIN_RegsStruct.MIN_SPEED	= cSPIN_CONF_PARAM_LSPD_BIT|MinSpd_Steps_to_Par(cSPIN_CONF_PARAM_MIN_SPEED);
        /* Acceleration duty cycle (torque) settings to cSPIN_CONF_PARAM_KVAL_ACC in %, range 0 to 99.6% */
	cSPIN_RegsStruct.KVAL_ACC 	= Kval_Perc_to_Par(cSPIN_CONF_PARAM_KVAL_ACC);
        /* Deceleration duty cycle (torque) settings to cSPIN_CONF_PARAM_KVAL_DEC in %, range 0 to 99.6% */
	cSPIN_RegsStruct.KVAL_DEC 	= Kval_Perc_to_Par(cSPIN_CONF_PARAM_KVAL_DEC);		
        /* Run duty cycle (torque) settings to cSPIN_CONF_PARAM_KVAL_RUN in %, range 0 to 99.6% */
	cSPIN_RegsStruct.KVAL_RUN 	= Kval_Perc_to_Par(cSPIN_CONF_PARAM_KVAL_RUN);
        /* Hold duty cycle (torque) settings to cSPIN_CONF_PARAM_KVAL_HOLD in %, range 0 to 99.6% */
	cSPIN_RegsStruct.KVAL_HOLD 	= Kval_Perc_to_Par(cSPIN_CONF_PARAM_KVAL_HOLD);
	        /* Thermal compensation param settings to cSPIN_CONF_PARAM_K_THERM, range 1 to 1.46875 */
	cSPIN_RegsStruct.K_THERM 	= KTherm_to_Par(cSPIN_CONF_PARAM_K_THERM);
	/* Intersect speed settings for BEMF compensation to cSPIN_CONF_PARAM_INT_SPD in steps/s, range 0 to 3906 steps/s */
	cSPIN_RegsStruct.INT_SPD 	= IntSpd_Steps_to_Par(cSPIN_CONF_PARAM_INT_SPD);
	/* BEMF start slope settings for BEMF compensation to cSPIN_CONF_PARAM_ST_SLP in % step/s, range 0 to 0.4% s/step */
	cSPIN_RegsStruct.ST_SLP 	= BEMF_Slope_Perc_to_Par(cSPIN_CONF_PARAM_ST_SLP);
	/* BEMF final acc slope settings for BEMF compensation to cSPIN_CONF_PARAM_FN_SLP_ACC in% step/s, range 0 to 0.4% s/step */
	cSPIN_RegsStruct.FN_SLP_ACC = BEMF_Slope_Perc_to_Par(cSPIN_CONF_PARAM_FN_SLP_ACC);
	/* BEMF final dec slope settings for BEMF compensation to cSPIN_CONF_PARAM_FN_SLP_DEC in% step/s, range 0 to 0.4% s/step */
	cSPIN_RegsStruct.FN_SLP_DEC = BEMF_Slope_Perc_to_Par(cSPIN_CONF_PARAM_FN_SLP_DEC);
	/* Stall threshold settings to cSPIN_CONF_PARAM_STALL_TH in mV, range 31.25 to 1000mV */
	cSPIN_RegsStruct.STALL_TH 	= StallTh_to_Par(cSPIN_CONF_PARAM_STALL_TH);
        /* Set Config register according to config parameters */
        /* clock setting, switch hard stop interrupt mode, */
        /*  supply voltage compensation, overcurrent shutdown */
        /* UVLO threshold, VCC reg output voltage , PWM frequency */
	cSPIN_RegsStruct.CONFIG 	= (uint16_t)cSPIN_CONF_PARAM_CLOCK_SETTING | \
                                          (uint16_t)cSPIN_CONF_PARAM_SW_MODE	   | \
                                          (uint16_t)cSPIN_CONF_PARAM_VS_COMP       | \
                                          (uint16_t)cSPIN_CONF_PARAM_OC_SD         | \
                                          (uint16_t)cSPIN_CONF_PARAM_UVLOVAL       | \
                                          (uint16_t)cSPIN_CONF_PARAM_VCCVAL	   | \
                                          (uint16_t)cSPIN_CONF_PARAM_PWM_DIV       | \
                                          (uint16_t)cSPIN_CONF_PARAM_PWM_MUL;

	/* Overcurrent threshold settings to cSPIN_CONF_PARAM_OCD_TH, range 31.25 to 1000mV */
	cSPIN_RegsStruct.OCD_TH 	= cSPIN_CONF_PARAM_OCD_TH;        
        /* Alarm settings to cSPIN_CONF_PARAM_ALARM_EN */
	cSPIN_RegsStruct.ALARM_EN 	= cSPIN_CONF_PARAM_ALARM_EN;
        /* Step mode and sycn mode settings via cSPIN_CONF_PARAM_SYNC_MODE and cSPIN_CONF_PARAM_STEP_MODE */
	cSPIN_RegsStruct.STEP_MODE 	= (uint8_t)cSPIN_CONF_PARAM_SYNC_MODE | \
                                          (uint8_t)cSPIN_CONF_PARAM_STEP_MODE;
    /* Sink/source current, duration of constant current phases, duration of overboost phase settings */
    cSPIN_RegsStruct.GATECFG1       = (uint16_t)cSPIN_CONF_PARAM_IGATE | \
                                      (uint16_t)cSPIN_CONF_PARAM_TCC   | \
                                      (uint16_t)cSPIN_CONF_PARAM_TBOOST;
    /* Blank time, Dead time stiings */
     cSPIN_RegsStruct.GATECFG2       = (uint16_t)cSPIN_CONF_PARAM_TBLANK | \
                                       (uint16_t)cSPIN_CONF_PARAM_TDT;
        /* Program all cSPIN registers */
	cSPIN_Registers_Set(&cSPIN_RegsStruct);

#if defined(DEBUG)
    /* check the values of all cSPIN registers */
    cSPIN_rx_data = cSPIN_Registers_Check(&cSPIN_RegsStruct);
    
    /* get the values of all cSPIN registers and print them to the terminal I/O */
    cSPIN_Registers_Get(&cSPIN_RegsStruct);
#endif /* defined(DEBUG) */             
    
    /**********************************************************************/
    /* Start example of FLAG interrupt management */
    /**********************************************************************/
	UARTprintf("Start example of FLAG interrupt management\n");
    /* Clear Flag pin */
    cSPIN_rx_data = cSPIN_Get_Status();       
    /* Interrupt configuration for FLAG signal */
    //cSPIN_Flag_Interrupt_GPIO_Config();
    UARTprintf("Run constant speed of 400 steps/s forward direction\n");
    /* Run constant speed of 400 steps/s forward direction */
    cSPIN_Run(FWD, Speed_Steps_to_Par(400));
	UARTprintf("Tentative to write to the current motor absolute position register\n");
    /* Tentative to write to the current motor absolute position register */
    /* while the motor is running */
    cSPIN_Set_Param(cSPIN_ABS_POS, 100);
    cSPIN_Delay(0x004FFFFF);
	UARTprintf("Get Status to clear FLAG due to non-performable command\n");
    /* Get Status to clear FLAG due to non-performable command */
    cSPIN_rx_data = cSPIN_Get_Status();
    cSPIN_Delay(0x004FFFFF);        
	UARTprintf("Perform SoftStop commmand\n");
    /* Perform SoftStop commmand */
	cSPIN_Soft_Stop();
	UARTprintf("Wait until not busy - busy pin test\n");
    /* Wait until not busy - busy pin test */
	while(cSPIN_Busy_HW());
    cSPIN_Delay(0x004FFFFF);
    /**********************************************************************/
    /* End example of FLAG interrupt management */
    /**********************************************************************/
	UARTprintf("End example of FLAG interrupt management\n");
	UARTprintf("Start example of BUSY interrupt management\n");

    /**********************************************************************/
    /* Start example of BUSY interrupt management */
    /**********************************************************************/
    /* Interrupt configuration for BUSY signal */
    //cSPIN_Busy_Interrupt_GPIO_Config();
    UARTprintf("Move by 100,000 steps reverse, range 0 to 4,194,303\n");
	/* Move by 100,000 steps reverse, range 0 to 4,194,303 */
	cSPIN_Move(REV, (uint32_t)(100000));
    /* STEVAL_PCC009V2 : during busy time the POWER LED is switched OFF */
    /* ST_cSPIN_6480H_DISCOVERY : during busy time the LED_BUSY is switched ON */
    /* Wait until not busy - busy pin test */
	UARTprintf("during busy time the LED_BUSY is switched ON\n");
	while(cSPIN_Busy_HW());
    /* Disable the power bridges */
	cSPIN_Soft_HiZ();
    cSPIN_Delay(0x004FFFFF);
    /**********************************************************************/
    /* End example of BUSY interrupt management */
    /**********************************************************************/          
	UARTprintf("End example of BUSY interrupt management\n");
	UARTprintf("Move by 60,000 steps rorward, range 0 to 4,194,303\n");

	/* Move by 60,000 steps rorward, range 0 to 4,194,303 */
	cSPIN_Move(FWD, (uint32_t)(60000));
	UARTprintf("Move by 60,000 steps rorward, range 0 to 4,194,303\n");
	/* Wait until not busy - busy pin test */
	while(cSPIN_Busy_HW());
	UARTprintf("Send cSPIN command change hold duty cycle to 0.5% \n");
	/* Send cSPIN command change hold duty cycle to 0.5% */
	cSPIN_Set_Param(cSPIN_KVAL_HOLD, Kval_Perc_to_Par(0.5));
	UARTprintf("Send cSPIN command change run duty cycle to 5%\n");
	/* Send cSPIN command change run duty cycle to 5% */
	cSPIN_Set_Param(cSPIN_KVAL_RUN, Kval_Perc_to_Par(5));
	UARTprintf("Run constant speed of 50 steps/s reverse direction\n");
	/* Run constant speed of 50 steps/s reverse direction */
	cSPIN_Run(REV, Speed_Steps_to_Par(50));
	UARTprintf("Wait few seconds - motor turns\n");
	/* Wait few seconds - motor turns */
	cSPIN_Delay(0x004FFFFF);
	UARTprintf("Perform SoftStop commmand\n");
	/* Perform SoftStop commmand */
	cSPIN_Soft_Stop();
	UARTprintf("RESET KVAL_HOLD to initial value\n");
    /* RESET KVAL_HOLD to initial value */
	cSPIN_Set_Param(cSPIN_KVAL_HOLD, Kval_Perc_to_Par(cSPIN_CONF_PARAM_KVAL_HOLD));
	UARTprintf("RESET KVAL_RUN to initial value\n");
	/* RESET KVAL_RUN to initial value */
	cSPIN_Set_Param(cSPIN_KVAL_RUN, Kval_Perc_to_Par(cSPIN_CONF_PARAM_KVAL_RUN));
	UARTprintf("Wait until not busy - busy status check in Status register\n");
	/* Wait until not busy - busy status check in Status register */
	while(cSPIN_Busy_SW());
	UARTprintf("Move by 100,000 steps forward, range 0 to 4,194,303\n");
	/* Move by 100,000 steps forward, range 0 to 4,194,303 */
	cSPIN_Move(FWD, (uint32_t)(100000));
	UARTprintf("Wait until not busy\n");
	/* Wait until not busy */
	while(cSPIN_Busy_SW());
	UARTprintf("Wait a few seconds, LED busy is off\n");
    /* Wait a few seconds, LED busy is off */
    cSPIN_Delay(0x004FFFFF);
	UARTprintf("Test of the Flag pin by polling, wait in endless cycle if problem is detected\n");
	/* Test of the Flag pin by polling, wait in endless cycle if problem is detected */
	if(cSPIN_Flag()) while(1);
	UARTprintf("Issue cSPIN Go Home command\n");
	/* Issue cSPIN Go Home command */
	cSPIN_Go_Home();
	UARTprintf("Wait untill not busy - busy pin test\n");
	/* Wait untill not busy - busy pin test */
	while(cSPIN_Busy_HW());
	UARTprintf("Wait a few seconds, LED busy is off\n");
    /* Wait a few seconds, LED busy is off */
    cSPIN_Delay(0x004FFFFF);
	UARTprintf("Issue cSPIN Go To command\n");
	/* Issue cSPIN Go To command */
	cSPIN_Go_To(0x0000FFFF);
	UARTprintf(" Wait untill not busy - busy pin test\n");
	/* Wait untill not busy - busy pin test */
	while(cSPIN_Busy_HW());
	UARTprintf("Wait a few seconds, LED busy is off\n");
    /* Wait a few seconds, LED busy is off */
    cSPIN_Delay(0x004FFFFF);
	UARTprintf("Issue cSPIN Go To command\n");
	/* Issue cSPIN Go To command */
	cSPIN_Go_To_Dir(FWD, 0x0001FFFF);
	UARTprintf("Wait untill not busy - busy pin test\n");
	/* Wait untill not busy - busy pin test */
	while(cSPIN_Busy_HW());
	UARTprintf("Wait a few seconds, LED busy is off\n");
    /* Wait a few seconds, LED busy is off */
    cSPIN_Delay(0x004FFFFF);
	UARTprintf("Read run duty cycle (cSPIN_KVAL_RUN) parameter from cSPIN\n");
	/* Read run duty cycle (cSPIN_KVAL_RUN) parameter from cSPIN */
	cSPIN_rx_data = cSPIN_Get_Param(cSPIN_KVAL_RUN);
	UARTprintf("Read intersect speed (cSPIN_INT_SPD) parameter from cSPIN\n");
	/* Read intersect speed (cSPIN_INT_SPD) parameter from cSPIN */
	cSPIN_rx_data = cSPIN_Get_Param(cSPIN_INT_SPD);
	UARTprintf("Read Status register content\n");
	/* Read Status register content */
	cSPIN_rx_data = cSPIN_Get_Status();
	UARTprintf("Read absolute position (cSPIN_ABS_POS) parameter from cSPIN\n");
	/* Read absolute position (cSPIN_ABS_POS) parameter from cSPIN */
	cSPIN_rx_data = cSPIN_Get_Param(cSPIN_ABS_POS);
	UARTprintf("Reset position counter\n");
	/* Reset position counter */
	cSPIN_Reset_Pos();
	UARTprintf("Read absolute position (cSPIN_ABS_POS) parameter from cSPIN\n");
	/* Read absolute position (cSPIN_ABS_POS) parameter from cSPIN */
	cSPIN_rx_data = cSPIN_Get_Param(cSPIN_ABS_POS);
	UARTprintf("Issue cSPIN Hard HiZ command - disable power stage (High Impedance)\n");
	/* Issue cSPIN Hard HiZ command - disable power stage (High Impedance) */
	cSPIN_Hard_HiZ();
 	UARTprintf("Start example of GoUntil Command\n");
    /**********************************************************************/
    /* Start example of GoUntil Command */
    /**********************************************************************/
    /* Configure Interrupt for switch motor in case the MCU is used to pilot the switch */
    //cSPIN_Switch_Motor_Interrupt_Config();        
    /* Motion in FW direction at speed 400steps/s via GoUntil command*/
    /* When SW is closed:  */
    /*    As ACT is set to ACTION_COPY, ABS_POS is saved to MARK register */
    /*    then a soft stop is done          */
	UARTprintf("Motion in FW direction at speed 400steps/s via GoUntil command,When SW is closed:  As ACT is set to ACTION_COPY, ABS_POS is saved to MARK register\n");
	UARTprintf("then a soft stop is done\n");
    cSPIN_Go_Until(ACTION_COPY, FWD, Speed_Steps_to_Par(400));
    UARTprintf("Waiting for soft Stop after GoUntil command\n");
    /* Waiting for soft Stop after GoUntil command*/
    while(cSPIN_Busy_HW());
	UARTprintf("User action needed to go further\n");
    /* User action needed to go further */
    /* User attention drawn by toggling a LED */
    //  cSPIN_Gpio_Toggle(LED_SPARE_Port, LED_SPARE_Pin); 
    cSPIN_Delay(0x100000);
	UARTprintf("Wait a few seconds, LED busy is off\n");
    //GPIO_ResetBits(LED_SPARE_Port, LED_SPARE_Pin);
    /* Wait a few seconds, LED busy is off */
    cSPIN_Delay(0x004FFFFF);
  	UARTprintf("Move by 50,000 steps in reverse direction, range 0 to 4,194,303\n");
    /* Move by 50,000 steps in reverse direction, range 0 to 4,194,303 */
    cSPIN_Move(REV, (uint32_t)(50000));
     UARTprintf("Waiting for end of move command\n");  
        /* Waiting for end of move command*/
	while(cSPIN_Busy_HW());
    UARTprintf("Wait a few seconds, LED busy is off \n");    
    /* Wait a few seconds, LED busy is off */
    cSPIN_Delay(0x004FFFFF);
    UARTprintf("Go to Mark saved with GoUntil command \n");
    /* Go to Mark saved with GoUntil command */
    cSPIN_Go_Mark();
    UARTprintf("Wait until not busy - busy pin test\n");
    /* Wait until not busy - busy pin test */
	while(cSPIN_Busy_HW());
	UARTprintf("Wait a few seconds, LED busy is off\n");
    /* Wait a few seconds, LED busy is off */
    cSPIN_Delay(0x004FFFFF);
    UARTprintf("End example of GoUntil Command\n");
    /**********************************************************************/
    /* End example of GoUntil Command */
    /**********************************************************************/
	UARTprintf("End example of GoUntil Command\n");
    UARTprintf("Start example of ReleaseSw Command\n");
    /**********************************************************************/
    /* Start example of ReleaseSw Command */
    /**********************************************************************/
    /* Motion via cSPIN_Release_SW command in REV direction at minimum speed*/
    /* (or  5 steps/s is minimum speed is < 5step/s)*/
    /* When SW is opened :  */
    /*    As ACT is set to ACTION_RESET, ABS_POS is reset i.e Home position is set */
    /*    then a soft stop is done          */
	UARTprintf("Motion via cSPIN_Release_SW command in REV direction at minimum speed\n");
	UARTprintf("(or  5 steps/s is minimum speed is < 5step/s)\n");
	UARTprintf("When SW is opened : \n");
	UARTprintf("As ACT is set to ACTION_RESET, ABS_POS is reset i.e Home position is set\n");
	UARTprintf("then a soft stop is done      \n");
    cSPIN_Release_SW(ACTION_RESET, REV);
    
    /* Waiting for soft Stop after ReleaseSw command*/
    while(cSPIN_Busy_HW());
    /* User action needed to go further */
    /* User attention drawn by toggling a LED */
    //  cSPIN_Gpio_Toggle(LED_SPARE_Port, LED_SPARE_Pin); 
      cSPIN_Delay(0x100000);
	UARTprintf("\n");
    //GPIO_ResetBits(LED_SPARE_Port, LED_SPARE_Pin);
    UARTprintf("Wait a few seconds, LED busy is off\n");
    /* Wait a few seconds, LED busy is off */
    cSPIN_Delay(0x004FFFFF);
    UARTprintf("Move by 100,000 steps forward, range 0 to 4,194,303\n");
    /* Move by 100,000 steps forward, range 0 to 4,194,303 */
    cSPIN_Move(FWD, (uint32_t)(100000));
   UARTprintf("Waiting for end of move command\n");
    /* Waiting for end of move command*/
	while(cSPIN_Busy_HW());
    UARTprintf("Wait a few seconds, LED busy is off\n");    
    /* Wait a few seconds, LED busy is off */
    cSPIN_Delay(0x004FFFFF);        
    UARTprintf("Go to Home set with ReleaseSW command\n");
    /* Go to Home set with ReleaseSW command */
    cSPIN_Go_Home();
    UARTprintf("Wait until not busy - busy pin test\n");
   /* Wait until not busy - busy pin test */
	while(cSPIN_Busy_HW());
    UARTprintf("Wait a few seconds, LED busy is off\n");    
    /* Wait a few seconds, LED busy is off */
    cSPIN_Delay(0x004FFFFF);
    /**********************************************************************/
    /* End example of ReleaseSw Command */
    /**********************************************************************/
	UARTprintf("End example of ReleaseSw Command\n");

     /* Get Status to clear FLAG due to switch turn-on event (falling edge on SW pin) */
    cSPIN_rx_data = cSPIN_Get_Status();       
    UARTprintf("Start example of StepClock Command\n");
    /**********************************************************************/
    /* Start example of StepClock Command */
    /**********************************************************************/
    //cSPIN_Busy_Interrupt_GPIO_DeConfig();
    /* Enable Step Clock Mode */
    cSPIN_Step_Clock(FWD);
	UARTprintf("Wait a few seconds, LED busy is off\n");
    //cSPIN_Busy_Interrupt_GPIO_Config();
    /* Wait a few seconds, LED busy is off */
    cSPIN_Delay(0x004FFFFF);
    UARTprintf("Set PWM period to 500 so PWM Frequency = 1MHz/ 500 = 2KHz\n");
    /* Set PWM period to 500 so PWM Frequency = 1MHz/ 500 = 2KHz */
    /* and so, motor moves at 2000 steps/s */
    cSPIN_PWM_Enable(2000);
    cSPIN_Delay(0x00FFFFFF);
    cSPIN_PWM_DISABLE();
	UARTprintf("End example of StepClock Command\n");
    /**********************************************************************/
    /* End example of StepClock Command */
    /**********************************************************************/
    //
    // Loop forever echoing data through the UART.
    //
    while(1)
    {
    }
}
