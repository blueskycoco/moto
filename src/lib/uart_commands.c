//*****************************************************************************
//
// rgb_commands.c - Command line functionality implementation
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
#include "cspin.h"
#include "inc/hw_types.h"
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
#include "utils/cmdline.h"
#include "uart_commands.h"
extern uint32_t responseArray[NUMBER_OF_SLAVES];
extern uint8_t commandArray[NUMBER_OF_SLAVES];
extern uint32_t argumentArray[NUMBER_OF_SLAVES];
extern uint8_t arrayTxBytes[NUMBER_OF_SLAVES];
extern uint32_t arrayValues[NUMBER_OF_SLAVES];
//*****************************************************************************
//
// Table of valid command strings, callback functions and help messages.  This
// is used by the cmdline module.
//
//*****************************************************************************
tCmdLineEntry g_psCmdTable[] =
{
    {"help",     CMD_help,      " : Display list of commands" },
	{"nop",      CMD_nop,       " : Issues cSPIN NOP command"},
	{"sParam",   CMD_sParam,    " : Issues cSPIN Set Param command,Ex sParam param value"},
	{"gParam",   CMD_gParam,    " : Issues cSPIN Get Param command,Ex gParam param"},
	{"Run",      CMD_Run,       " : Issues cSPIN Run command,Ex Run dir(1 fwd,0 rev) speed"},
	{"sClock",   CMD_sClock,    " : Issues cSPIN Step Clock command,Ex sClock dir(1 fwd,0 rev)"},
	{"Move",     CMD_Move,      " : Issues cSPIN Move command,Ex Move dir(1 fwd,0 rev) n_step"},
	{"goTo",     CMD_goTo,      " : Issues cSPIN Go To command,Ex goTo abs_pos"},
	{"goToDir",  CMD_goToDir,   " : Issues cSPIN Go To Dir command,Ex goToDir dir(1 fwd,0 rev) abs_pos"},
	{"goUntil",  CMD_goUntil,   " : Issues cSPIN Go Until command,Ex goUntil action(0 reset,8 copy) dir(1 fwd,0 rev) speed"},
	{"releaseSW",CMD_releaseSW, " : Issues cSPIN Release SW command,Ex releaseSW action(0 reset,8 copy) dir(1 fwd,0 rev)"},
	{"goHome",   CMD_goHome,    " : Issues cSPIN Go Home command. (Shorted path to zero position)"},
	{"goMark",   CMD_goMark,    " : Issues cSPIN Go Mark command"},
	{"resetPos", CMD_resetPos,  " : Issues cSPIN Reset Pos command"},
	{"resetDevice",CMD_resetDevice,  " : Issues cSPIN Reset Device command"},
	{"softStop", CMD_softStop,  " : Issues cSPIN Soft Stop command"},
	{"hardStop", CMD_hardStop,  " : Issues cSPIN Hard Stop command"},
	{"softHiZ",  CMD_softHiZ,   " : Issues cSPIN Soft HiZ command"},
	{"hardHiZ",  CMD_hardHiZ,   " : Issues cSPIN Hard HiZ command"},
	{"getStatus",CMD_getStatus, " : Issues cSPIN Get Status command"},
	{"busyHW",   CMD_busyHW,    " : Checks if the cSPIN is Busy by hardware - active Busy signal"},
	{"busySW",   CMD_busySW,    " : Checks if the cSPIN is Busy by SPI - Busy flag bit in Status Register"},
	{"flagHW",   CMD_flagHW,    " : Checks cSPIN Flag signal"},
	{"pwm",   	 CMD_pwm,	    " : STCK pwm,Ex pwm 2000 1000000"},
	{"auto",   	 CMD_auto,	    " : auto test mode"},
	{"msParam",  CMD_MsParam,  	" : Issues cSPIN Set multi Param command,Ex msParam No(0-3) param value"},
	{"mgParam",  CMD_MgParam,   " : Issues cSPIN Get multi Param command,Ex mgParam No(0-3) param"},
	{"mMove",    CMD_MMove,     " : Issues cSPIN multi Move command,Ex mMove No(0-3) dir(1 fwd,0 rev) n_step"},
	{"mRun",     CMD_MRun,      " : Issues cSPIN multi Run command,Ex mRun No(0-3) dir(1 fwd,0 rev) speed"},
	{"mCmd",     CMD_MCmd,      " : Issues cSPIN command(like goto,gotdir,gohome,etc),Ex mCmd No(0-3) param value"},
	{"mgetStatus",CMD_MgetStatus, " : Issues cSPIN Get all l6480 Status command"},
	{"mbusySW",   CMD_MbusySW,    " : Checks if the multi cSPIN is Busy by SPI - Busy flag bit in Status Register"},
	{"mauto",   	 CMD_Mauto,	    " : Multi auto test mode"},
    { 0, 0, 0 }
};
int
CMD_Mauto(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

	auto_mtest();

    return (0);
}
int
CMD_MbusySW(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    if(cSPIN_One_Or_More_Slaves_Busy_SW(NUMBER_OF_SLAVES))
		UARTprintf("at least one sw Moto is busy\n");
	else
		UARTprintf("all sw Moto is free\n");

    return (0);
}
int
CMD_MgetStatus(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

	cSPIN_All_Slaves_Get_Status(NUMBER_OF_SLAVES, responseArray);
    UARTprintf("Dev 0 Status = %2X\n",responseArray[0]);
	UARTprintf("Dev 1 Status = %2X\n",responseArray[1]);
	UARTprintf("Dev 2 Status = %2X\n",responseArray[2]);
	UARTprintf("Dev 3 Status = %2X\n",responseArray[3]);
    return (0);
}
int
CMD_MCmd(int argc, char **argv)
{
	uint32_t ui32No;
	uint32_t ui32Param;
	uint32_t ui32Value;
	int i;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc >= 3)
	{
		ui32No 	  =	ustrtoul(argv[1], 0, 10);
		ui32Param = ustrtoul(argv[2], 0, 10);
		if(argc==4)
		ui32Value = ustrtoul(argv[3], 0, 10);
		for (i = 0; i < NUMBER_OF_SLAVES; i++)
		  {
		    if (i == ui32No)
		    {
		      commandArray[i] = (uint8_t)ui32Param;
			  if(argc==4)
		      	argumentArray[i] = ui32Value;
		    }
		    else
		    {
		      commandArray[i] = cSPIN_NOP;
		      argumentArray[i] = cSPIN_NOP;
		    }
		  }
		cSPIN_All_Slaves_Send_Command(NUMBER_OF_SLAVES, commandArray, argumentArray);
	}
	
    return (0);
}
int
CMD_MRun(int argc, char **argv)
{
	uint32_t ui32No;
	uint32_t ui32Dir;
	uint32_t ui32Speed;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 4)
	{
		ui32No 	  =	ustrtoul(argv[1], 0, 10);
		ui32Dir   = ustrtoul(argv[2], 0, 10);
		ui32Speed  = ustrtoul(argv[3], 0, 10);
		cSPIN_One_Slave_Run(ui32No,NUMBER_OF_SLAVES,ui32Dir,ui32Speed);
	}
	
    return (0);
}
int
CMD_MMove(int argc, char **argv)
{
	uint32_t ui32No;
	uint32_t ui32Dir;
	uint32_t ui32Step;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 4)
	{
		ui32No 	  =	ustrtoul(argv[1], 0, 10);
		ui32Dir   = ustrtoul(argv[2], 0, 10);
		ui32Step  = ustrtoul(argv[3], 0, 10);
		cSPIN_One_Slave_Move(ui32No,NUMBER_OF_SLAVES,ui32Dir,ui32Step);
	}
	
    return (0);
}
int
CMD_MsParam(int argc, char **argv)
{
	uint32_t ui32No;
	uint32_t ui32Param;
	uint32_t ui32Value;
	int i;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 4)
	{
		ui32No 	  =	ustrtoul(argv[1], 0, 10);
		ui32Param = ustrtoul(argv[2], 0, 10);
		ui32Value = ustrtoul(argv[3], 0, 10);
		for (i = 0; i < NUMBER_OF_SLAVES; i++)
		  {
		    if (i == ui32No)
		    {
		      arrayTxBytes[i] = (uint8_t)ui32Param;
		      arrayValues[i] = ui32Value;
		    }
		    else
		    {
		      arrayTxBytes[i] = cSPIN_NOP;
		      arrayValues[i] = cSPIN_NOP;
		    }
		  }
		cSPIN_All_Slaves_Set_Param(NUMBER_OF_SLAVES,arrayTxBytes,arrayValues);
	}
	
    return (0);
}
int
CMD_MgParam(int argc, char **argv)
{
	uint32_t ui32No;
	uint32_t ui32Param;
	int i;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 3)
	{
		ui32No 	  =	ustrtoul(argv[1], 0, 10);
		ui32Param = ustrtoul(argv[2], 0, 10);
		for (i = 0; i < NUMBER_OF_SLAVES; i++)
		  {
		    if (i == ui32No)
		    {
		      commandArray[i] = (uint8_t)ui32Param;
		    }
		    else
		    {
		      commandArray[i] = cSPIN_NOP;
		    }
		  }
		cSPIN_All_Slaves_Set_Param(NUMBER_OF_SLAVES,arrayTxBytes,responseArray);
		UARTprintf("Dev %d Reg %2X = %2X\n",ui32No,ui32Param,responseArray[ui32No]);
	}
	
    return (0);
}
int
CMD_nop(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    cSPIN_Nop();

    return (0);
}
int
CMD_sParam(int argc, char **argv)
{
	uint32_t ui32Param;
	uint32_t ui32Value;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 3)
	{
		ui32Param = ustrtoul(argv[1], 0, 10);
		ui32Value = ustrtoul(argv[2], 0, 10);
		cSPIN_Set_Param(ui32Param,ui32Value);
	}
	
    return (0);
}
int
CMD_gParam(int argc, char **argv)
{
	uint32_t ui32Param;
	uint32_t ui32Value;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 2)
	{
		ui32Param = ustrtoul(argv[1], 0, 10);
		ui32Value=cSPIN_Get_Param(ui32Param);
		UARTprintf("Reg %2X = %2X\n",ui32Param,ui32Value);
	}
	
    return (0);
}
int
CMD_Run(int argc, char **argv)
{
	uint32_t ui32Dir;
	uint32_t ui32Speed;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 3)
	{
		ui32Dir = ustrtoul(argv[1], 0, 10);
		ui32Speed = ustrtoul(argv[2], 0, 10);
		cSPIN_Run(ui32Dir,ui32Speed);
	}
	
    return (0);
}
int
CMD_sClock(int argc, char **argv)
{
	uint32_t ui32Dir;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 2)
	{
		ui32Dir = ustrtoul(argv[1], 0, 10);
		cSPIN_Step_Clock(ui32Dir);
	}
	
    return (0);
}
int
CMD_Move(int argc, char **argv)
{
	uint32_t ui32Dir;
	uint32_t ui32Step;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 3)
	{
		ui32Dir = ustrtoul(argv[1], 0, 10);
		ui32Step = ustrtoul(argv[2], 0, 10);
		cSPIN_Move(ui32Dir,ui32Step);
	}
	
    return (0);
}
int
CMD_goTo(int argc, char **argv)
{
	uint32_t ui32Abs_pos;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 2)
	{
		ui32Abs_pos = ustrtoul(argv[1], 0, 10);
		cSPIN_Go_To(ui32Abs_pos);
	}
	
    return (0);
}
int
CMD_goToDir(int argc, char **argv)
{
	uint32_t ui32Dir;
	uint32_t ui32Abs_pos;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 3)
	{
		ui32Dir = ustrtoul(argv[1], 0, 10);
		ui32Abs_pos = ustrtoul(argv[2], 0, 10);
		cSPIN_Go_To_Dir(ui32Dir,ui32Abs_pos);
	}
	
    return (0);
}
int
CMD_goUntil(int argc, char **argv)
{
	uint32_t ui32Action;
	uint32_t ui32Dir;
	uint32_t ui32Speed;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 4)
	{
		ui32Action = ustrtoul(argv[1], 0, 10);
		ui32Dir = ustrtoul(argv[2], 0, 10);
		ui32Speed = ustrtoul(argv[3], 0, 10);
		cSPIN_Go_Until(ui32Action,ui32Dir,ui32Speed);
	}
	
    return (0);
}
int
CMD_releaseSW(int argc, char **argv)
{
	uint32_t ui32Dir;
	uint32_t ui32Action;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 3)
	{
		ui32Action = ustrtoul(argv[1], 0, 10);
		ui32Dir = ustrtoul(argv[2], 0, 10);
		cSPIN_Release_SW(ui32Action,ui32Dir);
	}
	
    return (0);
}
int
CMD_goHome(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    cSPIN_Go_Home();

    return (0);
}

int
CMD_goMark(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    cSPIN_Go_Mark();

    return (0);
}
int
CMD_resetPos(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    cSPIN_Reset_Pos();

    return (0);
}
int
CMD_resetDevice(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    cSPIN_Reset_Device();

    return (0);
}
int
CMD_softStop(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    cSPIN_Soft_Stop();

    return (0);
}
int
CMD_hardStop(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    cSPIN_Hard_Stop();

    return (0);
}
int
CMD_softHiZ(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    cSPIN_Soft_HiZ();

    return (0);
}
int
CMD_hardHiZ(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    cSPIN_Hard_HiZ();

    return (0);
}
int
CMD_getStatus(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    UARTprintf("Status = %2X\n",cSPIN_Get_Status());

    return (0);
}
int
CMD_busyHW(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    if(cSPIN_Busy_HW())
		UARTprintf("hw Moto is busy\n");
	else
		UARTprintf("hw Moto is free\n");

    return (0);
}
int
CMD_busySW(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    if(cSPIN_Busy_SW())
		UARTprintf("sw Moto is busy\n");
	else
		UARTprintf("sw Moto is free\n");

    return (0);
}
int
CMD_auto(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

	auto_test();

    return (0);
}
int
CMD_flagHW(int argc, char **argv)
{
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;

    if(cSPIN_Flag())
		UARTprintf("Flag is active\n");
	else
		UARTprintf("Flag is zero\n");

    return (0);
}
int
CMD_pwm(int argc, char **argv)
{
	uint32_t ui32Hz;
	uint32_t ui32Delay;
    //
    // Keep the compiler happy.
    //
    (void)argc;
    (void)argv;
	if(argc == 3)
	{
		ui32Hz = ustrtoul(argv[1], 0, 10);
		ui32Delay = ustrtoul(argv[2], 0, 10);
		UARTprintf("Hz %d,last %d\n",ui32Hz,ui32Delay);
		cSPIN_PWM_Enable(ui32Hz);
		cSPIN_Delay(ui32Delay);
		cSPIN_PWM_DISABLE();
	}

    return (0);
}

//*****************************************************************************
//
// Command: help
//
// Print the help strings for all commands.
//
//*****************************************************************************
int
CMD_help(int argc, char **argv)
{
    int32_t i32Index;

    (void)argc;
    (void)argv;

    //
    // Start at the beginning of the command table
    //
    i32Index = 0;

    //
    // Get to the start of a clean line on the serial output.
    //
    UARTprintf("\nAvailable Commands\n------------------\n\n");

    //
    // Display strings until we run out of them.
    //
    while(g_psCmdTable[i32Index].pcCmd)
    {
      UARTprintf("%17s %s\n", g_psCmdTable[i32Index].pcCmd,
                 g_psCmdTable[i32Index].pcHelp);
      i32Index++;
    }

    //
    // Leave a blank line after the help strings.
    //
    UARTprintf("\n");

    return (0);
}

