//*****************************************************************************
//
// rgb_commands.h - Prototypes for the evaluation board command line utils.
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

#ifndef __RGB_COMMANDS_H__
#define __RGB_COMMANDS_H__

//*****************************************************************************
//
// Defines for the command line argument parser provided as a standard part of
// TivaWare.  qs-rgb application uses the command line parser to extend
// functionality to the serial port.
//
//*****************************************************************************

#define CMDLINE_MAX_ARGS 8

//*****************************************************************************
//
// Declaration for the callback functions that will implement the command line
// functionality.  These functions get called by the command line interpreter
// when the corresponding command is typed into the command line.
//
//*****************************************************************************
extern int CMD_help (int argc, char **argv);
extern int CMD_nop (int argc, char **argv);
extern int CMD_sParam (int argc, char **argv);
extern int CMD_gParam (int argc, char **argv);
extern int CMD_Run (int argc, char **argv);
extern int CMD_sClock (int argc, char **argv);
extern int CMD_Move (int argc, char **argv);
extern int CMD_goTo (int argc, char **argv);
extern int CMD_goToDir (int argc, char **argv);
extern int CMD_goUntil (int argc, char **argv);
extern int CMD_releaseSW (int argc, char **argv);
extern int CMD_goHome (int argc, char **argv);
extern int CMD_goMark (int argc, char **argv);
extern int CMD_resetPos (int argc, char **argv);
extern int CMD_resetDevice (int argc, char **argv);
extern int CMD_softStop (int argc, char **argv);
extern int CMD_hardStop (int argc, char **argv);
extern int CMD_softHiZ (int argc, char **argv);
extern int CMD_hardHiZ (int argc, char **argv);
extern int CMD_getStatus (int argc, char **argv);
extern int CMD_busyHW (int argc, char **argv);
extern int CMD_busySW (int argc, char **argv);
extern int CMD_flagHW (int argc, char **argv);
extern int CMD_auto (int argc, char **argv);
extern void auto_test(void);
extern void auto_mtest(void);
extern int CMD_pwm(int argc, char **argv);
extern int CMD_MsParam(int argc, char **argv);
extern int CMD_Mauto(int argc, char **argv);
extern int CMD_MbusySW(int argc, char **argv);
extern int CMD_MCmd(int argc, char **argv);
extern int CMD_MRun(int argc, char **argv);
extern int CMD_MMove(int argc, char **argv);
extern int CMD_MgParam(int argc, char **argv);
extern int CMD_MgetStatus(int argc, char **argv);
#endif //__RGB_COMMANDS_H__
