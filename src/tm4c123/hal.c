#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "utils/uartstdio.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "inc/hw_ints.h"
#include "cspin.h"
#include "utils/softssi.h"
#include "driverlib/systick.h"
extern int flag,busy;
tSoftSSI g_sSoftSSI;
uint16_t g_pui16TxBuffer[16];
uint16_t g_pui16RxBuffer[16];
void
SysTickIntHandler(void)
{
    //
    // Call the SoftSSI timer tick.
    //
    SoftSSITimerTick(&g_sSoftSSI);
}

void
IntGPIOa(void)
{   
	if(GPIOIntStatus(GPIO_PORTA_BASE, true)&GPIO_PIN_7)
	{	
		flag=((ROM_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7)&(GPIO_PIN_7))==GPIO_PIN_7)?0:1;
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_INT_PIN_7);
	}
	else if(GPIOIntStatus(GPIO_PORTA_BASE, true)&GPIO_PIN_6)
	{
		busy=((ROM_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6)&(GPIO_PIN_6))==GPIO_PIN_6)?0:1;
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_INT_PIN_6);
	}
}
void
InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

void
initUart(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    //
    // Prompt for text to be entered.
    //
//    UARTSend((uint8_t *)"\033[2JEnter text: ", 16);
	InitConsole();

}
//*****************************************************************************
//
// This function sets up L6480 Spi,BUSY,FLAG,STBY_RESET,STCK Signal
//
//*****************************************************************************
void cSPIN_Peripherals_Init(void)
{
	uint32_t tmp;
	#if 1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	//GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 |
					   GPIO_PIN_2);
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3,
						   SSI_MODE_MASTER, 1000000, 8);
	SSIEnable(SSI0_BASE);
	while(SSIDataGetNonBlocking(SSI0_BASE, &tmp))
    {
    }
	#else
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	memset(&g_sSoftSSI, 0, sizeof(g_sSoftSSI));
    SoftSSIClkGPIOSet(&g_sSoftSSI, GPIO_PORTA_BASE, GPIO_PIN_2);
    //SoftSSIFssGPIOSet(&g_sSoftSSI, GPIO_PORTA_BASE, GPIO_PIN_3);
    SoftSSIRxGPIOSet(&g_sSoftSSI, GPIO_PORTA_BASE, GPIO_PIN_4);
    SoftSSITxGPIOSet(&g_sSoftSSI, GPIO_PORTA_BASE, GPIO_PIN_5);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1);
    SoftSSIRxBufferSet(&g_sSoftSSI, g_pui16RxBuffer,
                       sizeof(g_pui16RxBuffer) / sizeof(g_pui16RxBuffer[0]));
    SoftSSITxBufferSet(&g_sSoftSSI, g_pui16TxBuffer,
                       sizeof(g_pui16TxBuffer) / sizeof(g_pui16TxBuffer[0]));
    SoftSSIConfigSet(&g_sSoftSSI, SOFTSSI_FRF_MOTO_MODE_3, 8);
    SoftSSIEnable(&g_sSoftSSI);
	SysTickPeriodSet(SysCtlClockGet()/100000);
	SysTickIntEnable();
    SysTickEnable();
	#endif
	ROM_IntMasterEnable();

	GPIOIntDisable(GPIO_PORTA_BASE,GPIO_INT_PIN_7);
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_BOTH_EDGES);
	ROM_IntEnable(INT_GPIOA);
	GPIOIntEnable(GPIO_PORTA_BASE,GPIO_INT_PIN_7);
	int ui32Status = GPIOIntStatus(GPIO_PORTA_BASE, true);
	GPIOIntClear(GPIO_PORTA_BASE, ui32Status);
	
	//BUSY config
	GPIOIntDisable(GPIO_PORTA_BASE,GPIO_INT_PIN_6);
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_BOTH_EDGES);
	ROM_IntEnable(INT_GPIOA);
	GPIOIntEnable(GPIO_PORTA_BASE,GPIO_INT_PIN_6);
	ui32Status = GPIOIntStatus(GPIO_PORTA_BASE, true);
	GPIOIntClear(GPIO_PORTA_BASE, ui32Status);
	flag=((ROM_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7)&(GPIO_PIN_7))==GPIO_PIN_7)?0:1;
	busy=((ROM_GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_6)&(GPIO_PIN_6))==GPIO_PIN_6)?0:1;
	//config pwm
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);
}

/**
  * @brief  Transmits/Receives one byte to/from cSPIN over SPI.
  * @param  byte Transmited byte
  * @retval Received byte
  */
uint8_t cSPIN_Write_Byte(uint8_t byte)
{
	uint32_t result=0;
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
	SSIDataPut(SSI0_BASE, byte);
	//SoftSSIDataPut(&g_sSoftSSI, byte);
	while(SSIBusy(SSI0_BASE));
	SSIDataGet(SSI0_BASE, &result);
	//SoftSSIDataGet(&g_sSoftSSI, &result);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);

	return (result&0xff);
}
/**
  * @brief  Transmits/Receives several bytes to cSPIN over SPI
  * @param  pTxByte pTxBytePointer to TX bytes
  * @param  pRxByte Pointer to RX bytes
  * @param  nBytes Number of TX = RX bytes
  * @retval None
  */
void cSPIN_Write_Daisy_Chain_Bytes(uint8_t *pTxByte, uint8_t *pRxByte, uint8_t nBytes)
{
	uint32_t index;
	uint32_t result=0;
	/* nSS signal activation - low */
	//GPIO_ResetBits(cSPIN_nSS_Port, cSPIN_nSS_Pin);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
	/* SPI byte send */
	for (index = 0; index < nBytes; index++)
	{
		//SPI_I2S_SendData(cSPIN_SPI, *pTxByte);
		SSIDataPut(SSI0_BASE, *pTxByte);
		//SoftSSIDataPut(&g_sSoftSSI, *pTxByte);
		/* Wait for SPIx Busy flag */
		//while (SPI_I2S_GetFlagStatus(cSPIN_SPI, SPI_I2S_FLAG_BSY) != RESET);
		//*pRxByte = SPI_I2S_ReceiveData(cSPIN_SPI);
		
		//SSIDataGet(SSI0_BASE, &result);
		//SoftSSIDataGet(&g_sSoftSSI, &result);
		*pRxByte=(uint8_t)result;
		pTxByte++;
		pRxByte++;
	}
	while(SSIBusy(SSI0_BASE));
	/* nSS signal deactivation - high */
	//GPIO_SetBits(cSPIN_nSS_Port, cSPIN_nSS_Pin);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
}

/**
  * @brief  Resets CSPIN and puts it into standby mode
  * @param  None
  * @retval None
  */
void cSPIN_Reset_And_Standby(void)
{
  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 0);
  cSPIN_Delay(10000);
  GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_PIN_1);
}
/**
  * @brief  Enable a PWM on the STCK pin from STM32 
  * @param  Period to be set (PWM Freq = 1MHZ/Period)
  * @retval None
  */
void cSPIN_PWM_Enable(uint16_t Period)
{
	uint32_t param=SysCtlClockGet()/Period;
	//UARTprintf("PWM set to %d\n",param);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, param);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
         PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 2);
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
}
/**
  * @brief  Disable PWM on the STCK pin from STM32 
  * @param  None
  * @retval None
  */
void cSPIN_PWM_DISABLE(void)
{
	PWMGenDisable(PWM0_BASE, PWM_GEN_0);
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
}

