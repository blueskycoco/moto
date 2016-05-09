#include "lib6480.h"
void cSPIN_Peripherals_Init(void)
{
  /* Used peripherals clock enable -------------------------------------------*/
  RCC_APB1PeriphClockCmd(cSPIN_PERIPHERAL_CLKs_APB1, ENABLE);
  RCC_APB2PeriphClockCmd(cSPIN_PERIPHERAL_CLKs_APB2, ENABLE);
  
  /* Configure pins used by cSPIN --------------------------------------------*/
#if (defined(STEVAL_PCC009V2) || defined(ST_CSPIN_6480H_DISCOVERY))
  /* Configure on-board power LED ------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = POWER_LED_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(POWER_LED_Port, &GPIO_InitStructure);
  GPIO_SetBits(POWER_LED_Port, POWER_LED_Pin);
  /* Configure on-board status LED -----------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = STATUS_LED_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(STATUS_LED_Port, &GPIO_InitStructure);
  /* Configure STBY_RESET GPIO connected to cSPIN STBY_RESET pin*/
  GPIO_InitStructure.GPIO_Pin = cSPIN_STBY_RESET_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(cSPIN_STBY_RESET_Port, &GPIO_InitStructure);
#endif /* (defined(STEVAL_PCC009V2) || defined(ST_CSPIN_6480H_DISCOVERY)) */ 
#ifdef STEVAL_PCC009V2 /* Only if PCC009V2 evalboard is used ---------------*/
  /* Configure Port C GPIO pin 2 connected to keypad button "*" */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* Configure Port C GPIO pin 3 connected to keypad button "7" */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* Configure Port C GPIO pin 6 connected to keypad button "4" */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif /* STEVAL_PCC009V2 */    
#ifdef ST_CSPIN_6480H_DISCOVERY /* Only if DISCOVERY board is used -----------*/
  /* Configure on-board busy LED -----------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = LED_BUSY_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED_BUSY_Port, &GPIO_InitStructure);
  /* Configure on-board spare LED --------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = LED_SPARE_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED_SPARE_Port, &GPIO_InitStructure);
  /* Configure GPIO connected to S1 button via BUTTON_A wire */
  GPIO_InitStructure.GPIO_Pin = BUTTON_A_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(BUTTON_A_Port, &GPIO_InitStructure);
  /* Configure GPIO connected to S3 button via BUTTON_B wire */
  GPIO_InitStructure.GPIO_Pin = BUTTON_B_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(BUTTON_B_Port, &GPIO_InitStructure);
  /* Configure SW_MOTOR GPIO connected to J8 jumper */
  GPIO_InitStructure.GPIO_Pin = SW_MOTOR_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(SW_MOTOR_Port, &GPIO_InitStructure);
  /* Configure SW GPIO connected to cSPIN SW pin*/
  GPIO_InitStructure.GPIO_Pin = cSPIN_SW_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(cSPIN_SW_Port, &GPIO_InitStructure);
  GPIO_SetBits(cSPIN_SW_Port, cSPIN_SW_Pin);
#endif /* ST_CSPIN_6480H_DISCOVERY */
      
  /* Configure SPI pin: SCK --------------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = cSPIN_SCK_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(cSPIN_SCK_Port, &GPIO_InitStructure);

  /* Configure SPI pin: MOSI -------------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = cSPIN_MOSI_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(cSPIN_MOSI_Port, &GPIO_InitStructure);

  /* Configure SPI pin: nSS --------------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = cSPIN_nSS_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(cSPIN_nSS_Port, &GPIO_InitStructure);

  /* Configure SPI pin: MISO --------------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = cSPIN_MISO_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(cSPIN_MISO_Port, &GPIO_InitStructure);
  
  /* Configure cSPIN - Busy pin ----------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = cSPIN_BUSY_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(cSPIN_BUSY_Port, &GPIO_InitStructure);	  
  				
  /* Configure cSPIN - Flag pin ----------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = cSPIN_FLAG_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(cSPIN_FLAG_Port, &GPIO_InitStructure);	  
  
  /* Configure PWM connected to cSPin STCK -----------------------------------*/  
  GPIO_InitStructure.GPIO_Pin = cSPIN_PWM1_Pin;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(cSPIN_PWM1_Port, &GPIO_InitStructure);	  
  
  /* SPI configuration ------------------------------------------------------*/
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_Init(cSPIN_SPI, &SPI_InitStructure);
  
  /* Enable SPI */
  SPI_Cmd(cSPIN_SPI, ENABLE);

  /* Interrupt Channel configuration and enable */
  cSPIN_Interrupt_Channel_Config();

}
