/**
  ******************************************************************************
  * @file    stm32f10x_it.c 
  * @author  IPC Rennes
  * @version V2.1
  * @date    October 15, 2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "main.h"
#include "cspin.h"
#include "cspin_config.h"

/** @addtogroup STM32F10x Exceptions and Interrupt Handlers
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
  uint8_t cmdArray[NUMBER_OF_SLAVES];
  uint32_t rspArray[NUMBER_OF_SLAVES];
  uint32_t cSPIN_rxdata; 
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
   while (1)
  {
  }
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles RCC interrupt request. 
  * @param  None
  * @retval None
  */
void RCC_IRQHandler(void)
{
  if(RCC_GetITStatus(RCC_IT_HSERDY) != RESET)
  { 
    /* Clear HSERDY interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_HSERDY);

    /* Check if the HSE clock is still available */
    if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
    { 
#ifdef SYSCLK_HSE
      /* Select HSE as system clock source */
      RCC_SYSCLKConfig(RCC_SYSCLKSource_HSE);
#else
 
      /* Enable PLL: once the PLL is ready the PLLRDY interrupt is generated */ 
      RCC_PLLCmd(ENABLE);

#endif /* SYSCLK_HSE */      
    }
  } 

  if(RCC_GetITStatus(RCC_IT_PLLRDY) != RESET)
  { 
    /* Clear PLLRDY interrupt pending bit */
    RCC_ClearITPendingBit(RCC_IT_PLLRDY);

    /* Check if the PLL is still locked */
    if (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != RESET)
    { 
      /* Select PLL as system clock source */
      RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    }
  }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

#if defined(STEVAL_PCC009V2)
/**
  * @brief  This function handles External interrupt Line 2 request.
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  {
    /* Clear the EXTI line 2 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line2);
    if (daisy_chain != 0)
    {
      if (number_of_slaves > 1)
      {
        cmdArray[DEVICE_1] = cSPIN_SOFT_STOP;
        cmdArray[DEVICE_2] = cSPIN_SOFT_STOP;
        cSPIN_All_Slaves_Send_Command(number_of_slaves, cmdArray, 0);
      }
      else
      {
        /* Wait just in case the motor is accelerating or decelerating */
        while(cSPIN_Busy_HW());
        cSPIN_All_Slaves_Get_Status(number_of_slaves, rspArray);
        cSPIN_rxdata = rspArray[DEVICE_1] & cSPIN_STATUS_MOT_STATUS;
        if (cSPIN_rxdata == cSPIN_STATUS_MOT_STATUS_CONST_SPD)
        {
          cmdArray[DEVICE_1] = cSPIN_SOFT_STOP;
          cSPIN_All_Slaves_Send_Command(number_of_slaves, cmdArray, 0);
        }
        else if (cSPIN_rxdata == cSPIN_STATUS_MOT_STATUS_STOPPED)
        {
          cSPIN_One_Slave_Run(DEVICE_1, number_of_slaves, FWD, Speed_Steps_to_Par(MAX_SPEED[DEVICE_1])>>2);
        }
        else
        {
          assert_param(0);
        }
      }
    }
    else
    {
      /* Wait just in case the motor is accelerating or decelerating */
      while(cSPIN_Busy_HW());
      cSPIN_rxdata = cSPIN_Get_Status() & cSPIN_STATUS_MOT_STATUS;
      if (cSPIN_rxdata == cSPIN_STATUS_MOT_STATUS_CONST_SPD)
      {
        cSPIN_Soft_HiZ();
      }
      else if (cSPIN_rxdata == cSPIN_STATUS_MOT_STATUS_STOPPED)
      {
        cSPIN_Run(FWD, Speed_Steps_to_Par(((uint16_t)(cSPIN_CONF_PARAM_MAX_SPEED))>>2));
      }
      else
      {
        assert_param(0);
      }
    }
  } 
}

/**
  * @brief  This function handles External interrupt Line 3 request.
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
    /* Clear the EXTI line 3 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line3);
    /* Wait until the motor is not busy */
    while(cSPIN_Busy_HW());
    if (daisy_chain != 0)
    {
      if (number_of_slaves > 1)
      {
        cSPIN_One_Slave_Run(DEVICE_2, number_of_slaves, FWD, Speed_Steps_to_Par(MAX_SPEED[DEVICE_2]));
      }
      else
      {
        /* Read current speed parameter from cSPIN */
        cmdArray[DEVICE_1] = cSPIN_SPEED;
        cSPIN_All_Slaves_Get_Param(number_of_slaves, cmdArray, rspArray);
        /* Halve the motor speed */
        cSPIN_One_Slave_Run(DEVICE_1, number_of_slaves, FWD, rspArray[DEVICE_1] >> 1);
      }
    }
    else
    {
      /* Read current speed parameter from cSPIN */
      cSPIN_rxdata = cSPIN_Get_Param(cSPIN_SPEED);
      /* Halve the motor speed */
      cSPIN_Run(FWD, cSPIN_rxdata >> 1);
    }
  }
}

/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
    /* Clear the EXTI line 6 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line6);
    /* Wait until the motor is not busy */
    while(cSPIN_Busy_HW());
    if (daisy_chain != 0)
    {
      if (number_of_slaves > 1)
      {
        cSPIN_One_Slave_Run(DEVICE_1, number_of_slaves, FWD, Speed_Steps_to_Par(MAX_SPEED[DEVICE_1]));
      }
      else
      {
        /* Read current speed parameter from cSPIN */
        cmdArray[DEVICE_1] = cSPIN_SPEED;
        cSPIN_All_Slaves_Get_Param(number_of_slaves, cmdArray, rspArray);
        /* Double the motor speed */
        cSPIN_One_Slave_Run(DEVICE_1, number_of_slaves, FWD, rspArray[DEVICE_1] << 1);
      }
    }
    else
    {
      /* Read current speed parameter from cSPIN */
      cSPIN_rxdata = cSPIN_Get_Param(cSPIN_SPEED);
      /* Double the motor speed */
      cSPIN_Run(FWD, cSPIN_rxdata << 1);
    }
  }
}

/**
  * @brief  This function handles External lines 15 to 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
    /* Clear the EXTI line 10 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line10);
    if (GPIO_ReadInputDataBit(cSPIN_BUSY_Port, cSPIN_BUSY_Pin)==Bit_RESET)
    {
      GPIO_SetBits(POWER_LED_Port, POWER_LED_Pin);
    }
    else
    {
      GPIO_ResetBits(POWER_LED_Port, POWER_LED_Pin);
    }
  }
  if(EXTI_GetITStatus(EXTI_Line11) != RESET)
  {
    /* Clear the EXTI line 11 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line11);
    if (GPIO_ReadInputDataBit(cSPIN_FLAG_Port, cSPIN_FLAG_Pin)==Bit_RESET)
    {
      GPIO_SetBits(STATUS_LED_Port, STATUS_LED_Pin);
    }
    else
    {
      GPIO_ResetBits(STATUS_LED_Port, STATUS_LED_Pin);
    }
  }
}
#endif /* defined(STEVAL_PCC009V2) */

#if defined(ST_CSPIN_6480H_DISCOVERY)
/**
  * @brief  This function handles External interrupt Line 0 request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    /* Clear the EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);
    GPIO_ResetBits(cSPIN_SW_Port, cSPIN_SW_Pin);
    cSPIN_Delay(0x00010000);
    GPIO_SetBits(cSPIN_SW_Port, cSPIN_SW_Pin);
  }
}

/**
  * @brief  This function handles External interrupt Line 1 request.
  * @param  None
  * @retval None
  */
void EXTI1_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
    /* Clear the EXTI line 1 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line1);
    /* Wait just in case the motor is accelerating or decelerating */
    while(cSPIN_Busy_HW());
    cSPIN_rxdata = cSPIN_Get_Status() & cSPIN_STATUS_MOT_STATUS;
    /* Accelerate if the motor is already running at constant speed */
    if (cSPIN_rxdata == cSPIN_STATUS_MOT_STATUS_CONST_SPD)
    {
      /* Read current speed parameter from cSPIN */
      cSPIN_rxdata = cSPIN_Get_Param(cSPIN_SPEED);
      /* Double the motor speed */
      cSPIN_Run(FWD, cSPIN_rxdata << 1);
    }
    /* Start the motor */
    else if (cSPIN_rxdata == cSPIN_STATUS_MOT_STATUS_STOPPED)
    {
      cSPIN_Run(FWD, Speed_Steps_to_Par(((uint16_t)(cSPIN_CONF_PARAM_MAX_SPEED))>>2));
      /* Warns the user that the motor is running by switching ON a LED */
      GPIO_SetBits(LED_SPARE_Port, LED_SPARE_Pin);
    }
    /* Unexpected motor status */
    else
    {
      assert_param(0);
    }  
  }
}

/**
  * @brief  This function handles External interrupt Line 2 request.
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  {
    /* Clear the EXTI line 2 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line2);
    /* Wait just in case the motor is accelerating or decelerating */
    while(cSPIN_Busy_HW());
    cSPIN_rxdata = cSPIN_Get_Status() & cSPIN_STATUS_MOT_STATUS;
    /* Decelerate if the motor is already running at constant speed */
    if (cSPIN_rxdata == cSPIN_STATUS_MOT_STATUS_CONST_SPD)
    {
      /* Read current speed parameter from cSPIN */
      cSPIN_rxdata = cSPIN_Get_Param(cSPIN_SPEED);
      if ((cSPIN_rxdata >> 1) > Speed_Steps_to_Par(cSPIN_CONF_PARAM_MIN_SPEED))
      {
        /* Halve the motor speed */
        cSPIN_Run(FWD, cSPIN_rxdata >> 1);
      }
      else
      {
        /* Disable the power bridges after a smooth stop */
        cSPIN_Soft_HiZ();
        /* Warns the user that the motor is stopped by switching off a LED */
        GPIO_ResetBits(LED_SPARE_Port, LED_SPARE_Pin);
      }
    }
    /* Unexpected motor status */
    else if (cSPIN_rxdata != cSPIN_STATUS_MOT_STATUS_STOPPED)
    {
      assert_param(0);
    }
  }
}

/**
  * @brief  This function handles External lines 15 to 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
    /* Clear the EXTI line 10 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line10);   
    if (GPIO_ReadInputDataBit(cSPIN_FLAG_Port, cSPIN_FLAG_Pin)==Bit_RESET)
    {
      GPIO_SetBits(STATUS_LED_Port, STATUS_LED_Pin);
    }
    else
    {
      GPIO_ResetBits(STATUS_LED_Port, STATUS_LED_Pin);
    }
  }
  if(EXTI_GetITStatus(EXTI_Line11) != RESET)
  {
    /* Clear the EXTI line 11 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line11);
    if (GPIO_ReadInputDataBit(cSPIN_BUSY_Port, cSPIN_BUSY_Pin)==Bit_RESET)
    {
      GPIO_SetBits(LED_BUSY_Port, LED_BUSY_Pin);
    }
    else
    {
      GPIO_ResetBits(LED_BUSY_Port, LED_BUSY_Pin);
    }
  }
}
#endif /* defined(ST_CSPIN_6480H_DISCOVERY) */

/** @} */  

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
