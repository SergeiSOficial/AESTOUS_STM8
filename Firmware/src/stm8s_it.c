/**
  ******************************************************************************
  * @file stm8s_it.c
  * @brief This file contains all the interrupt routines, for Cosmic compiler.
  * @author STMicroelectronics - MCD Application Team
  * @version V2.0.0
  * @date Dec-2017
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8s_it.h"
#include "stm8_stl_lib.h"
#include "stm8_stl_classB_var.h"

#include "main.h"
#include "app.h"

/** @addtogroup IT_Functions
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile static uint16_t tmpCC1_last;     /* keeps the latest TIMx/CCR1 captured value */
volatile static uint16_t tmpCC1_last_cpy; /* keeps the last but one TIMx/CCR1 captured value */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

#ifdef _COSMIC_
/**
  * @brief Dummy interrupt routine
  * @param  None
  * @retval None
*/
@far @interrupt void NonHandledInterrupt(void)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /* _COSMIC_ */

/**
  * @brief TRAP interrupt routine
  * @param  None
  * @retval None
*/
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
/**
  * @brief  Top Level Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(TLI_IRQHandler, 0)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  Auto Wake Up Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief Clock Controller Interrupt routine.
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(CLK_IRQHandler, 2)
{
  fail_safe_assert(0x20u, "Clock Source failure (Clock Security System)");
  return;
}

/**
  * @brief External Interrupt PORTA Interrupt routine.
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief External Interrupt PORTB Interruption routine.
  * @param None
  * @retval
  * None
*/
INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief External Interrupt PORTC Interrupt routine.
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief External Interrupt PORTD Interrupt routine.
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief External Interrupt PORTE Interrupt routine.
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 7)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#ifdef STM8S903
/**
  * @brief External Interrupt PORTF Interrupt routine.
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(EXTI_PORTF_IRQHandler, 8)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /* STM8S903 */

#if defined(STM8S208) || defined(STM8AF52Ax)
/**
  * @brief CAN RX Interrupt routine.
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(CAN_RX_IRQHandler, 8)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief CAN TX Interrupt routine.
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(CAN_TX_IRQHandler, 9)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /* STM8S208, STM8AF52Ax */

/**
  * @brief SPI Interrupt routine.
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(SPI_IRQHandler, 10)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief Timer1 Update/Overflow/Trigger/Break Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief Timer1 Capture/Compare Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, 12)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

#ifdef STM8S903
/**
  * @brief Timer5 Update/Overflow/Break/Trigger Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(TIM5_UPD_OVF_BRK_TRG_IRQHandler, 13)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
/**
  * @brief Timer5 Capture/Compare Interrupt routine.
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(TIM5_CAP_COM_IRQHandler, 14)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

#else  /*STM8S208, STM8S207, STM8S105, STM8S103, STM8AF62Ax, STM8AF52Ax, STM8AF626x */
/**
  * @brief Timer2 Update/Overflow/Break Interrupt routine.
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief Timer2 Capture/Compare Interruption routine.
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(TIM2_CAP_COM_IRQHandler, 14)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /*STM8S903*/

#if defined(STM8S208) || defined(STM8S207) || defined(STM8S007) || defined(STM8S105) || \
    defined(STM8S005) || defined(STM8AF62Ax) || defined(STM8AF52Ax) || defined(STM8AF626x)
/**
  * @brief Timer3 Update/Overflow/Break Interrupt routine
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(TIM3_UPD_OVF_BRK_IRQHandler, 15)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief Timer3 Capture/Compare Interrupt routine.
  * @param None
  * @retval
  * None
  */
INTERRUPT_HANDLER(TIM3_CAP_COM_IRQHandler, 16)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /*STM8S208, STM8S207, STM8S007, STM8S105, STM8S005, STM8AF62Ax, STM8AF52Ax, STM8AF626x */

#if !defined(STM8S105) && !defined(STM8S005)
/**
  * @brief UART1 TX Interruption routine.
  * @param None
  * @retval None
  */
INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17)
{
  /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief UART1 RX Interruption routine.
  * @param None
  * @retval None
  */
INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
{
  /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
#endif /* STM8S105, STM8S005 */

/**
  * @brief I2C Interrupt routine
  * @parram None
  * @retval None
  */
INTERRUPT_HANDLER(I2C_IRQHandler, 19)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

#if defined(STM8S105) || defined(STM8S005) || defined(STM8AF626x)
/**
  * @brief UART2 TX interrupt routine.
  * @param None
  * @retval None
  */
INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20)
{
  /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief UART2 RX interrupt routine.
  * @param None
  * @retval None
  */
INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
{
  /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
#endif /* STM8S105, STM8S005, STM8AF626x */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined(STM8AF52Ax) || defined(STM8AF62Ax)
/**
  * @brief UART3 TX interrupt routine.
  * @param None
  * @retval None
  */
INTERRUPT_HANDLER(UART3_TX_IRQHandler, 20)
{
  /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief UART3 RX interrupt routine.
  * @param None
  * @retval None
  */
INTERRUPT_HANDLER(UART3_RX_IRQHandler, 21)
{
  /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
#endif /* STM8S208, STM8S207, STM8S007, STM8AF52Ax, STM8AF62Ax */

#if defined(STM8S207) || defined(STM8S007) || defined(STM8S208) || defined(STM8AF52Ax) || defined(STM8AF62Ax)
/**
  * @brief ADC2 interrupt routine.
  * @param None
  * @retval None
  */
INTERRUPT_HANDLER(ADC2_IRQHandler, 22)
{
  /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
#else  /*STM8S105, STM8S103, STM8S903, STM8AF626x */
/**
  * @brief ADC1 interrupt routine.
  * @param None
  * @retval None
  */
INTERRUPT_HANDLER(ADC1_IRQHandler, 22)
{
    ADC1_ClearITPendingBit(ADC1_IT_EOC);
    AppADCHandler();
  /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
#endif /* STM8S208, STM8S207, STM8S007, STM8AF52Ax, STM8AF62Ax */

#ifdef STM8S903
/**
  * @brief Timer6 Update/Overflow/Trigger Interruption routine.
  * @param None
  * @retval None
  */
INTERRUPT_HANDLER(TIM6_UPD_OVF_TRG_IRQHandler, 23)
{
  TIM6->SR1 = ~TIM6_SR1_UIF;

#else  /* STM8S208, STM8S207, STM8S007, STM8S105, STM8S103, STM8AF62Ax, STM8AF52Ax, STM8AF626x */
/**
  * @brief Timer4 Update/Overflow/Trigger Interruption routine.
  * @param None
  * @retval None
  */
INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{
  TIM4->SR1 = ~TIM4_SR1_UIF;
#endif /* STM8S903 */
#if defined(STL_INCL_RUN_RAM) && defined(STL_INCL_RUN)
  /* Verify TickCounter integrity */
  if ((TickCounter ^ TickCounterInv) == 0xFFFFu)
  {
    TickCounter++;
    TickCounterInv = ~TickCounter;

    if (TickCounter >= TEST_TIMEBASE)
    {
      ClassBTestStatus RamTestResult = TEST_RUNNING;

#if defined(EVAL_BOARD_CONTROL)
      BSP_LED_Toogle(LED_VLM);
#endif /* EVAL_BOARD_CONTROL */

      /* Reset timebase counter */
      TickCounter = 0u;
      TickCounterInv = 0xFFFFu;

      /* Set Flag read in main loop */
      TimeBaseFlag = 0xAAu;
      TimeBaseFlagInv = 0x55u;

#ifdef STL_INCL_RUN_RAM
      ISRCtrlFlowCnt += RAM_MARCH_ISR_CALLER;
      RamTestResult = STL_TranspMarch();
      ISRCtrlFlowCntInv -= RAM_MARCH_ISR_CALLER;
#else
      RamTestResult = TEST_RUNNING;
#endif /* STL_INCL_RUN_RAM */

      switch (RamTestResult)
      {
      case TEST_RUNNING:
        break;
      case TEST_OK:
#ifdef STL_VERBOSE_RUN
        putchar('#'); /* RAM check is finished OK */
#endif                /* STL_VERBOSE_RUN */
#if defined(EVAL_BOARD_CONTROL)
        BSP_LED_Toogle(LED_VLM);
#endif /* EVAL_BOARD_CONTROL */
        break;
      case TEST_FAILURE:
      case CLASS_B_DATA_FAIL:
      default:
        fail_safe_assert(0x21u, "RAM Error (March Run-time check)");
        break;
      } /* End of the switch */

      /* Do we reached the end of RAM test? */
      /* Verify 1st ISRCtrlFlowCnt integrity */
      if ((ISRCtrlFlowCnt ^ ISRCtrlFlowCntInv) == 0xFFFFu)
      {
        if (RamTestResult == TEST_OK)
        {
          if (ISRCtrlFlowCnt != RAM_TEST_COMPLETED)
          {
            fail_safe_assert(0x22u, "Control Flow Error (March Run-time check)");
          }
          else /* Full RAM was scanned */
          {
            ISRCtrlFlowCnt = 0u;
            ISRCtrlFlowCntInv = 0xFFFFu;
          }
        } /* End of RAM completed if */
      }   /* End of control flow monitoring */
      else
      {
        fail_safe_assert(0x23u, "Control Flow Error (ISR) \n\r");
      }
#if defined(EVAL_BOARD_CONTROL)
      BSP_LED_Toogle(LED_VLM);
#endif /* EVAL_BOARD_CONTROL */
    }  /* End of the 20 ms timebase interrupt */
  }
  else /* Class error on TickCounter */
  {
    fail_safe_assert(0x24u, "Class B Error on TickCounter");
  }

#endif /* (STL_INCL_RUN_RAM || STL_INCL_RUN_FLASH) && STL_INCL_RUN */

  return;
}

/**
  * @brief Eeprom EEC Interruption routine.
  * @param None
  * @retval None
  */
INTERRUPT_HANDLER(EEPROM_EEC_IRQHandler, 24)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/