/**
  ******************************************************************************
  * @file main.c
  * @brief This file contains the main function for STL implementation example
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
#include "stm8_stl_lib.h"
#include "stm8_stl_classB_var.h"
#include "main.h"

/**
  * @addtogroup ClassBDemo
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Global variables ---------------------------------------------------------*/

#ifdef _COSMIC_
  uint8_t putchar (char c);
#endif /* _COSMIC_ */
#ifdef _IAR_
  int putchar (int c);
#endif /* _IAR_ */

/* Global inline BSP functions declaration -----------------------------------*/
#ifdef _IAR_
  extern void gpio_LED_init(void);
  extern void gpio_test_init(void);
  extern void toogle_test_pin(void);
  extern void BSP_LED_On(uint8_t led);
  extern void BSP_LED_Off(uint8_t led);
  extern void BSP_LED_Toogle(uint8_t led);
#endif /* _IAR_ */
/* Private functions ---------------------------------------------------------*/
/**
  * @brief Example firmware main entry point.
  * @par Parameters:
  * None
  * @retval
  * None
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * - InitClassB_Demo()
  * - STL_VerboseInit()
  * - STL_InitRunTimeChecks()
  * - STL_DoRunTimeChecks()
  * - LCD_SetCursorPos()
  * - LCD_Print()
  * - printf()
  */
void main(void)
{
  /* Initialize Watch Dogs */
  refresh_iwdog();              /* its default setting is used here - ~215ms timeout */ 
  refresh_wwdog(0x7fu, 0x7fu);
    
  #if defined(EVAL_BOARD_CONTROL)
    gpio_LED_init();
    gpio_test_init();
  #endif /* EVAL_BOARD_CONTROL */
       
  #ifdef DEBUG
    #ifdef STL_VERBOSE
      STL_VerboseInit();                /* init UART if not initialized yet */
    #endif /* STL_VERBOSE */
    #ifdef STL_VERBOSE_RUN
      printf("\n\r*** STM8S STL run time init ***\n\r");
    #endif  /* STL_VERBOSE_RUN */
  #endif /* DEBUG */
	           	
  enableInterrupts();
  
  /* init run time measurements */
  #ifdef STL_INCL_RUN
    STL_InitRunTimeChecks();
  #endif /* STL_INCL_RUN */

  #ifdef STL_INCL_HSECSS
    switch_clock_system(to_HSE);
    #ifdef STL_VERBOSE_RUN
      STL_VerboseInit(); 
    #endif  /* STL_VERBOSE_RUN */
  #endif /* STL_INCL_HSECSS */
    
  /* enable interrupts here to run time base */
  enableInterrupts();
    	
  #ifdef STL_VERBOSE_RUN
    /* Display the welcome text on first LCD line and on UART */
     printf("\n\r*** STM8S Self-Test Library Main ***\n\r");
  #endif /* STL_VERBOSE_RUN */
    
  #ifdef EVAL_BOARD_CONTROL
  #endif /* EVAL_BOARD_CONTROL */
      
  while (1)
  {
    #ifdef STL_INCL_RUN
      STL_DoRunTimeChecks();
    #else
      refresh_iwdog();                  /* For demo purposes, if hardware IWDG/WWDG is ON  */
      refresh_wwdog(0x7fu, 0x7fu);
    #endif /* STL_INCL_RUN */
		
    #ifdef EVAL_BOARD_CONTROL
    #endif /* EVAL_BOARD_CONTROL */
  }
}
#if defined(STL_VERBOSE_POR) || defined(STL_VERBOSE_RUN) || defined(STL_VERBOSE_FAILSAFE)
/* ---------------------------------------------------------------------------*/
/**
  * @brief Re-targets the C library printf function to the UART1 or UART2 for STM8S105x.
  * @param
  * char Character to send
  * @retval
  * char Character sent
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * None
  */
#ifdef _COSMIC_
uint8_t putchar (char c)
#endif /* _COSMIC_ */
#ifdef _IAR_
int putchar (int c)
#endif /* _IAR_ */
{
  /* put c to hardware here */
  #if defined(STM8S208) || defined(STM8S207) || defined(STM8S103) || defined(STM8S903)
    while ((UART1->SR & UART1_SR_TC) == 0)
    {}
    UART1->DR = (uint8_t)(c);
    while ((UART1->SR & UART1_SR_TC) == 0)
    {}
  #endif /* STM8S208, STM8S207, STM8S103, STM8S903 */
	 
  #if defined(STM8S105)
    while ((UART2->SR & UART2_SR_TC) == 0);
    UART2->DR = c;
    while ((UART2->SR & UART2_SR_TC) == 0);
  #endif /* STM8S105 */

  return (c);
}
#endif /* any_verbose_defined */


/* ---------------------------------------------------------------------------*/
#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
/**
  * @}
  */
/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/