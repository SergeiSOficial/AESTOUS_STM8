/**
  ******************************************************************************
  * @file stm8_stl_main.c
  * @brief Contains the Self-test functions executed during main
  *       program execution.
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
ErrorStatus STL_CheckStack(void);
void STL_TimeBaseInit(void);
uint16_t debug_count;

extern const uint8_t __checksum_begin;
extern const uint8_t __checksum_end;


#ifdef STL_INCL_RUN
/* Private functions ---------------------------------------------------------*/
/**
  * @brief Initializes the Class B variables and their inverte
  *        redundant counterparts. Init also the Systick and RTC timer
  *        for clock frequency monitoring.
  * @par Parameters:
  * None
  * @retval
  * None
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * - STL_TranspMarchCInit()
  * - STL_TranspMarchXInit()
  * - STL_FlashCrcInit()
  * - STL_InitClock_Xcross_Measurement()
  * - FailSafe()
  * - printf()
  * - STL_TimeBaseInit()
  */
void STL_InitRunTimeChecks(void)
{
  #if defined (EVAL_BOARD_CONTROL)
    toogle_test_pin();
  #endif /* EVAL_BOARD_CONTROL */

  /* Init Class B variables required in main routine and TIM4 Timebase interrupt
  service routine for timing purposes */
  TickCounter = 0u;
  TickCounterInv = 0xFFFFu;

  TimeBaseFlag = 0u;
  TimeBaseFlagInv = 0xFFu;

  LastCtrlFlowCnt = 0u;
  LastCtrlFlowCntInv = 0xFFFFu;

  /* Initialize variables for SysTick interrupt routine control flow monitoring */
  ISRCtrlFlowCnt = 0u;
  ISRCtrlFlowCntInv = 0xFFFFu;

  /* Initialize variables for invariable memory check */
  CtrlFlowCnt = 0u;
  CtrlFlowCntInv = 0xFFFFu;

  #ifndef STL_INCL_POR
    LSI_HSIStartUpFreqkHz = 0u;
    LSI_HSEStartUpFreqkHz = 0u;
  #endif  /* STL_INCL_POR */

  #ifdef STL_INCL_RUN_RAM
    CtrlFlowCnt = RAM_MARCH_INIT_CALLER;
    STL_TranspMarchInit();
    CtrlFlowCntInv -= RAM_MARCH_INIT_CALLER;
  #endif /* STL_INCL_RUN_RAM */

  /* Initialize variables for invariable memory check */
  #ifdef STL_INCL_RUN_FLASH
    #ifndef CRC_CHECK_8
      STL_FlashCrc16Init();
    #else
      STL_FlashCrc8Init();
    #endif /* CRC_CHECK_8 */
  #endif /* STL_INCL_RUN_FLASH */

  /* Initializes TIM3 to measure indirectly HSE frequency */
  #ifdef STL_INCL_RUN_CLOCK
    CtrlFlowCnt += LSI_CHECK_INIT_CALLER;

    if (STL_InitClock_Xcross_Measurement() == ERROR)
    {
      fail_safe_assert( 0x10u, "Abnormal Clock Test routine termination (main init)");
    }

    CtrlFlowCntInv -= LSI_CHECK_INIT_CALLER;
  #endif /* STL_INCL_RUN_CLOCK */

  #ifdef STL_INCL_RUN_STACK
    /* For stack overflow detection function */
    aStackOverFlowPtrn[0] = 0xEEu;
    aStackOverFlowPtrn[1] = 0xBBu;
    aStackOverFlowPtrn[2] = 0xDDu;
    aStackOverFlowPtrn[3] = 0xCCu;
  #endif /* STL_INCL_RUN_STACK */

  /* Initialize timer to provide main System timebase */
  #if defined(STL_INCL_RUN_RAM) || defined(STL_INCL_RUN_FLASH)
    CtrlFlowCnt += TIM_BASE_INIT_CALLER;
    STL_TimeBaseInit();
    CtrlFlowCntInv -= TIM_BASE_INIT_CALLER;
  #endif /* STL_INCL_RUN_RAM || STL_INCL_RUN_FLASH */

  /* Check that all initialization routines were correctly executed */
  if (((CtrlFlowCnt ^ CtrlFlowCntInv) != 0xFFFFu)
     || (CtrlFlowCnt != CHECKPOINT_INIT ))
  {
    fail_safe_assert( 0x11u, "Control Flow Error (Main init)");
  }
  else
  {
    #ifdef STL_VERBOSE_RUN
      printf("Control Flow OK (Main init)\n\r");
    #endif  /* STL_VERBOSE_RUN */
    /* Initialize variables for main routine control flow monitoring */
    CtrlFlowCnt = 0u;
    CtrlFlowCntInv = 0xFFFFu;
  }

  #if defined (EVAL_BOARD_CONTROL)
    toogle_test_pin();
  #endif /* EVAL_BOARD_CONTROL */
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief Performs run time cpu, stack, clock, partial flash and flow control checks
  *        refresh window and independent watch dogs
  * @par Parameters:
  * None
  * @retval
  * None
  * @par Required preconditions:
  * STL_InitRunTimeChecks()
  * @par Library functions called:
  * - STL_RunTimeCPUTest()
  * - STL_CheckStack()
  * - STL_ClockFreqTest()
  * - STL_crc16Run()
  * - FailSafe()
  * - printf()
  */
void STL_DoRunTimeChecks(void)
{
  /* Is the time base duration elapsed? */
  if (TimeBaseFlag == 0xAAu)
  {
    #if defined (EVAL_BOARD_CONTROL)
      BSP_LED_Toogle(LED_NVM);
    #endif /* EVAL_BOARD_CONTROL */

    uint8_t tempTimeBaseFlagInv = TimeBaseFlagInv; // for fix undefined behavior: the order of volatile accesses is undefined in this statement
    /* Verify Time base flag integrity (class B variable) */
    if ((TimeBaseFlag ^ tempTimeBaseFlagInv) == 0xFFu)
    {
      ClassBTestStatus RomTest;
      /* Reset Flag (no need to reset the redundant: it is not tested if
      TimeBaseFlag != 0xAA when time base is elapsed */
      TimeBaseFlag = 0u;

      /*----------------------------------------------------------------------*/
      /*---------------------------- CPU registers ----------------------------*/
      /*----------------------------------------------------------------------*/
      #ifdef STL_INCL_RUN_CPU
        CtrlFlowCnt += CPU_RUN_CALLER;
        if (STL_RunTimeCPUTest() != CPUTEST_SUCCESS)
        {
          fail_safe_assert( 0x12u, "Run Time CPU Test Failure");
        }
        else
        {
          CtrlFlowCntInv -= CPU_RUN_CALLER;
        }
      #endif /* STL_INCL_RUN_CPU */

      /*----------------------------------------------------------------------*/
      /*------------------------- Stack overflow -----------------------------*/
      /*----------------------------------------------------------------------*/
      #ifdef STL_INCL_RUN_STACK
        CtrlFlowCnt += STACK_OVERFLOW_CALLER;
        if (STL_CheckStack() != SUCCESS)
        {
          fail_safe_assert( 0x13u, "Stack overflow");
        }
        else
        {
          CtrlFlowCntInv -= STACK_OVERFLOW_CALLER;
        }
      #endif /* STL_INCL_RUN_STACK */
      /*----------------------------------------------------------------------*/
      /*------------------------- Clock monitoring ---------------------------*/
      /*----------------------------------------------------------------------*/

      #ifdef STL_INCL_RUN_CLOCK
        CtrlFlowCnt += FREQ_TEST_CALLER;

        switch (STL_ClockFreqTest())
        {
          case FREQ_OK:
          case TEST_ONGOING:
            CtrlFlowCntInv -= FREQ_TEST_CALLER;
            break;

          case EXT_SOURCE_FAIL:
            fail_safe_assert( 0x14u, "EXT clock frequency error (clock test)");
            break;

          case HSI_SOURCE_FAIL:
            fail_safe_assert( 0x15u, "Syst clock frequency error (clock test)");
            break;

          case CLASS_B_VAR_FAIL:
            fail_safe_assert( 0x16u, "Class B variable error (clock test)");
            break;

          case LSI_START_FAIL:
          case HSE_START_FAIL:
          case HSI_HSE_SWITCH_FAIL:
          case CTRL_FLOW_ERROR:
          default:
            fail_safe_assert( 0x17u, "Abnormal Clock Test routine termination (main)");
            break;
        }
      #endif /* STL_INCL_RUN_CLOCK */

      /*----------------------------------------------------------------------*/
      /*------------------ Invariable memory CRC check -----------------------*/
      /*----------------------------------------------------------------------*/
      #ifdef STL_INCL_RUN_FLASH
        CtrlFlowCnt += FLASH_RUN_TEST_CALLER;
        RomTest = STL_crc16Run();
        switch ( RomTest )
        {
          case TEST_RUNNING:
            CtrlFlowCntInv -= FLASH_RUN_TEST_CALLER;
            break;

          case TEST_OK:
            #if defined (EVAL_BOARD_CONTROL)
              BSP_LED_Toogle(LED_NVM);
            #endif /* EVAL_BOARD_CONTROL */
            #ifdef STL_VERBOSE_RUN
              putchar('*');             /* Flash check finished OK */
            #endif /* STL_VERBOSE_RUN */
            CtrlFlowCntInv -= FLASH_RUN_TEST_CALLER;
            break;

          case TEST_FAILURE:
          case CLASS_B_DATA_FAIL:
          case CTRL_FLW_ERROR:
          default:
            fail_safe_assert( 0x18u, " Run-time FLASH CRC Error\n\r");
            break;
        }
      #else /* test result ignored */
        RomTest = TEST_OK;
      #endif /* STL_INCL_RUN_FLASH */

      /*----------------------------------------------------------------------*/
      /*---------------- Check Safety routines Control flow  -----------------*/
      /*------------- Refresh Window and independent watchdogs ---------------*/
      /*----------------------------------------------------------------------*/
      if (((CtrlFlowCnt ^ CtrlFlowCntInv) == 0xFFFFu)
      && ((LastCtrlFlowCnt ^ LastCtrlFlowCntInv) == 0xFFFFu))
      {
        if (RomTest == TEST_OK)
        {
        #ifdef _IAR_
          if ((CtrlFlowCnt == FULL_FLASH_CHECKED) \
          && ((CtrlFlowCnt - LastCtrlFlowCnt) == (LAST_DELTA_MAIN)))
        #endif /* _IAR_ */
        #ifdef _COSMIC_
          if ((CtrlFlowCnt - LastCtrlFlowCnt) == (LAST_DELTA_MAIN))
        #endif /* _COSMIC_ */
          {
            CtrlFlowCnt = 0u;
            CtrlFlowCntInv = 0xFFFFu;
          }
          else  /* Return value form crc check was corrupted */
          {
            fail_safe_assert( 0x19u, "Control Flow Error (main loop, Flash CRC)");
          }
        }
        else  /* Flash test not completed yet */
        {
          if ((CtrlFlowCnt - LastCtrlFlowCnt) != DELTA_MAIN)
          {
            fail_safe_assert( 0x1Au, "Control Flow Error (main loop, Flash CRC on-going)");
          }
        }

        LastCtrlFlowCnt = CtrlFlowCnt;
        LastCtrlFlowCntInv = CtrlFlowCntInv;
      }
      else
      {
        fail_safe_assert( 0x1Bu, "Control Flow Error (main loop)");
      }
    } /* End of periodic Self-test routine */
    else  /* Class B variable error (can be Systick interrupt lost) */
    {
      fail_safe_assert( 0x1Cu, "Class B variable error (time base check)");
    }

    #if defined (EVAL_BOARD_CONTROL)
      toogle_test_pin();
    #endif /* EVAL_BOARD_CONTROL */

    /* Reload WWDG and IWDG counters */
    #if !defined(STM8L10X)  && !defined(STM8TL5X)
      refresh_wwdog(WWDG_PERIOD, WWDG_WINDOW);
    #endif /* !STM8L10X && !STM8TL5X */
    refresh_iwdog();

    #if defined (EVAL_BOARD_CONTROL)
      toogle_test_pin();
      BSP_LED_Toogle(LED_NVM);
    #endif /* EVAL_BOARD_CONTROL */
  } /* End of periodic Self-test routine */
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief This function verifies that Stack didn't overflow
  * @par Parameters:
  * None
  * @retval
  * ErrorStatus = {ERROR; SUCCESS}
  * @par Required preconditions:
  * STL_InitRunTimeChecks()
  * @par Library functions called:
  * None
  */
ErrorStatus STL_CheckStack(void)
{
    ErrorStatus result = ERROR;

  CtrlFlowCnt += STACK_OVERFLOW_CALLEE;

  if (aStackOverFlowPtrn[0] != 0xEEu)
  {
    result = ERROR;
  }
  else /* aStackOverFlowPtrn[0] == 0xEE */
  {
    if (aStackOverFlowPtrn[1] != 0xBBu)
    {
      result = ERROR;
    }
    else /* aStackOverFlowPtrn[1] == 0xBB */
    {
      if (aStackOverFlowPtrn[2] != 0xDDu)
      {
        result = ERROR;
      }
      else /* aStackOverFlowPtrn[2] == 0xDD */
      {
        if (aStackOverFlowPtrn[3] != 0xCCu)
        {
          result = ERROR;
        }
        else
        {
          result = SUCCESS;
        }
      }
    }
  }

  CtrlFlowCntInv -= STACK_OVERFLOW_CALLEE;

  return (result);

}

/* ---------------------------------------------------------------------------*/
/**
  * @brief This function initializes the TIM4 to provide a System Tick
  * @par Parameters:
  * None
  * @retval
  * None
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * None
  */
void STL_TimeBaseInit(void)
{
  CtrlFlowCnt += TIM_BASE_INIT_CALLEE;

  /* Time base configuration */
#ifdef STM8S903
  CLK->PCKENR1 |= CLK_PCKENR1_TIM6;      /* enable clock to TIM6 */
  TIM6->PSCR = (uint8_t)(7u & TIM6_PSCR_PSC); /* prescaler syst clock/128 */
  #ifdef STL_INCL_HSECSS
    TIM6->ARR = (uint8_t)187;                 /* auto reload gives 1ms period @ 187,5kHz (24MHz/128) */
  #else
    TIM6->ARR = (uint8_t)124;                 /* auto reload gives 1ms period @ 125kHz (16MHz/128) */
  #endif /* STL_INCL_HSECSS */
  TIM6->IER |= TIM6_IER_UIE;             /* enable TIM6 update interrupt */
  ITC->ISPR6 |= (uint8_t)(1u << 6);           /* set the highest priority level for TIM6 due to RAM transparent test */
  TIM6->CR1 |= TIM6_CR1_CEN;             /* counter enable */
#else /* TIM4 applied */
  #ifndef STM8L10X
    CLK->PCKENR1 |= CLK_PCKENR1_TIM4;      /* enable clock to TIM4 */
  #else
    CLK->PCKENR |= CLK_PCKENR_TIM4;
  #endif /* STM8L10X */
  TIM4->PSCR = (uint8_t)(7u & TIM4_PSCR_PSC); /* prescaler syst clock/128 */
  #ifdef STL_INCL_HSECSS
    TIM4->ARR = (uint8_t)187;	         /* auto reload gives 1ms period @ 187,5kHz (24MHz/128) */
  #else
    TIM4->ARR = (uint8_t)124;                 /* auto reload gives 1ms period @ 125kHz (16MHz/128) */
  #endif /* STL_INCL_HSECSS */
  TIM4->IER |= TIM4_IER_UIE;             /* enable TIM4 update interrupt */
  ITC->ISPR6 |= (uint8_t)(1u << 6);           /* set the highest priority level for TIM4 due to RAM transparent test */
  TIM4->CR1 |= TIM4_CR1_CEN;             /* counter enable */
#endif /* STM8S903 */

  CtrlFlowCntInv -= TIM_BASE_INIT_CALLEE;
}
#endif /* STL_INCL_RUN */
/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/