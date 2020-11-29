/**
  ******************************************************************************
  * @file stm8_stl_clockstart.c
  * @brief This file contains the test function for clock circuitry and wrong
   *       frequency detection at start-up.
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
ErrorStatus STL_LSIinit(void);
uint16_t calc_captured_value(void);
#ifdef STL_INCL_HSECSS
  ErrorStatus STL_HSE_CSSinit(void);
#endif /* STL_INCL_HSECSS */

/* Private functions ---------------------------------------------------------*/
/**
  * @brief Start up the internal and external oscillators and verifies that
  *        clock source is within the expected range
  * @par Parameters:
  * None
  * @retval
  * ClockStatus {LSI_START_FAIL, HSE_START_FAIL, HSI_HSE_SWITCH_FAIL, TEST_ONGOING,
  *              EXT_SOURCE_FAIL, CLASS_B_VAR_FAIL, CTRL_FLOW_ERROR, FREQ_OK}
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * - STL_LSIinit()
  * - STL_HSE_CSSinit()
  * - STL_GetLSI_Freq()
  */
ClockStatus STL_ClockStartUpTest(void)
{
  ClockStatus clck_sts = TEST_ONGOING; /* In case of unexpected exit */
  uint16_t lsi_period;
  uint16_t expected_value;

  CtrlFlowCnt += CLOCK_POR_CALLEE;

  /* Start low speed internal (LSI) oscillator */
  if (STL_LSIinit() != SUCCESS)
  {
    clck_sts = LSI_START_FAIL;
  }
  if (clck_sts == TEST_ONGOING)
  {
    /* Configure dedicated timer to measure LSI period */
    if (STL_InitClock_Xcross_Measurement() == ERROR)
    {
      clck_sts = XCROSS_CONFIG_FAIL;
    }
    else
    {
      /* measure two consequent capture events */
      lsi_period = STL_MeasureLSIPeriod();
      expected_value = calc_captured_value();

      /*-------------------- HSI measurement check -------------------------*/
      if (lsi_period < (expected_value * 4u / 5u))
      {
        /* HSI -20% below expected */
        clck_sts = HSI_SOURCE_FAIL;
      }
      else if (lsi_period > (expected_value * 6u / 5u))
      {
        /* HSI +20% above expected */
        clck_sts = HSI_SOURCE_FAIL;
      }

    #ifndef STL_INCL_HSECSS
      else
      {
        clck_sts = FREQ_OK;   /* HSI keeps expected frequency */
      }
    }
    #else /* test continues by standard test of HSE clock */
    }
    /* Start High-speed external oscillator (HSE) and enable CSS */
    if (clck_sts == TEST_ONGOING)
    {
      if (STL_HSE_CSSinit() != SUCCESS)
      {
        clck_sts = HSE_START_FAIL;
      }
      else
      {
        /* if HSE started OK - switch system to HSE */
        if (switch_clock_system(to_HSE) != SUCCESS)
        {
          clck_sts = HSI_HSE_SWITCH_FAIL;
        }
        #ifdef STL_VERBOSE_POR
          STL_VerboseInit();
        #endif  /* STL_VERBOSE_POR */
      }
    }

    /*-------------- Start Reference Measurement -------------------------------*/
    if (clck_sts == TEST_ONGOING)
    {
      /* measure two consequent capture events */
      lsi_period = STL_MeasureLSIPeriod();
      expected_value = calc_captured_value();

      /*-------------------- HSE measurement check -------------------------*/
      if (lsi_period < (expected_value * 3u / 4u))
      {
        /* Sub-harmonics: HSE -25% below expected */
        clck_sts = EXT_SOURCE_FAIL;
      }
      else if (lsi_period > (expected_value * 5u / 4u))
      {
        /* Harmonics: HSE +25% above expected */
        clck_sts = EXT_SOURCE_FAIL;
      }
      else
      {
        clck_sts = FREQ_OK;   /* Crystal or Resonator started correctly, with expected frequency */
      }                     /* No harmonics/sub-harmonics */
    }
    #endif /* STL_INCL_HSECSS */
  }

  CtrlFlowCntInv -= CLOCK_POR_CALLEE;
  return(clck_sts);
}

/* -------------------------------------------------------------------------- */
/**
  * @brief Switch to selected clock using automatic mode.
  * @par Parameters:
  * clck: to_HSE, to_LSI, to_HSI
  * @retval
  * ErroStatus (ERROR,SUCCESS)
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * None
  */
#ifdef STL_INCL_HSECSS
ErrorStatus switch_clock_system(uint8_t clck)
{
  uint16_t time_out = CLK_SWITCH_TIMEOUT;
  ErrorStatus result = SUCCESS;

  if (CLK->SWR != clck)
  {
    CLK->SWCR &= (uint8_t)(~CLK_SWCR_SWIF);			    /* clear SWIF flag */
    CLK->SWCR |= CLK_SWCR_SWEN;	                /* enable clock switching control */
    CLK->SWR = clck;										        /* initiate automatic switch mode */
    #if defined (STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD)
      while (((CLK->SWCR & CLK_SWCR_SWBSY) != 0u)  &&  (time_out != 0u))
    #else
      while (((CLK->SWCR & CLK_SWCR_SWIF) == 0u)  &&  (time_out != 0u))
    #endif  /* STM8L15xx */
      {
        --time_out;
      }
      if (time_out == 0u)
      {
        result =  ERROR;
      }
  }
  return result;
}
#endif /* STL_INCL_HSECSS */
/* ---------------------------------------------------------------------------*/
/**
  * @brief Start-up the low speed internal RC oscillator (F ~128 kHz)
  * @par Parameters:
  * None
  * @retval
  * ErrorStatus = {ERROR; SUCCESS}
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * None
  */
ErrorStatus STL_LSIinit(void)
{
  ErrorStatus result = SUCCESS;
  #if !defined(STM8L10X) && !defined(STM8TL5X)
    uint16_t time_out = LSI_START_TIMEOUT;
  #endif /* !STM8L10X && !STM8TL5X */

  CtrlFlowCnt += LSI_INIT_CALLEE;

  /* Enable LSI */
  #if !defined(STM8L10X) && !defined(STM8TL5X)
    #if defined (STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD)
      CLK->ICKCR |= CLK_ICKCR_LSION;
    #else /* STM8AS */
      CLK->ICKR |= CLK_ICKR_LSIEN;
    #endif /* STM8L15xx */

    /* Wait till LSI is ready */
    #if defined (STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD)
      while (((CLK->ICKCR & CLK_ICKCR_LSIRDY) == 0u) && (time_out != 0u))
    #else /* STM8AS */
      while (((CLK->ICKR & CLK_ICKR_LSIRDY) == 0u) && (time_out != 0u))
    #endif  /* STM8L15xx */
      {
        time_out--;
      }
      if (time_out == 0u)
      {
        result = ERROR;     /* Internal low speed oscillator failure */
      }
  #else /* case of STM8L 8K */
    #ifdef STM8L10X
      CLK->PCKENR |= (u8)(CLK_PCKENR_AWU);
    #endif /* STM8L10X */
    #ifdef STM8TL5X
      CLK->PCKENR1 |= (u8)(CLK_PCKENR1_AWU);
    #endif /* STM8TL15X */
  #endif /* !STM8L10X && !STM8TL5X */

  CtrlFlowCntInv -= LSI_INIT_CALLEE;

  return (result);
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief Start-up the high speed external oscillator (based on crystal or resonator), enable the clock security system
  * @par Parameters:
  * None
  * @retval
  * ErrorStatus = {ERROR; SUCCESS}
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * None
  */
#ifdef STL_INCL_HSECSS
ErrorStatus STL_HSE_CSSinit(void)
{
  ErrorStatus result = SUCCESS;
  uint16_t time_out = HSE_START_TIMEOUT;

  CtrlFlowCnt += HSE_INIT_CALLEE;

  /* Start-up the oscillator (HSE: High-speed External) */
  #if defined (STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD)
    CLK->ECKCR |= CLK_ECKCR_HSEON;
  #else
    CLK->ECKR |= CLK_ECKR_HSEEN;
  #endif  /* STM8L15xx */

  /* Wait till HSE is ready */
  #if defined (STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD)
    while (((CLK->ECKCR & CLK_ECKCR_HSERDY) == 0u) && (time_out != 0u))
  #else
    while (((CLK->ECKR & CLK_ECKR_HSERDY) == 0u) && (time_out != 0u))
  #endif  /* STM8L15xx */
    {
      --time_out;
    }

  if (time_out == 0u)
  {
    result = ERROR;     /* Internal low speed oscillator failure */
  }
  else
  {
    /* Enable the Clock Security System (CSS): any failure on HSE will cause
       an Interrupt and switch back to internal oscillator */
    /* At this time the CPU clock source is still the internal oscillator */
    enableInterrupts();
    CLK->CSSR |= CLK_CSSR_CSSDIE;     /* CSS detection interrupt enable */
    CLK->CSSR |= CLK_CSSR_CSSEN;      /* CSS enable */
  }
  #if defined(STM8S208) || defined(STM8S207)
    /* if no wait state is set at OPT7 and 24MHz XTAL is applied, clock divider has to keep syst fcy below 16 MHz) */
    if ((HSE_VALUE == 24000000uL) && ((OPT->OPT7 & 1u) == 0u) && ((CLK->CKDIVR & CLK_CKDIVR_CPUDIV) == 0u))
    {
      CLK->CKDIVR = (1u & CLK_CKDIVR_CPUDIV);
    }
  #endif /* STM8S208 STM8S207 */
  CtrlFlowCntInv -= HSE_INIT_CALLEE;

  return (result);
}
#endif /* STL_INCL_HSECSS */

 /* ---------------------------------------------------------------------------*/
/**
  * @brief Calculates expected captured value [counter ticks] based on CLK regs setting
  * @par Parameters:
  * None
  * @retval
  * uint8_t expected capture valuse
  * @par Required preconditions:
  * None
  * @par Library functions called:
  */
uint16_t calc_captured_value(void)
{
  uint16_t capt_val;

  #if !defined(STM8L10X) && !defined(STM8TL5X)
    #if defined (STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD)
      switch ( CLK->SCSR )
      {
        case 0x01:
          capt_val = (uint16_t)(((4u * HSI_VALUE) / LSI_VALUE) >> ((CLK->CKDIVR & CLK_CKDIVR_CKM) >> 3u));
          break;
        case 0x04:
          capt_val = (uint16_t)((4u * HSE_VALUE) / LSI_VALUE);
          break;
        case 0x08:
          capt_val = (uint16_t)((4u * LSE_VALUE) / LSI_VALUE);
          break;
        default:
          capt_val = 4u;                 /* invalid value in SCSR will generate HW reset */
          break;
      }
    #else /* case of STM8AS */
      switch ( CLK->CMSR )
      {
        case 0xE1:
          capt_val = (uint16_t)(((4u * HSI_VALUE) / LSI_VALUE) >> ((CLK->CKDIVR & CLK_CKDIVR_HSIDIV) >> 3u));
          break;
        case 0xB4:
          capt_val = (uint16_t)((4u * HSE_VALUE) / LSI_VALUE);
          break;
        default:
          capt_val = 4u;                 /* invalid value in CMSR will generate HW reset */
          break;
      }
    #endif  /* STM8L15xx */
  #else /* caae of STM8L 8K */
    capt_val = (uint16_t)((4u * HSI_VALUE / LSI_VALUE) >> (CLK->CKDIVR & CLK_CKDIVR_HSIDIV));
  #endif /* !STM8L10X && !STM8TL5X */

  #if defined(EVAL_BOARD_LCD)
    LSI_Calculated = capt_val;
  #endif /* EVAL_BOARD_LCD */

  return (capt_val);
}
/*---------------------------------------------------------------------------*/
/**
  * @brief Configure TIMx to measure LSI period
  * @param  : None
  * @retval : ErrorStatus = (ERROR, SUCCESS)
  */
ErrorStatus STL_InitClock_Xcross_Measurement(void)
{
  uint16_t time_out = FREQ_MEAS_TIMEOUT;
  ErrorStatus sts = SUCCESS;

  CtrlFlowCnt += XCLK_MEASURE_INIT_CALLEE;

  #if defined (STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD) || defined (STM8L10X) || defined (STM8TL5X)
    /* Start-up BEEP on LSI clock frequency */
    /* enable clock to TIM2 */
    #ifdef STM8L10X
      CLK->PCKENR |= (u8)(CLK_PCKENR_TIM2 | CLK_PCKENR_AWU);		/* STM8L10x */
      /* LSI clock connected to TIM2 Input Capture 1 to measure LSI frequency */
      AWU->CSR |= AWU_CSR_MSR;
    #elif defined(STM8TL5X)
      CLK->PCKENR1 |= (u8)(CLK_PCKENR1_TIM2 | CLK_PCKENR1_AWU);  /* STM8TL5x */
      /* LSI clock connected to TIM2 Input Capture 1 to measure LSI frequency */
      AWU->CSR |= AWU_CSR_MSR;
    #else /* case of STM8L 32K */
      CLK->PCKENR1 |= (u8)(CLK_PCKENR1_TIM2 | CLK_PCKENR1_BEEP); /* STM8L15x */
      /* LSI clock connected to TIM2 Input Capture 1 to measure LSI frequency */
      CLK->CBEEPR = (1u << 1u);
      while(((CLK->CBEEPR & CLK_CBEEPR_BEEPSWBSY) != 0u)  &&  (time_out != 0u))
      {
        --time_out;
      }
      /* LSI clock connected to TIM2 Input Capture 1 to measure LSI frequency */
      BEEP->CSR1 |= BEEP_CSR1_MSR;
    #endif /* STM8Lxxx */

    /* Enable capture of 4 events on TIM2_CC1 */
    TIM2->PSCR = 0u;                       /* init divider register /1	*/
    TIM2->ARRH = 0xffu;			   /* init ARR & OC1 compare registers */
    TIM2->ARRL = 0xffu;
    TIM2->CNTRH = 0xffu;
    TIM2->CNTRL = 0xffu;

    /* CC1 input capture, no filter, divider= 4 */
    TIM2->CCMR1 &= (uint8_t)(~(TIM_CCMR_ICxPSC | TIM_CCMR_CCxS));
    TIM2->CCMR1 |= ((2u << 2) & TIM_CCMR_ICxPSC) | (1u & TIM_CCMR_CCxS);

    TIM2->CCER1 = TIM_CCER1_CC1E; 			/* CC1 IC enable, rising edge */
    TIM2->CR1 |= (uint8_t)(TIM_CR1_URS | TIM_CR1_CEN); 	/* enable timer */

    /* wait for next capture event */
    while (((TIM2->SR1 & TIM_SR1_CC1IF) != TIM_SR1_CC1IF) && (time_out != 0u))
    {
      --time_out;
    }
  #endif /* STM8L15X_MD || STM8L15X_MDP || STM8L15X_HD || STM8L10X || STM8TL5X*/

  #if defined(STM8S208) || defined(STM8S207) || defined(STM8S105)
    /* enable clock to TIM3 */
    CLK->PCKENR1 |= (CLK_PCKENR1_TIM3);

    /* LSI clock connected to TIM3 Input Capture 1 to measure LSI frequency */
    AWU->CSR |= AWU_CSR_MSR;

    /* Enable capture of 4 events on TIM3_CC1 */
    TIM3->PSCR = 0u;                                    /* init divider register /1 */
    TIM3->ARRH = 0xffu;			                /* init ARR & OC1 compare registers */
    TIM3->ARRL = 0xffu;
    TIM3->CNTRH = 0xffu;
    TIM3->CNTRL = 0xffu;
    TIM3->CCMR1 &= (uint8_t)(~(TIM3_CCMR_ICxPSC | TIM3_CCMR_CCxS));
    TIM3->CCMR1 |= ((2u << 2) & TIM3_CCMR_ICxPSC) | (1u & TIM3_CCMR_CCxS);
    /* CC1 input capture, no filter, divider= 4 */
    TIM3->CCER1 = TIM3_CCER1_CC1E; 		        /* CC1 IC enable, rising edge */
    TIM3->CR1 |= (uint8_t)(TIM3_CR1_URS | TIM3_CR1_CEN);      /* enable timer */
    /* wait for next capture event */
    while (((TIM3->SR1 & TIM3_SR1_CC1IF) != TIM3_SR1_CC1IF) && (time_out != 0u))
    {
      --time_out;
    }
  #endif /* STM8S208 || STM8S207 || STM8S105 */

  #if defined(STM8S103) || defined(STM8S903)
    /* enable clock to TIM1 */
    CLK->PCKENR1 |= (CLK_PCKENR1_TIM1);

    /* LSI clock connected to TIM1 Input Capture 1 to measure LSI frequency */
    AWU->CSR |= AWU_CSR_MSR;

    /* Enable capture of 4 events on TIM1_CC1 */
    TIM1->PSCRH = 0u;                                   /* init divider register /1 */
    TIM1->PSCRL = 0u;
    TIM1->ARRH = 0xffu;			                /* init ARR, CNT & OC1 compare registers */
    TIM1->ARRL = 0xffu;
    TIM1->CNTRH = 0xffu;
    TIM1->CNTRL = 0xffu;
    TIM1->CCMR1 = ((2u << 2) & TIM1_CCMR_ICxPSC) | (1u & TIM1_CCMR_CCxS);
    /* CC1 input capture, no filter, divider= 4 */
    TIM1->CCER1 = TIM1_CCER1_CC1E;                      /* CC1 IC enable, rising edge */
    TIM1->CR1 |= (uint8_t)(TIM1_CR1_URS | TIM1_CR1_CEN);     /* enable timer */
    while (((TIM1->SR1 & TIM1_SR1_CC1IF) != TIM1_SR1_CC1IF) && (time_out != 0u))
    {
      --time_out;
    }
  #endif /* STM8S103 || STm8S903 */

  CtrlFlowCntInv -= XCLK_MEASURE_INIT_CALLEE;

  if (time_out == 0u)
  {
    sts = ERROR;
  }
  return(sts);
}

/* ---------------------------------------------------------------------------*/
/**
  * @brief LSI frequency measuremet
  * @par Parameters:
  * None
  * @retval
  * uint16_t difference between two subsequent capture events
  * @par Required preconditions:
  * STL_InitClock_Xcross_Measurement()
  * @par Library functions called:
  * None
  */
uint16_t STL_MeasureLSIPeriod(void)
{
#if defined(STM8L15X_MD) || defined(STM8L15X_MDP) || defined(STM8L15X_HD) || \
    defined(STM8L10X) || defined(STM8TL5X) || defined(STM8S208) || \
        defined(STM8S207) || defined(STM8S105) || defined(STM8S103) || defined(STM8S903)

  uint16_t time_out = LSI_MEASURE_TIMEOUT;
  static uint16_t temp_cc1;
  static uint16_t temp_cc1_last;
#endif
  static uint16_t period;

  CtrlFlowCnt += XLCK_LSI_PERIOD_CALLEE;

  #if defined(STM8L15X_MD) || defined(STM8L15X_MDP) || defined(STM8L15X_HD) || defined(STM8L10X) || defined(STM8TL5X)
    TIM2->SR2 &= (uint8_t)(~TIM_SR2_CC1OF);    /* clear CC1 overcapture flag */
    TIM2->SR1 &= (uint8_t)(~TIM_SR1_CC1IF);    /* clear CC1 capture flag */

    /* wait for a next capture event on cc1 */
    while (((TIM2->SR1 & TIM_SR1_CC1IF) != TIM_SR1_CC1IF) && (time_out != 0u))
    {
      --time_out;
    }
    /* Get first CCR1 value*/
    temp_cc1_last = ((uint16_t)(TIM2->CCR1H) << 8);
    temp_cc1_last += TIM2->CCR1L;          /* preload register is frozen till CCR1L value is not read */
    /* The CC1IF flag is already cleared here be reading CCR1L register */

    /* wait for one more capture event on cc1 */
    while (((TIM2->SR1 & TIM_SR1_CC1IF) != TIM_SR1_CC1IF) && (time_out != 0u))
    {
      --time_out;
    }

    /* if there is no or over capture event return zero period */
    if (((TIM2->SR2 & TIM_SR2_CC1OF) != 0u))
    {
      period = 0u;                         /* when overcaptured, ignore this measurement result */
    }
    else if (time_out == 0u)
    {
      period = 1u;                         /* set wrong (too short) LSI period at case of no timer event */
    }
    else
    {
      /* Get next CCR1 value*/
      temp_cc1 = ((uint16_t)(TIM2->CCR1H) << 8);
      temp_cc1 += TIM2->CCR1L;               /* preload register is frozen till CCR1L value is not read */
      /* The CC1IF flag is already cleared here be reading CCR1L register */
      period = temp_cc1 - temp_cc1_last;
    }
  #endif /* STM8L15X_MD || STM8L15X_MDP || STM8L15X_HD */

  #if defined(STM8S208) || defined(STM8S207) || defined(STM8S105)
    TIM3->SR2 &= (uint8_t)(~TIM3_SR2_CC1OF);    /* clear CC1 overcapture flag */
    TIM3->SR1 &= (uint8_t)(~TIM3_SR1_CC1IF);    /* clear CC1 capture flag */

    /* wait for a next capture event on cc1 */
    while (((TIM3->SR1 & TIM3_SR1_CC1IF) != TIM3_SR1_CC1IF) && (time_out != 0u))
    {
      --time_out;
    }
    /* Get first CCR1 value*/
    temp_cc1_last = ((uint16_t)(TIM3->CCR1H) << 8);
    temp_cc1_last += TIM3->CCR1L;          /* preload register is frozen till CCR1L value is not read */
    /* The CC1IF flag is already cleared here be reading CCR1L register */

    /* wait for one more capture event on cc1 */
    while (((TIM3->SR1 & TIM3_SR1_CC1IF) != TIM3_SR1_CC1IF) && (time_out != 0u))
    {
      --time_out;
    }

    /* if there is no or over capture event return zero period */
    if (((TIM3->SR2 & TIM3_SR2_CC1OF) != 0u))
    {
      period = 0u;                         /* when overcaptured, ignore this measurement result */
    }
    else if (time_out == 0u)
    {
      period = 1u;                         /* set wrong (too short) LSI period at case of no timer event */
    }
    else
    {
      /* Get next CCR1 value*/
      temp_cc1 = ((uint16_t)(TIM3->CCR1H) << 8);
      temp_cc1 += TIM3->CCR1L;               /* preload register is frozen till CCR1L value is not read */
      /* The CC1IF flag is already cleared here be reading CCR1L register */
      period = temp_cc1 - temp_cc1_last;
    }
  #endif /* STM8S208 || STM8S207 || STM8S105 */

  #if defined(STM8S103) || defined(STM8S903)
    /* wait for a next capture event on cc1 */
    while (((TIM1->SR1 & TIM1_SR1_CC1IF) != TIM1_SR1_CC1IF) && (time_out != 0u))
    {
      --time_out;
    }

    /* Get first CCR1 value*/
    temp_cc1_last = (TIM1->CCR1H << 8);
    temp_cc1_last += TIM1->CCR1L;          /* preload register is frozen till CCR1L value is not read */
    /* The CC1IF flag is already cleared here be reading CCR1L register */

    /* wait for one more capture event on cc1 */
    while (((TIM1->SR1 & TIM1_SR1_CC1IF) != TIM1_SR1_CC1IF) && (time_out != 0u))
    {
      --time_out;
    }

    /* if there is no or over capture event return zero period */
    if (((TIM1->SR2 & TIM1_SR2_CC1OF) != 0u) || (time_out == 0))
    {
      period = 0;
    }
    else
    {
      /* Get next CCR1 value*/
      temp_cc1 = (TIM1->CCR1H << 8);
      temp_cc1 += TIM1->CCR1L;               /* preload register is frozen till CCR1L value is not read */
      /* The CC1IF flag is already cleared here be reading CCR1L register */
      period = temp_cc1 - temp_cc1_last;
    }
  #endif /* STM8S103 || STM8S903 */

  #if defined(EVAL_BOARD_LCD)
    LSI_Measured = period;
  #endif /* EVAL_BOARD_LCD */

  CtrlFlowCntInv -= XLCK_LSI_PERIOD_CALLEE;
  return (period);
}
/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/