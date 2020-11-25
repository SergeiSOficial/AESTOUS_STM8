/**
  ******************************************************************************
  * @file stm8_stl_clockrun.c
  * @brief This file contains the test function for clock circuitry and wrong frequency
  *        detection in run.
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
/**
  * @brief Verifies if clock source frequency did not change vs initial measurement
  * @par Parameters:
  * None
  * @retval
  * ClockStatus = {LSI_START_FAIL; HSE_START_FAIL;
  *                HSI_HSE_SWITCH_FAIL; TEST_ONGOING; EXT_SOURCE_FAIL;
  *                CLASS_B_VAR_FAIL, CTRL_FLOW_ERROR, FREQ_OK}
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * - STL_MeasureLSIPeriod()
  */
ClockStatus STL_ClockFreqTest(void)
{
  ClockStatus result = TEST_ONGOING; /* In case of unexpected exit */
  static uint16_t lsi_period;
  static uint16_t expected_value;
	
  CtrlFlowCnt += FREQ_TEST_CALLEE;

  if ((lsi_period = STL_MeasureLSIPeriod()) != 0u)
  {
    expected_value = calc_captured_value();
  #ifdef STL_INCL_HSECSS
    if ((lsi_period < (expected_value * 3u / 4u))\
    || (lsi_period > (expected_value * 5u / 4u)))
    {
      switch_clock_system(to_HSI);      /* Switch back to internal clock */
      result = EXT_SOURCE_FAIL;	        /* Sub-harmonics: HSE +/-25% out of expected */
    }
  #else /* products without HSE feature */
    if ((lsi_period < (expected_value * 4u / 5u))\
    || (lsi_period > (expected_value * 6u / 5u)))
    {
      result = HSI_SOURCE_FAIL;	        /* HSI +-20% out of expected */
    }
  #endif /* STL_INCL_HSECSS */ 
    else
    {
      result = FREQ_OK;         /* frequecy is within expected range */
    }
  }
  
  CtrlFlowCntInv -= FREQ_TEST_CALLEE;
  return (result);
}

/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/