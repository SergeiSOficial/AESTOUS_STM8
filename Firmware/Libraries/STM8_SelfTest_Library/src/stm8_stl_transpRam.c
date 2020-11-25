/**
  ******************************************************************************
  * @file stm8_stl_transpRam.c
  * @brief This module contains functions for transparent RAM functional
  *        test using March C-/X algorithm, to be done during run-time.
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

#ifdef STL_INCL_RUN_RAM
  /* ==============================================================================*/
  /* MISRA violation of rule 17.4 - pointer arithmetic is used for RAM area testing */
  #ifdef _IAR_
    #pragma diag_suppress=Pm088
  #endif /* _IAR_ */

/* Private functions ---------------------------------------------------------*/
/**
  * @brief Initializes the pointer to the RAM for the run-time
  *        transparent functional test.
  * @par Parameters:
  * None
  * @retval
  * None
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * None
  */
void STL_TranspMarchInit(void)
{
  CtrlFlowCnt += RAM_MARCH_INIT_CALLEE;

  p_RunTimeRamChk = CLASS_B_START;
  /* allow to include overlay at begin of the tested area 
     supposed the CLASS_B_START points to first address to be tested */
  p_RunTimeRamChk -= RT_RAM_BLOCK_OVERLAP;    
  p_RunTimeRamChkInv = (uint8_t *)(uint16_t)(~(uint16_t)(p_RunTimeRamChk));

  CtrlFlowCntInv -= RAM_MARCH_INIT_CALLEE;

}
/* ---------------------------------------------------------------------------*/
/**
  * @brief This function verifies that 6 words of RAM are functional,
  *        (with overlapping) using the March algorithm.
  * @par Parameters:
  * None
  * @retval
  * ClassBTestStatus {TEST_RUNNING, CLASS_B_DATA_FAIL,
  *                   CTRL_FLW_ERROR, TEST_FAILURE, TEST_OK}
  * @par Required preconditions:
  * STL_TranspMarchInit()
  * @par Library functions called:
  * None
  */
ClassBTestStatus STL_TranspMarch(void)
{
  ClassBTestStatus result = TEST_RUNNING;
  uint8_t index;		/* Index for temporary data storage while RAM is being tested */
  #ifdef _COSMIC_
    uint8_t @near *p_ram_block_end, *p_ram_block_start;
  #endif /* _COSMIC_ */
  #ifdef _IAR_
    uint8_t *p_ram_block_end, *p_ram_block_start;
  #endif /* _IAR_ */

  ISRCtrlFlowCnt += RAM_MARCH_ISR_CALLEE;

  /* Check Class B var integrity */
  if ((((uint16_t)p_RunTimeRamChk) ^ ((uint16_t)p_RunTimeRamChkInv)) == 0xFFFFu)
  {
    /* test if all the area is already tested - supossed CLASS_B_END points to last word to be tested */
    if (p_RunTimeRamChk >= CLASS_B_END)
    {
      /*------------- Apply March test to the RAM Buffer itself --------------- */
      p_RunTimeRamChk = &aRunTimeRamBuf[0];
      p_RunTimeRamChkInv = (uint8_t *)(uint16_t)(~(uint16_t)(p_RunTimeRamChk));

      /*---------------------------- STEP 1 --------------------------------- */
      /* Write background with addresses increasing */
      do
      {
        *p_RunTimeRamChk++ = BCKGRND;
      }
      while (p_RunTimeRamChk < &aRunTimeRamBuf[RT_RAM_BUF_SIZE]);
      /*---------------------------- STEP 2 --------------------------------- */
      /* Verify background and write inverted background addresses increasing */
      p_RunTimeRamChk = &aRunTimeRamBuf[0];
      do
      {
        if (*p_RunTimeRamChk != BCKGRND)
        {
          result = TEST_FAILURE;
        }
        *p_RunTimeRamChk++ = INV_BCKGRND;
      }
      while (p_RunTimeRamChk < &aRunTimeRamBuf[RT_RAM_BUF_SIZE]);

      #ifndef STL_RUN_USE_MARCHX
        /*---------------------------- STEP 3 --------------------------------- */
        /* Verify inverted background and write background addresses increasing */
        p_RunTimeRamChk = &aRunTimeRamBuf[0];
        do
        {
          if ( *p_RunTimeRamChk != INV_BCKGRND)
          {
            result = TEST_FAILURE;
          }
          *p_RunTimeRamChk++ = BCKGRND;
        }
        while (p_RunTimeRamChk < &aRunTimeRamBuf[RT_RAM_BUF_SIZE]);
        /*---------------------------- STEP 4 --------------------------------- */
        /* Verify background and write inverted background addresses decreasing */
        p_RunTimeRamChk = &aRunTimeRamBuf[RT_RAM_BUF_SIZE];
        do
        {
          --p_RunTimeRamChk;
          if ( *p_RunTimeRamChk != BCKGRND )
          {
            result = TEST_FAILURE;
          }
          *p_RunTimeRamChk = INV_BCKGRND;
        }
        while (p_RunTimeRamChk > &aRunTimeRamBuf[0]);
      #endif /* STL_RUN_USE_MARCHX */

      /*---------------------------- STEP 5 --------------------------------- */
      /* Verify inverted background and write background addresses decreasing */
      p_RunTimeRamChk = &aRunTimeRamBuf[RT_RAM_BUF_SIZE];
      do
      {
        --p_RunTimeRamChk;
        if ( *p_RunTimeRamChk != INV_BCKGRND )
        {
          result = TEST_FAILURE;
        }
        *p_RunTimeRamChk = BCKGRND;
      }
      while (p_RunTimeRamChk > &aRunTimeRamBuf[0]);

      /*---------------------------- STEP 6 --------------------------------- */
      /* Verify background with addresses increasing */
      p_RunTimeRamChk = &aRunTimeRamBuf[0];
      do
      {
        if ( *p_RunTimeRamChk++ != BCKGRND )
        {
          result = TEST_FAILURE;
        }
      }
      while (p_RunTimeRamChk < &aRunTimeRamBuf[RT_RAM_BUF_SIZE]);

      /* Prepare next Transparent RAM test from the beginning of Class B area */
      p_RunTimeRamChk = CLASS_B_START - 1;
      p_RunTimeRamChkInv = ((uint8_t *)(uint16_t)(~(uint16_t)(p_RunTimeRamChk)));
      if (result == TEST_RUNNING)
      {
        result = TEST_OK; /* Means all selected variable memory was scanned */
      }
      else  /* Buffer is not functional */
      {
        result = TEST_FAILURE;
      }
    } /* ------------------ End of Buffer Self-check ------------------------ */
    else
    { /* ------------------ Regular memory Self-check ----------------------- */
      /*---------------------------- STEP 1 --------------------------------- */
      /* Save the content of all the bytes to be tested and start MarchC-
         Write background with addresses increasing */
      p_ram_block_start = p_RunTimeRamChk;
      p_ram_block_end = p_RunTimeRamChk + RT_RAM_BLOCKSIZE + 2u * RT_RAM_BLOCK_OVERLAP;
      index = 1u; /* first and last item of the buffer is not used and tested as overlay */
      do
      {
        aRunTimeRamBuf[index++] = *p_RunTimeRamChk;
        *p_RunTimeRamChk++ = BCKGRND;
      }
      while (p_RunTimeRamChk < p_ram_block_end);

      /*---------------------------- STEP 2 --------------------------------- */
      /* Verify background and write inverted background addresses increasing */
      p_RunTimeRamChk = p_ram_block_start;
      do
      {
        if (*p_RunTimeRamChk  != BCKGRND)
        {
          result = TEST_FAILURE;
        }
        *p_RunTimeRamChk++ = INV_BCKGRND;
      }
      while (p_RunTimeRamChk < p_ram_block_end);

      #ifndef STL_RUN_USE_MARCHX
        /*---------------------------- STEP 3 --------------------------------- */
        /* Verify inverted background and write background addresses increasing */
        p_RunTimeRamChk = p_ram_block_start;
        do
        {
          if (*p_RunTimeRamChk != INV_BCKGRND)
          {
            result = TEST_FAILURE;
          }
          *p_RunTimeRamChk++ = BCKGRND;
        }
        while (p_RunTimeRamChk < p_ram_block_end);
      
        /*---------------------------- STEP 4 --------------------------------- */
        /* Verify background and write inverted background addresses decreasing */
        p_RunTimeRamChk = p_ram_block_end;
        do
        {
          --p_RunTimeRamChk;
          if (*p_RunTimeRamChk != BCKGRND)
          {
            result = TEST_FAILURE;
          }
          *p_RunTimeRamChk = INV_BCKGRND;
        }
        while (p_RunTimeRamChk > p_ram_block_start);
      #endif /* STL_RUN_USE_MARCHX */
			
      /*---------------------------- STEP 5 --------------------------------- */
      /* Verify inverted background and write background addresses decreasing */
      p_RunTimeRamChk = p_ram_block_end;
      do
      {
        --p_RunTimeRamChk;
        if (*p_RunTimeRamChk != INV_BCKGRND)
        {
          result = TEST_FAILURE;
        }
        *p_RunTimeRamChk = BCKGRND;
      }
      while (p_RunTimeRamChk > p_ram_block_start);
      /*---------------------------- STEP 6 --------------------------------- */
      /* Verify background with addresses increasing */
      /* and restore the content of the 6 tested words */
      p_RunTimeRamChk = p_ram_block_start;
      index = 1u;
      do
      {
        if (*p_RunTimeRamChk != BCKGRND)
        {
          result = TEST_FAILURE;
        }
        *p_RunTimeRamChk++ = aRunTimeRamBuf[index++];
      }
      while (p_RunTimeRamChk < p_ram_block_end);
			
      /* Prepare next Row Transparent RAM test */
      p_RunTimeRamChk -=  (2u *  RT_RAM_BLOCK_OVERLAP);
      p_RunTimeRamChkInv = ((uint8_t *)(uint16_t)(~(uint16_t)(p_RunTimeRamChk)));
      if (result != TEST_RUNNING)
      {
        result = TEST_FAILURE;  /* byte block under test was not functional */
      }
    } /* --------------- End of Regular memory Self-check --------------------- */
		
  }
  else  /* Class B error on p_RunTimeRamChk when entering the function*/
  {
    result = CLASS_B_DATA_FAIL;
  }

  ISRCtrlFlowCntInv -= RAM_MARCH_ISR_CALLEE;

  return (result);
}
  #ifdef _IAR_
    #pragma diag_default=Pm088
  #endif /* _IAR_ */
  /* ==============================================================================*/

#endif /* STL_INCL_RUN_RAM */
/**
  * @}
  */
/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/