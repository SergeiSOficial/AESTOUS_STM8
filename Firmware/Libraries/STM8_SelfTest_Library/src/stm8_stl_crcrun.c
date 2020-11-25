/**
  ******************************************************************************
  * @file stm8_stl_crcrun.c
  * @brief Contains the functions performing run time invariable memory checks
  *        based on 16-bit CRC by default
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

#ifdef _COSMIC_
  uint8_t CRC_ERROR_s @0xFF; /* Trick to export constants to asm with immediate adressing */
  uint8_t CRC_ONGOING_s @0x01;	/* Trick */
#endif /* _COSMIC_ */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef _COSMIC_
extern uint8_t _block_checksum160(void);
extern @far uint8_t _block_checksum161(void);
extern uint8_t _block_checksum80(void);
extern @far uint8_t _block_checksum81(void);
#endif /* _COSMIC_ */

#ifdef _IAR_
  uint8_t FlashCrc16_Block_test(void);
#endif /* _IAR_ */

#ifdef STL_INCL_RUN_FLASH
/* Private functions ---------------------------------------------------------*/
/**
  * @brief Computes the crc in multiple steps and compare it with the
  *        ref value when the whole memory has been tested
  * @par Parameters:
  * None
  * @retval
  * ClassBTestStatus {TEST_RUNNING, CLASS_B_DATA_FAIL,
  *                  CTRL_FLW_ERROR, TEST_FAILURE, TEST_OK}
  * @par Required preconditions:
  * - STL_FlashCrcInit()
  * @par Library functions called:
  * - block_checksum16x()
  * - STL_FlashCrcInit()
  */
ClassBTestStatus STL_crc16Run(void)
{
  ClassBTestStatus result = CTRL_FLW_ERROR; /* In case of abnormal func exit*/

  CtrlFlowCnt += CRC16_RUN_TEST_CALLEE;

  /* Check Class B var integrity */
  if ((CurrentCrc16 ^ CurrentCrc16Inv) == 0xFFFFu)
  {
  #ifdef _COSMIC_
    #ifndef CRC_CHECK_8
      #ifndef CRC_CHECK_FAR
        switch ( _block_checksum160() )
      #else
        switch ( _block_checksum161() )
      #endif /* CRC_CHECK_FAR */
    #else
      #ifndef CRC_CHECK_FAR
        switch ( _block_checksum80() )
      #else
        switch ( _block_checksum81() )
      #endif /* CRC_CHECK_FAR */
    #endif /* CRC_CHECK_8 */
  #endif /* _COSMIC_ */
  #ifdef _IAR_
    switch ( FlashCrc16_Block_test() )
  #endif  /* _IAR_ */      
    {
    #if defined(DEBUG) && defined(_COSMIC_)
      case CRC_ERROR:  /* flash test error is ignored at debug mode with Cosmic */
    #endif /* DEBUG && _COSMIC_ */
      case CRC_OK:
        result = TEST_OK;
        #ifndef CRC_CHECK_8
          STL_FlashCrc16Init(); /* Prepare next test */
        #else
          STL_FlashCrc8Init();
        #endif /* CRC_CHECK_8 */
        break;

      case CRC_ONGOING:
        result = TEST_RUNNING;
        break;

	#if !defined(DEBUG) || !defined(_COSMIC_)
      case CRC_ERROR:
	#endif /* !(DEBUG || _COSMIC_) */
      default:
        result = TEST_FAILURE;
        break;
    }
  }
  else  /* Class B error CurrentCrc16 */
  {
    result = CLASS_B_DATA_FAIL;
  }

  CtrlFlowCntInv -= CRC16_RUN_TEST_CALLEE;

  return (result);

}
#endif /* STL_INCL_RUN_FLASH */
/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/