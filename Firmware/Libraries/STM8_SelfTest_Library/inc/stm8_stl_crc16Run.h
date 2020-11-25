/**
  ******************************************************************************
  * @file stm8_stl_crc16Run.h
  * @brief Contains the prototypes of the functions performing
  *        run time invariable memory checks based on 16-bit CRC
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STL_FLASH_CRC16_RUN_TIME_H
#define __STL_FLASH_CRC16_RUN_TIME_H

/* Includes ------------------------------------------------------------------*/
#include "stm8_stl_lib.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define CRC_OK (0u)
#define CRC_ERROR (0xFFu)
#define CRC_ONGOING (0x01u)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
  #ifdef _COSMIC_
    @near void STL_FlashCrc16Init(void);
    @near void STL_FlashCrc8Init(void);
  #endif /* _COSMIC_ */
  #ifdef _IAR_
    void STL_FlashCrc16Init(void);
  #endif /* _IAR_ */
ClassBTestStatus STL_crc16Run(void);

#endif /* __STL_FLASH_CRC16_RUN_TIME_H */

/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
