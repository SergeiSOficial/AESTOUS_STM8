/**
  ******************************************************************************
  * @file stm8_stl_fullRam_Mc.h
  * @brief This file contains prototype of the RAM functional test
  *         to be done at start-up.
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
#ifndef __FULL_RAM_MARCHC_H
#define __FULL_RAM_MARCHC_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* This is to be sure that tested value is in-line with code returned by the
   with assembly routine. */
#define FULL_RAM_OK ((uint8_t)0x01)
  #ifdef _COSMIC_
    uint8_t FULL_RAM_OK_s @0x01; /* Trick to export constants to asm with immediate addressing */
  #endif /* _COSMIC_ */
  #ifdef _IAR_
    __no_init volatile uint8_t FULL_RAM_OK_s @ 0x01; /* Trick to export constants to asm with immediate addressing */
  #endif /* _IAR_ */ 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
  #ifdef _COSMIC_
    @near ErrorStatus STL_FullRamMarchC(void);
  #endif /* _COSMIC_ */
  #ifdef _IAR_ 
    ErrorStatus STL_FullRamMarchC(void);
  #endif /* _IAR_ */   

#endif /* __FULL_RAM_MARCHC_H */

/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
