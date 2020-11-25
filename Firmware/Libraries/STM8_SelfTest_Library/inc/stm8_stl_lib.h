/**
  ******************************************************************************
  * @file stm8_stl_lib.h
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
#ifndef __STM8S20x_STL_LIB_H
#define __STM8S20x_STL_LIB_H

/* Includes ------------------------------------------------------------------*/
/* All user customizable parameters */
#include "stdio.h"

#if defined(STM8L15X_MD) || defined(STM8L15X_MDP) || defined(STM8L15X_HD)
  #include "stm8l15x.h"
#elif defined(STM8L10X)
  #include "stm8l10x.h"
#elif defined(STM8TL5X)
  #include "stm8tl5x.h"
#elif defined(STM8S208) || defined (STM8S207) || defined (STM8S007) || defined(STM8S105) || defined(STM8S005) || defined(STM8S103)\
   || defined(STM8S003) || defined(STM8S903) || defined (STM8AF52Ax) || defined(STM8AF62Ax) || defined(STM8AF626x)
  #include "stm8s.h"
#else /* product type doesn't match */
  #error Please check if corect product representative is defined and corresponding header file is included here
#endif /* STM8xxx */

#include "stm8_stl_param.h"

/* Examples of Self Test library demo routines use */
#include "stm8_stl_startup.h"
#include "stm8_stl_main.h"

/* STM8 CPU test */
#include "stm8_stl_cpu.h"

/* Clock frequency test */
#include "stm8_stl_clockstart.h"
#include "stm8_stl_clockrun.h"

/* Invariable memory test */
#include "stm8_stl_crc16Run.h"

/* Variable memory test */
#include "stm8_stl_fullRam_Mc.h"

#include "stm8_stl_transpRam.h"

/* External variables --------------------------------------------------------*/
#if defined(EVAL_BOARD_LCD)
  extern char aDemoString[17];
  extern uint16_t LSI_Calculated;
  extern uint16_t LSI_Measured;
#endif /* EVAL_BOARD_LCD */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef _COSMIC_
  @near void FailSafe(uint16_t err_no);
  #define INLINE_FUNCTION @inline
#endif /* _COSMIC_ */
#ifdef _IAR_
  void FailSafe(uint16_t err_no);
  #define INLINE_FUNCTION inline
#endif /* _IAR_ */

/* inline function to control of independent watchdog */
INLINE_FUNCTION void refresh_iwdog(void)
{
  IWDG->KR = 0xAAu;
}
#ifndef STM8L10X
/* inline function to control of window watchdog */
INLINE_FUNCTION void refresh_wwdog(uint8_t cnt, uint8_t win)
{
  WWDG->CR = cnt | WWDG_CR_WDGA;
  WWDG->WR = win;
}
#endif /* STM8L10X */

void InitClassB_Demo(void);
void LCD_Print(uint8_t *ptr);

/* inline function handling entry to FailSafe state while passisng 
   information about the error (error code and verbose description)
   in dependency of the debugging level */
INLINE_FUNCTION void fail_safe_assert(uint16_t err_code, char *verb_txt)
{
  #ifdef EVAL_BOARD_LCD
    /* error code goes to on eval board display */
    refresh_wwdog(0x7Fu, 0x7Fu);
    refresh_iwdog();
    InitClassB_Demo();
    refresh_wwdog(0x7Fu, 0x7Fu);
    refresh_iwdog();
    sprintf(aDemoString, ">FailSafe Er:%02X", (err_code)); LCD_Print((uint8_t *)aDemoString); 
  #endif /* EVAL_BOARD_LCD */

  #ifdef STL_VERBOSE_FAILSAFE
    /* verbose text goes to UART terminal */
    STL_VerboseInit();
    putchar('\n');
    putchar('\r');
    printf(verb_txt);
  #endif /* STL_VERBOSE_FAILSAFE */

/* error code is passed as parameter to FailSafe routine */
  FailSafe(err_code);
}

#endif /* __STM8S20x_STL_LIB_H */
/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
