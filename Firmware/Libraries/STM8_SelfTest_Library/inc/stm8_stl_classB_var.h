/**
  ******************************************************************************
  * @file stm8_stl_classB_var.h
  * @brief This file contains definition all class B variables
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
#ifndef __STL_CLASS_B_VAR_H
#define __STL_CLASS_B_VAR_H
#include "stm8_stl_param.h"


/* This avoids having mutiply defined global variables */
#ifdef ALLOC_GLOBALS
  #define EXTERN
#else
  #define EXTERN extern
#endif /* ALLOC_GLOBALS */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Global variables --------------------------------------------------------- */
extern uint8_t __ICFEDIT_region_CLASSB_start__;
extern uint8_t __ICFEDIT_region_CLASSB_end__;

#ifdef _COSMIC_
  extern @near uint8_t _clb_start;
  extern @near uint8_t _clb_end;
#endif /* _COSMIC_ */

  /* Temporary RAM buffer used during transparent run-time tests */
  /* WARNING: Real reserved RAM location from 0x.... to 0x.... */
  /* ==============================================================================*/
  /* MISRA violation of rule 8.5 - Class B variables are defined at one place only 
     defined just at begin of stm8l_stl_startup.c file, all other files include this
     header as a common reference */
#ifdef _IAR_
  #pragma diag_suppress=Pm123
#endif /* _IAR_ */

#ifdef _COSMIC_
  #pragma section @near [RUN_TIME_BUF]
  /* Buffer to store original RAM content during run-time tests */
	EXTERN @near uint8_t aRunTimeRamBuf[RT_RAM_BUF_SIZE];

  /* RAM pointer for run-time tests */
	EXTERN @near uint8_t *p_RunTimeRamChk;
	EXTERN @near uint8_t *p_RunTimeRamChkInv;

  #pragma section @near [CLASS_B]
  
  EXTERN @near uint16_t CtrlFlowCnt;   /* program flow execution at start */
  EXTERN @near uint16_t ISRCtrlFlowCnt; /* program flow execution in interrupt */
  EXTERN @near uint16_t TickCounter; /* Sofware time base used in main program (SysTick ISR) */
  EXTERN @near volatile uint8_t TimeBaseFlag; /* Indicates to the main routine a 100ms tick */
  EXTERN @near uint16_t LastCtrlFlowCnt; /* Control flow counter from one main loop to the other */
  EXTERN @near uint8_t *p_RunCrc16Chk; /* Pointer to FLASH for crc16 run-time tests */
  #ifdef CRC_CHECK_FAR
    EXTERN @near uint16_t MSB_RunCrc16Chk; /* MSB address for far crc16 run-time tests */
  #endif /* CRC_CHECK_FAR */
  EXTERN @near uint16_t CurrentCrc16; 	/* Current FLASH 16-bit Crc */
  EXTERN @near uint16_t CurrentDesc; 	/* Current Descriptor Address */
  EXTERN @tiny uint8_t CRCBlockIndex;	/* Counter needed for partial CRC test */

  #pragma section @near [STACK_BOTTOM]
  
  EXTERN @near volatile uint8_t aStackOverFlowPtrn[4]; /* Magic pattern for Stack overflow in this array */

  #pragma section @near [CLASS_B_REV]
  
  EXTERN @near uint16_t CtrlFlowCntInv;
  EXTERN @near uint16_t ISRCtrlFlowCntInv;
  EXTERN @near uint16_t TickCounterInv;
  EXTERN @near volatile uint8_t TimeBaseFlagInv;
  EXTERN @near uint16_t LastCtrlFlowCntInv;
  EXTERN @near uint8_t *p_RunCrc16ChkInv;
  #ifdef CRC_CHECK_FAR
    EXTERN @near uint16_t MSB_RunCrc16ChkInv;
  #endif /* CRC_CHECK_FAR */
  EXTERN @near uint16_t CurrentCrc16Inv;
  EXTERN @near uint16_t CurrentDescInv;

  #pragma section @near [bss]
#endif  /* _COSMIC_ */

#ifdef _IAR_
  EXTERN __near uint8_t aRunTimeRamBuf[RT_RAM_BUF_SIZE] @ "RUN_TIME_RAM_CHCK";

  /* RAM pointer for run-time tests - correction from version 1.1.0 certified by VDE at 2012 */
  EXTERN __near uint8_t *p_RunTimeRamChk @ "RUN_TIME_RAM_CHCK";
  EXTERN __near uint8_t *p_RunTimeRamChkInv @ "RUN_TIME_RAM_CHCK";

  EXTERN __near uint16_t CtrlFlowCnt @ "CLASS_B_RAM";   /* program flow execution at start */
  EXTERN __near uint16_t ISRCtrlFlowCnt @ "CLASS_B_RAM"; /* program flow execution in interrupt */
  EXTERN __near uint16_t TickCounter @ "CLASS_B_RAM"; /* Sofware time base used in main program (SysTick ISR) */
  EXTERN __near volatile uint8_t TimeBaseFlag @ "CLASS_B_RAM"; /* Indicates to the main routine a 100ms tick */
  EXTERN __near uint16_t LastCtrlFlowCnt @ "CLASS_B_RAM"; /* Control flow counter from one main loop to the other */
  EXTERN __near const uint8_t *p_RunCrc16Chk @ "CLASS_B_RAM"; /* Pointer to FLASH for crc16 run-time tests */
  EXTERN __near uint16_t CurrentCrc16 @ "CLASS_B_RAM"; 	/* Current FLASH 16-bit Crc */
  EXTERN __near uint16_t CurrentDesc @ "CLASS_B_RAM"; 	/* Current Descriptor Address */
  EXTERN __near uint16_t CRCBlockIndex @ "CLASS_B_RAM";	/* Counter needed for partial CRC test */

  EXTERN __near volatile uint8_t aStackOverFlowPtrn[4] @ "STACK_BOTTOM"; /* Magic pattern for Stack overflow in this array */

  EXTERN __near uint16_t CtrlFlowCntInv @ "CLASS_B_RAM_REV";
  EXTERN __near uint16_t ISRCtrlFlowCntInv @ "CLASS_B_RAM_REV";
  EXTERN __near uint16_t TickCounterInv @ "CLASS_B_RAM_REV";
  EXTERN __near volatile uint8_t TimeBaseFlagInv @ "CLASS_B_RAM_REV";
  EXTERN __near uint16_t LastCtrlFlowCntInv @ "CLASS_B_RAM_REV";
  EXTERN __near const uint8_t *p_RunCrc16ChkInv @ "CLASS_B_RAM_REV";
  EXTERN __near uint16_t CurrentCrc16Inv @ "CLASS_B_RAM_REV";
  EXTERN __near uint16_t CurrentDescInv @ "CLASS_B_RAM_REV";
  EXTERN __near uint16_t CRCBlockIndexInv @ "CLASS_B_RAM_REV";
#endif  /* _IAR_ */

#ifdef _IAR_
  #pragma diag_default=Pm123
#endif /* _IAR_ */
  /* ==============================================================================*/
        
#endif /* __STL_CLASS_B_VAR_H */

/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
