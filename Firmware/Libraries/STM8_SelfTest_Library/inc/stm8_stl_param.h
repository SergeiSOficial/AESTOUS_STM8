/**
  ******************************************************************************
  * @file stm8_stl_param.h
  * @brief This file contains the parameters to be customized for
  *          the final application.
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
#ifndef __SELFTEST_PARAM_H
#define __SELFTEST_PARAM_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
  typedef enum {
    TEST_RUNNING,
    CLASS_B_DATA_FAIL,
    CTRL_FLW_ERROR,
    TEST_FAILURE,
    TEST_OK
  } ClassBTestStatus;

/* User configuration defines ------------------------------------------------*/

  /* Next parameters control including Class B tests during power-on phase */
  #define STL_INCL_POR        /* Include all start up tests - reset vector points to Class B start up routine */
  //#define STL_INCL_POR_CPU    /* Include CPU start up test */
  #define STL_INCL_POR_WDOG   /* Include IWDOG start up test */
  //#define STL_INCL_POR_FLASH  /* Include Flash CRC start up test - disable it when use break points! */
  //#define STL_INCL_POR_RAM    /* Include full RAM march C start up test */
  //#define STL_INCL_POR_CLOCK  /* Include clock start up test */

  /* Next parameters control including Class B tests during main program execution */
  #define STL_INCL_RUN        /* Include all run time tests */
//  #define STL_INCL_RUN_CPU    /* Include CPU run time test */
//  #define STL_INCL_RUN_STACK  /* Include stack boundaries run time test */
//  #define STL_INCL_RUN_CLOCK  /* Include clock run time test */
  //#define STL_INCL_RUN_FLASH  /* Include flash CRC run time test */
//  #define STL_INCL_RUN_RAM    /* Include partial RAM march C run time test in interrupt */
//  #define STL_RUN_USE_MARCHX  /* use march X instead C for partial RAM run time test */

#if defined(STM8S903) || defined(STM8S103) || defined(STM8S003) || defined(STM8L10X) || defined(STM8TL5X)
  #undef STL_INCL_HSECSS
#else /* product featuring by HSE */
  #define STL_INCL_HSECSS     /* Include all HSE tests & CSS & switch appli to HSE when HSE is avalibale */
#endif

#ifdef STL_VERBOSE
  /* Next parameters enable self-diagnostic messages reported on a PC via UART */
  #define STL_VERBOSE_POR     /* enable messages reported during power-on phase */
  #define STL_VERBOSE_RUN     /* enable messages reported during main program execution */
  #define STL_VERBOSE_FAILSAFE /* enable message reporting entry into FailSafe routine  */
#endif /* STL_VERBOSE */

  /* WWDG counter seeting - it counts down from WWDG_PERIOD, next refresh must fall between WWDG_WINDOW and 0x40  */
#if defined(DEBUG) || defined(EVAL_BOARD_LCD)
  #define WWDG_PERIOD 0x7Fu
  #define WWDG_WINDOW 0x7Fu
#else
  #define WWDG_PERIOD 0x60u
  #define WWDG_WINDOW 0x60u
#endif /* DEBUG */

/*  #define STL_CCO_ENABLE        enable clock output at CCO pin after reset */

/*  #define CRC_CHECK_8           CRC 8 bit simple computation enable (16-bit is default) */
/*  #define CRC_CHECK_FAR         CRC computation for far model enable (64KB is default) */

  /* These are the direct and inverted data (pattern) used during the RAM
  test, performed using March C- Algorithm */
  #define BCKGRND     ((uint8_t)0x00)
  #define INV_BCKGRND ((uint8_t)0xFF)

  /* Define here the main time base period. Here 16MHz/128 gives 125kHz input clock */
#if defined(STM8S903)
  #define TIMER_BASE_CK_PRESCALER TIM6_PRESCALER_128
#else
  #define TIMER_BASE_CK_PRESCALER TIM4_PRESCALER_128
#endif /* STM8S903 */
  #define TIMER_BASE_AUTORELOAD_PERIOD ((uint8_t)124)	/* Gives 1ms period @ 125kHz */
  /* This is to provide a time base longer than the Tim4 TB for the main */
  /* For instance this is needed to refresh the LSI watchdog and window watchdog */
  #define TEST_TIMEBASE  ((uint16_t)10) /* Unit is defined her above by TIMER 4/6 settings */

  /* Timeout required to avoid being stuck in while loops during clock circuitry
  initialization, in case of problem. Even if Watchdog is active, these timeouts
  allow to react more quickly */
  /* timing below are given with 16MHZ crystal oscillator */
  #define LSI_START_TIMEOUT ((uint16_t)1700) /* ~1ms (COSMIC, speed optimized) */

  /* ~35ms to be adapted depending on crystal/resonator */
  #define HSE_START_TIMEOUT ((uint16_t)17000) /* ~10ms (COSMIC, speed optimized) */

  #define LSI_MEASURE_TIMEOUT ((uint16_t)17000)  /* ~10ms (COSMIC, speed optimized) */

  #define FREQ_MEAS_TIMEOUT ((uint16_t)1700)/* ~1ms (COSMIC, speed optimized) */

  #define FREQ_MEAS_INIT_TIMEOUT ((uint16_t)1700)/* ~1ms (COSMIC, speed optimized) */

  #define CLK_SWITCH_TIMEOUT  ((uint16_t)0x491)

#if defined(STM8L15X_MD) || defined(STM8L15X_MDP) || defined(STM8L15X_HD)
  #define to_HSE  ((uint8_t)0x04)    /* definition of clock source codes (to be copied to SWR) */
  #define to_HSI  ((uint8_t)0x01)
  #define to_LSI  ((uint8_t)0x02)
#else
  #define to_HSE  ((uint8_t)0xB4)
  #define to_HSI  ((uint8_t)0xE1)
  #define to_LSI  ((uint8_t)0xD2)
#endif /* STM8L15X_xxx */

  /* -------------------------------------------------------------------------- */
  /* ------------------ CONTROL FLOW TAGS and CHECKPOINTS --------------------- */
  /* -------------------------------------------------------------------------- */

  /* initial start up tests */
#ifdef STL_INCL_POR_CPU
  #define CPU_POR_CALLER         (2u)
  #define CPU_POR_CALLEE         (3u) /* Do not modify: hard coded in assembly file */
#else
  #define CPU_POR_CALLER         (0u)
  #define CPU_POR_CALLEE         (0u)
#endif /* STL_INCL_POR_CPU */

#ifdef STL_INCL_POR_WDOG
  #define WDG_TEST_CALLER         (5u)
#else
  #define WDG_TEST_CALLER         (0u)
#endif /* STL_INCL_POR_WDOG */

#ifdef STL_INCL_POR_FLASH
  #define CRC16_TEST_CALLER       (7u)
#else
  #define CRC16_TEST_CALLER       (0u)
#endif /* STL_INCL_POR_FLASH */

  /* RAM_TEST_CALLEE is only needed for CtrlFlowCntInv when exiting routine */
  /* This is because the RAM test routines destroys the control flow counters */
  #define RAM_TEST_CALLEE         (0xFFFFFFFFuL)



#ifdef STL_INCL_POR_CLOCK
  #define CLOCK_POR_CALLER       (11u)
  #define CLOCK_POR_CALLEE       (13u)
  #define LSI_INIT_CALLEE        (17u)
  #define HSE_INIT_CALLEE        (19u)
  #define XCLK_MEASURE_INIT_CALLEE (23u)
  #define XLCK_LSI_PERIOD_CALLEE  (27u)
#else
  #define CLOCK_POR_CALLER       (0u)
  #define CLOCK_POR_CALLEE       (0u)
  #define LSI_INIT_CALLEE        (0u)
  #define HSI_INIT_CALLEE        (19u)
  #define XCLK_MEASURE_INIT_CALLEE (0u)
  #define XLCK_LSI_PERIOD_CALLEE  (0u)
#endif /* STL_INCL_POR_CLOCK */

  #define CHECKPOINT1 ((uint16_t)CPU_POR_CALLER + \
                        CPU_POR_CALLEE + \
                        WDG_TEST_CALLER + \
                        CRC16_TEST_CALLER)
#ifndef STL_INCL_HSECSS
  #define CHECKPOINT2 ((uint16_t)CLOCK_POR_CALLER + \
                       CLOCK_POR_CALLEE + \
                       LSI_INIT_CALLEE + \
                       XCLK_MEASURE_INIT_CALLEE +\
                       XLCK_LSI_PERIOD_CALLEE)

#else
  #define CHECKPOINT2 ((uint16_t)CLOCK_POR_CALLER + \
                       CLOCK_POR_CALLEE + \
                       LSI_INIT_CALLEE + \
                       HSE_INIT_CALLEE + \
                       XCLK_MEASURE_INIT_CALLEE + \
                       XLCK_LSI_PERIOD_CALLEE + \
                       XLCK_LSI_PERIOD_CALLEE)
#endif /* STL_INCL_HSECSS */
  /* run tests */
#ifdef STL_INCL_RUN_CPU
  #define CPU_RUN_CALLER         (9u)
  #define CPU_RUN_CALLEE         (11u) /* Do not modify: hard coded in assembly file */
#else
  #define CPU_RUN_CALLER         (0u)
  #define CPU_RUN_CALLEE         (0u)
#endif /* STL_INCL_RUN_CPU */

#ifdef STL_INCL_RUN_STACK
  #define STACK_OVERFLOW_CALLER   (29u)
  #define STACK_OVERFLOW_CALLEE   (31u)
#else
  #define STACK_OVERFLOW_CALLER   (0u)
  #define STACK_OVERFLOW_CALLEE   (0u)
#endif /* STL_INCL_RUN_STACK */

  #define CLOCKPERIOD_TEST_CALLEE (37u)

#ifdef STL_INCL_RUN_FLASH
  #define CRC16_RUN_TEST_CALLEE   (41u)
  #define CRC32_RUN_TEST_CALLEE   (43u)
  #define FLASH_RUN_TEST_CALLER   (47u)
#else
  #define CRC16_RUN_TEST_CALLEE   (0u)
  #define CRC32_RUN_TEST_CALLEE   (0u)
  #define FLASH_RUN_TEST_CALLER   (0u)
#endif /* STL_INCL_RUN_FLASH */

#ifdef STL_INCL_RUN_CLOCK
  #define FREQ_TEST_CALLER        (21u)
  #define FREQ_TEST_CALLEE        (23u)
  #define LSI_CHECK_INIT_CALLER   (57u)
  #define LSI_CHECK_INIT_CALLEE   XCLK_MEASURE_INIT_CALLEE
  #define LSI_MEASURE_CALLEE_     (25u)
#else
  #define FREQ_TEST_CALLER        (0u)
  #define FREQ_TEST_CALLEE        (0u)
  #define LSI_CHECK_INIT_CALLER	  (0u)
  #define LSI_MEASURE_CALLEE_     (0u)
#endif /* STL_INCL_RUN_CLOCK */

  #define DELTA_MAIN  (uint16_t)(CPU_RUN_CALLER + \
                            CPU_RUN_CALLEE + \
                            STACK_OVERFLOW_CALLER + \
                            STACK_OVERFLOW_CALLEE + \
                            FREQ_TEST_CALLER + \
                            FREQ_TEST_CALLEE + \
                            XLCK_LSI_PERIOD_CALLEE + \
                            FLASH_RUN_TEST_CALLER + \
                            CRC16_RUN_TEST_CALLEE)

  #define LAST_DELTA_MAIN ((uint16_t) DELTA_MAIN)
  #define FULL_FLASH_CHECKED (uint16_t)((DELTA_MAIN * CRC_NUMB_BLOCKS) & 0xFFFFu)

  /* ISR tests */
  #define CLOCKPERIOD_ISR_CALLEE  (5u)


#ifdef STL_INCL_RUN_RAM
    #define RAM_MARCH_INIT_CALLER  (37u)
    #define RAM_MARCH_INIT_CALLEE  (41u)
    #define RAM_MARCH_ISR_CALLER   (7u)
    #define RAM_MARCH_ISR_CALLEE   (11u)
#else
    #define RAM_MARCH_INIT_CALLER  (0u)
    #define RAM_MARCH_INIT_CALLEE  (0u)
    #define RAM_MARCH_ISR_CALLER   (0u)
    #define RAM_MARCH_ISR_CALLEE   (0u)
#endif /* STL_INCL_RUN_RAM */

#if defined(STL_INCL_RUN_RAM) || defined(STL_INCL_RUN_FLASH)
  #define TIM_BASE_INIT_CALLER     (29u)
  #define TIM_BASE_INIT_CALLEE     (31u)
#else
  #define TIM_BASE_INIT_CALLER     (0u)
  #define TIM_BASE_INIT_CALLEE     (0u)
#endif /* STL_INCL_RUN_RAM || STL_INCL_RUN_FLASH */

  /* This is for STL init called from user main */
#define CHECKPOINT_INIT ((uint16_t)RAM_MARCH_INIT_CALLER +\
                                   RAM_MARCH_INIT_CALLEE +\
                                   LSI_CHECK_INIT_CALLER +\
                                   LSI_CHECK_INIT_CALLEE +\
                                   TIM_BASE_INIT_CALLER +\
                                   TIM_BASE_INIT_CALLEE)
//#define CHECKPOINT_INIT ((uint16_t)RAM_MARCH_INIT_CALLER +\
//                                   RAM_MARCH_INIT_CALLEE +\
//                                   TIM_BASE_INIT_CALLER +\
//                                   TIM_BASE_INIT_CALLEE)

  /* This is for March C tests */
#define DELTA_ISR  ((uint16_t) RAM_MARCH_ISR_CALLER + \
                               RAM_MARCH_ISR_CALLEE)

/* Constants necessary for Transparent March tests defined at icf linker file.
   CLASS_B_START points out first address and CLASS_B_END the last one to be tested.
   NOTE: The tested area includes additional overlaps at its begin and end to test crosstalks
         of the fisrt and last tested addresses. If the number of the tested adresses
         is not multiply of the block size, additional redundant addresses to fit the
         step size are tested during the last step perfromed at the end of the area */
#ifdef _COSMIC_
#define CLASS_B_START &_clb_start
#define CLASS_B_END &_clb_end
#endif /* _COSMIC_ */
#ifdef _IAR_
#define CLASS_B_START &__ICFEDIT_region_CLASSB_start__
#define CLASS_B_END &__ICFEDIT_region_CLASSB_end__
#endif /* _IAR_ */

/* Note - Be careful when change the RT_RAM_BLOCKSIZE definition as it is related to the backup buffer size as well as
          to number of steps (rows) necessary to complete the Class B area test overall applied at control flow */
#define RT_RAM_BLOCKSIZE      ((uint16_t)4u)  /* Number of RAM cells tested at once */
#define RT_RAM_BLOCK_OVERLAP  ((uint16_t)1u)  /* Min overlap to cover coupling fault from a tested block to the other */

/* The buffer size must be bigger than the number of tested bytes at single step: it must include also adjacent memory cells
   for buffer self-test to check coupling faults of all RAM buffer bytes used for temporary storage */
#define RT_RAM_BUF_SIZE       (RT_RAM_BLOCKSIZE + 4u * RT_RAM_BLOCK_OVERLAP)

/* constants used at run time program control flow (derived from size of the tested block and size of the tested Class B RAM area */
#define CLASS_B_ROWS (uint16_t)(((uint16_t)(CLASS_B_END) - (uint16_t)(CLASS_B_START) + RT_RAM_BLOCKSIZE) / RT_RAM_BLOCKSIZE)
  /* +1 below is for passing the backup buffer self test */
#define RAM_TEST_COMPLETED (DELTA_ISR * (CLASS_B_ROWS + 1u))

#ifdef _COSMIC_
  /* Number of bytes per run-time Flash CRC calculation */
  uint8_t BLOCKSIZE_s @0x10;// Trick to export constants to asm with immediate addressing
#endif /* _COSMIC_ */
#ifdef _IAR_
  /* uncomment next line to include slow CRC calculation instead of fast one */
  /* #define SLOW_CRC_FLASH_CHECK */

  /* Number of bytes totally tested at Flash CRC calculation from start address */
      /* !!! This value must be fully divided by CRC block size !!! */
  #define CRC_FLASH_SIZE ((uint16_t)(&__checksum_end) - (uint16_t)(&__checksum_begin) + 1u)
  /* Number of bytes tested per one step at run-time Flash CRC calculation */
  #define CRC_BLOCKSIZE 16u
  #define CRC_NUMB_BLOCKS  (CRC_FLASH_SIZE/CRC_BLOCKSIZE)
  #define CRC_INIT  0u
#endif /* _IAR_ */

  /* Constants necessary for Power-on full RAM test at Cosmic (IAR overtakes them from icf files) */
  /* Note user has to adjust SRAM memory range corresponding to conreete product */
  #ifdef _COSMIC_
    #if defined(STM8S208) || defined(STM8S207)
      uint16_t RAM_END_DEC_s @0x17FFu;
      uint16_t RAM_END_INC_s @0x1800u;
    #elif defined(STM8L15X_HD) || defined(STM8TL5X)
      uint16_t RAM_END_DEC_s @0xFFFu;
      uint16_t RAM_END_INC_s @0x1000u;
    #elif defined(STM8S105) || defined(STM8L15X_MD)
      uint16_t RAM_END_DEC_s @0x7FFu;
      uint16_t RAM_END_INC_s @0x800u;
    #elif defined(STM8S103) || defined(STM8S903) || defined(STM8L15X_LD)
      uint16_t RAM_END_DEC_s @0x3FFu;
      uint16_t RAM_END_INC_s @0x400u;
    #elif defined(STM8L10X)
      uint16_t RAM_END_DEC_s @0x5FFu;
      uint16_t RAM_END_INC_s @0x600u;
    #endif /* STM8xxx */
  #endif /* _COSMIC_ */

  /* __stext is the COSMIC compiler entry point, normally executed right after
  reset, but bypassed by self-test routines */
#ifdef _COSMIC_
#define GotoCompilerStartUp() _asm("xdef __stext\n" "jp __stext\n");
//extern @inline void fail_safe_assert(uint8_t err_code, char *verb_txt);
#endif /* _COSMIC_ */
#ifdef _IAR_
extern void __iar_program_start(void);
extern void  __iar_data_init2(void);
extern void main(void);
//extern inline void fail_safe_assert(uint8_t err_code, char *verb_txt);
#define GotoCompilerStartUp() { __iar_start_continue(); }
#endif /* _IAR_ */


/* Exported functions ------------------------------------------------------- */
/* inline function to to control fail safe mode entry */

#endif /* __SELFTEST_PARAM_H */
/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
