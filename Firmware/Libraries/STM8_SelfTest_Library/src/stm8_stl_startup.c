/**
  ******************************************************************************
  * @file stm8_stl_startup.c
  * @brief This file contains all calls to the routines to be executed at start
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
#include "main.h"

#if defined (EVAL_BOARD_LCD)
  #if defined(STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD)
    #include "stm8_eval_lcd.h"
  #else
    #include "mono_lcd_wl.h"
  #endif /* STM8L15xx */
#endif /* EVAL_BOARD_LCD */ 

#define ALLOC_GLOBALS
#include "stm8_stl_classB_var.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global inline functions declaration ---------------------------------------*/
#ifdef _IAR_
  extern void refresh_iwdog(void);
  extern void refresh_wwdog(uint8_t cnt, uint8_t win);
  extern void fail_safe_assert(uint16_t err_code, char *verb_txt);
#endif /* _IAR_ */
/* Private function prototypes -----------------------------------------------*/
#ifdef _COSMIC_
  extern int _classb_checksum160(void);
  extern @far int _classb_checksum161(void);
  extern int _classb_checksum80(void);
  extern @far int _classb_checksum81(void);
  extern void STL_StartUp (void);
#endif /* _COSMIC_ */
#ifdef _IAR_
  extern uint16_t FlashCRC16_StartUp_CheckSum(void);
  extern void __iar_start_continue(void);
  extern void STL_StartUp (void);
#endif /* _IAR_ */ 

/* Private functions ---------------------------------------------------------*/
/**
  * @brief This routine executed in case of failure detected during
  *        one of the self-test routines
  * @par Parameters:
  * None
  * @retval
  * None
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * None
  */
void FailSafe(uint16_t err_no)
{
  disableInterrupts();
  #ifdef STL_VERBOSE_FAILSAFE
    #ifdef STL_INCL_HSECSS
      switch_clock_system(to_HSI);
    #endif /* STL_VERBOSE_FAILSAFE */
    STL_VerboseInit();	
    printf("\n\r>>>>>> FailSafe Mode (Error Code=%02X)\n\r", err_no);
  #endif  /* STL_VERBOSE_FAILSAFE */
    
  #if defined (EVAL_BOARD_CONTROL)
    BSP_LED_On(LED_ERR);
  #endif /* EVAL_BOARD_CONTROL */

  while (1)
  {
    #ifdef DEBUG
      refresh_iwdog();     /* reload & start independent wdog */
      #if !defined(STM8L10X) && !defined(STM8TL5X)
        refresh_wwdog(0x7Fu, 0x7Fu);
      #endif /* !STM8L10X && !STM8TL5X */
    #endif /* DEBUG */
  }
}

#ifdef STL_INCL_POR
/* ---------------------------------------------------------------------------*/
/**
  * @brief Calls the very first test routines executed right after the reset
  *        Output: Zero initialized RAM, CPU clock provided by the HSE
  *                Flash interface initialized, Systick timer ON (2ms timebase)
  * @par Parameters:
  * None
  * @retval
  * None
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * - STL_StartUpCPUTest()
  * - STL_WDGSelfTest()
  * - _classb_checksum16x()
  * - STL_FullRamMarchC()
  * - STL_ClockStartUpTest()
  * - printf()
  * - FailSafe()
  */
void STL_StartUp (void)
{
  /* workarround - some STMS207 derivates doesn't reset SP correctly */
  #ifdef _COSMIC_
    _asm("xref __stack");
    _asm("ldw x,#__stack");
    _asm("ldw sp,x");
  #endif /* _COSMIC_ */
    
  /*High speed internal clock prescaler: 1*/
  #if defined(STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD)
    CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_CKM);
  #else
    CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_HSIDIV);
  #endif /* STM8L15xx */
  
  #ifdef STL_VERBOSE_POR
    STL_VerboseInit();
    #if defined(STM8S208) || defined(STM8S207)
      printf("\n\r*** STM8S20x Self Test Library Init *** Rev2.0.0 @");
    #elif defined(STM8S105) || defined(STM8S005)
      printf("\n\r*** STM8Sx05 Self Test Library Init *** Rev2.0.0 @");
    #elif defined(STM8S103) || defined(STM8S003)
      printf("\n\r*** STM8Sx03 Self Test Library Init *** Rev2.0.0 @");
    #elif defined(STM8S903)
      printf("\n\r*** STM8S903 Self Test Library Init *** Rev2.0.0 @");
    #elif defined(STM8AF52x)
      printf("\n\r*** STM8AF52x Self Test Library Init *** Rev2.0.0 @");
    #elif defined(STM8AF62x)
      printf("\n\r*** STM8AF62x Self Test Library Init *** Rev2.0.0 @");
    #elif defined(STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD)
      printf("\n\r*** STM8L15x Self Test Library Init *** Rev2.0.0 @");
    #elif defined(STM8L10X)
      printf("\n\r*** STM8L10x Self Test Library Init *** Rev2.0.0 @");
    #elif defined(STM8TL5X)
      printf("\n\r*** STM8TL5x Self Test Library Init *** Rev2.0.0 @");
    #elif defined(STM8A626x)
      printf("\n\r*** STM8A626x Self Test Library Init *** Rev2.0.0 @");
    #endif /* STM8xxx */
    #ifdef _COSMIC_
      printf("Cosmic\n\r");
    #endif /* _COSMIC_ */
    #ifdef _IAR_
      printf("IAR\n\r");
    #endif /* _IAR_ */
      
  #endif  /* STL_VERBOSE_POR */
  
  #ifdef STL_CCO_ENABLE  
    #if defined(STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD)
      GPIOC->CR1 |= 0x10;                       /* push-pull on PC4 */
    #endif /* STM8L15xx */
    CLK->CCOR= (0x0c<<1) | CLK_CCOR_CCOEN; /* FMASTER to CCO output (PC4/PD0) */
  #endif /* STL_CCO_ENABLE */
    
  #if defined (EVAL_BOARD_CONTROL)
    gpio_LED_init();
    gpio_test_init();
  #endif /* EVAL_BOARD_CONTROL */

  /*--------------------------------------------------------------------------*/
  /*------------------- CPU registers and Flags Self Test --------------------*/
  /*--------------------------------------------------------------------------*/
  /* WARNING: all registers destroyed when exiting this function */

  /* Initializes counter for control flow monitoring */
  CtrlFlowCnt = CPU_POR_CALLER;
  CtrlFlowCntInv = 0xFFFFu;

  #ifdef STL_INCL_POR_CPU
    if (STL_StartUpCPUTest() != CPUTEST_SUCCESS)
    {
      fail_safe_assert( 0x00u, "Start-up CPU Test Failure\n\r");
    }
    else  /* Test OK */
    {
      /* If else statement is not executed, it will be detected by control flow monitoring */
      CtrlFlowCntInv -= CPU_POR_CALLER;
      #ifdef STL_VERBOSE_POR
        printf("\n\rStart-up CPU Test OK\n\r");
      #endif /* STL_VERBOSE_POR */
    }
  #endif /* STL_INCL_POR_CPU */

  /*--------------------------------------------------------------------------*/
  /*--------------------- Watchdog check and reload -------------------------*/
  /*--------------------------------------------------------------------------*/
  IWDG->KR = 0xCCu;     /* IWDG Enable */
  IWDG->KR = 0x55u;     /* IWDG WriteAccess Enable */
  IWDG->PR = 0x04u;     /* IWDG Prescaler to 64 */
  IWDG->RLR = 0xFFu;    /* set a 255ms timeout period */
  refresh_iwdog();        /* reload & start independent wdog */

  #ifdef STL_INCL_POR_WDOG
    CtrlFlowCnt += WDG_TEST_CALLER;
    STL_WDGSelfTest();
    CtrlFlowCntInv -= WDG_TEST_CALLER;
  #endif /* STL_INCL_POR_WDOG */

  /*--------------------------------------------------------------------------*/
  /*--------------------- Invariable memory CRC check ------------------------*/
  /*--------------------------------------------------------------------------*/
  #ifdef STL_INCL_POR_FLASH
    #if defined (EVAL_BOARD_CONTROL)
      BSP_LED_On(LED_NVM);
    #endif /* EVAL_BOARD_CONTROL */
  
    /* Regular 16-bit crc computation */
    CtrlFlowCnt += CRC16_TEST_CALLER;
  
    #ifdef _COSMIC_ 
      #ifndef CRC_CHECK_8
        #ifndef CRC_CHECK_FAR
          #ifdef DEBUG
            _classb_checksum160(); /* flash check result is executed */
            if ( FALSE ) 							 /* but not verified at debug mode with Cosmic */
          #else
            if ( _classb_checksum160() )
          #endif /* DEBUG */
        #else
          #ifdef DEBUG
            _classb_checksum161();
            if ( FALSE )
          #else
            if ( _classb_checksum161() )
          #endif /* DEBUG */
        #endif /* CRC_CHECK_FAR */
      #else
        #ifndef CRC_CHECK_FAR
          #ifdef DEBUG
            _classb_checksum80();
            if ( FALSE )
          #else
            if ( _classb_checksum80() )
          #endif /* DEBUG */
        #else
          #ifdef DEBUG
            _classb_checksum81();
            if ( FALSE )
          #else
            if ( _classb_checksum81() )
          #endif /* DEBUG */
        #endif /* CRC_CHECK_FAR */
      #endif /* CRC_CHECK_8 */
    #endif /* _COSMIC_ */
  
    #ifdef _IAR_
      #ifndef CRC_CHECK_8
        if ( FlashCRC16_StartUp_CheckSum() != CRC_OK )
      #else
        #error "8-bit CRC is not supported"
      #endif /* CRC_CHECK_8 */
    #endif /* _IAR_ */
      {
        #ifndef CRC_CHECK_8
          fail_safe_assert( 0x01u, "FLASH 16-bit CRC Error at Start-up\n\r");
        #else
          fail_safe_assert( 0x01u, "FLASH 8-bit CRC Error at Start-up\n\r");
        #endif /* CRC_CHECK_8 */
        /* If else statement is not executed, it will be detected by control flow monitoring */
      }
      else  /* Test OK */
      {
        CtrlFlowCntInv -= CRC16_TEST_CALLER;
        #ifdef STL_VERBOSE_POR
          #ifndef CRC_CHECK_8
            printf("Start-up FLASH 16-bit CRC OK\n\r");
          #else
            printf("Start-up FLASH 8-bit CRC OK\n\r");
          #endif /* CRC_CHECK_8 */
        #endif  /* STL_VERBOSE_POR */
      }
      
    #if defined (EVAL_BOARD_CONTROL)
      BSP_LED_Off(LED_NVM);
    #endif /* EVAL_BOARD_CONTROL */
  #endif /* STL_INCL_POR_FLASH */

  /*--------------------------------------------------------------------------*/
  /*   Verify Control flow before RAM init (which clears Ctrl flow counters)  */
  /*--------------------------------------------------------------------------*/
  if (((CtrlFlowCnt ^ CtrlFlowCntInv) != 0xFFFFu)
  || (CtrlFlowCnt != CHECKPOINT1 ))
  {
    fail_safe_assert( 0x02u, "Control Flow Error (Start-up 1)");
  }
  else
  {
    #ifdef STL_VERBOSE_POR
      printf("Control Flow OK (Start-up 1)\n\r");
    #endif  /* STL_VERBOSE_POR */
  }

  /*--------------------------------------------------------------------------*/
  /* --------------------- Variable memory functional test -------------------*/
  /*--------------------------------------------------------------------------*/ 
  #ifdef STL_INCL_POR_RAM
    #if defined (EVAL_BOARD_CONTROL)
      BSP_LED_On(LED_VLM);
    #endif /* EVAL_BOARD_CONTROL */
    
    /* WARNING: All the stack space is zero-initialized when exiting from this routine */
    if (STL_FullRamMarchC() != FULL_RAM_OK)
    {
      fail_safe_assert( 0x03u, "RAM Test Failure");
    }
    #ifdef STL_VERBOSE_POR
      printf("Full RAM Test OK\n\r");
    #endif /* STL_VERBOSE_POR */
  
    /* Note, control flow variables are cleared inside the RAM test routine and  */
    /* re-initialized before return to check correct passing of the test */
  
    #if defined (EVAL_BOARD_CONTROL)
      BSP_LED_Off(LED_VLM);
    #endif /* EVAL_BOARD_CONTROL */
        
  #else /* test is not included */
    CtrlFlowCnt = 0u;     /* reinit control flow variables */
    CtrlFlowCntInv = 0xFFFFu;  
  #endif /* STL_INCL_POR_RAM */

  /*--------------------------------------------------------------------------*/
  /*----------------------- Clock Frequency Self Test ------------------------*/
  /*--------------------------------------------------------------------------*/
  #ifdef STL_INCL_POR_CLOCK
    #if defined (EVAL_BOARD_CONTROL)
      BSP_LED_On(LED_NVM);
    #endif /* EVAL_BOARD_CONTROL */
      
    CtrlFlowCnt += CLOCK_POR_CALLER;
    
    switch ( STL_ClockStartUpTest() )
    {
      case FREQ_OK:
        #ifdef STL_VERBOSE_POR
          printf("Clock frequency OK\n\r");
        #endif  /* STL_VERBOSE_POR */
        break;
  
  
      case HSI_SOURCE_FAIL:
        fail_safe_assert( 0x04u, "HSI clock source failure");
        break;
        
      case LSI_START_FAIL:
        fail_safe_assert( 0x05u, "LSI start-up failure");
        break;
  
      case HSE_START_FAIL:
        fail_safe_assert( 0x06u, "HSE start-up failure");
        break;
  
      case HSI_HSE_SWITCH_FAIL:
        fail_safe_assert( 0x07u, "Clock switch failure");
        break;
  
      case EXT_SOURCE_FAIL:
        fail_safe_assert( 0x08u, "EXT clock source failure");
        break;
  
      case TEST_ONGOING:
      default:
        fail_safe_assert( 0x09u, "Abnormal Clock Test routine termination (POR)");
        break;
    }
    CtrlFlowCntInv -= CLOCK_POR_CALLER;
    
    #if defined (EVAL_BOARD_CONTROL)
      BSP_LED_Off(LED_NVM);
    #endif /* EVAL_BOARD_CONTROL */
  #else /* test is not included */
    enableInterrupts();
  #endif /* STL_INCL_POR_CLOCK */

  /*--------------------------------------------------------------------------*/
  /* -----  Verify Control flow before Starting main program execution ------ */
  /*--------------------------------------------------------------------------*/
  if (((CtrlFlowCnt ^ CtrlFlowCntInv) != 0xFFFFu) || (CtrlFlowCnt != CHECKPOINT2))
  {
    fail_safe_assert( 0x0Au, "Control Flow Error (Start-up 2)");
  }
  #ifdef STL_VERBOSE_POR
    printf("Control Flow OK (Start-up 2)\n\r");
  #endif  /* STL_VERBOSE_POR */
  
  GotoCompilerStartUp()
}
#endif /* STL_INCL_POR */


#if defined(STL_VERBOSE_POR) || defined(STL_VERBOSE_RUN) || defined(STL_VERBOSE_FAILSAFE)
/* ---------------------------------------------------------------------------*/
/**
  * @brief Initializes the UART1 and few I/Os for test purposes
  * @par Parameters:
  * None
  * @retval
  * None
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * None
  */
void STL_VerboseInit(void)
{
 /* UART configuration ------------------------------------------------------*/
/*        - BaudRate = 115200 baud
          - Word Length = 8 Bits
          - One Stop Bit
          - No parity
          - Receive and transmit enabled
          - USART Clock disabled  */
 
  
  #if defined(STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD)
    CLK->PCKENR3 |= CLK_PCKENR3_USART2; /* enable system clock to feed the peripheral */
    USART2->CR2 &= ~USART_CR2_TEN;  /* transmit disable */
    USART2->CR1 = 0;		/* 8 data bits, no parity */
    USART2->CR3 = 0;	        /* 1 stop bit */
      
    if (CLK->SWR == to_HSE)      /* 115200Bd */ 
    {
      USART2->BRR2 = 0x0bu;	/* at 16MHz (HSE max) */
      USART2->BRR1 = 0x08u;
    }
    else
    {
      if ((CLK->CKDIVR & CLK_CKDIVR_CKM) == 0)
      {
        USART2->BRR2 = 0x0bu;     /* at 16MHz (HSI max) */
        USART2->BRR1 = 0x08u;
      }
      else
      {
        USART2->BRR2 = 0x01u;	/* at 2MHz (HSI after reset) */
        USART2->BRR1 = 0x01u;
      }
    }
    USART2->CR2 = USART_CR2_TEN;  /* transmit enable */
  #endif /* STM8L15X_MD || STM8L15X_MDP || STM8L15X_HD */

  #if defined(STM8L10X) || defined(STM8TL5X)
    USART_PORT->CR1 |= USART_TX_PIN; /* STM8L101-EVAL needs to setup TX pin as push pull */
    CLK->PCKENR |= CLK_PCKENR_USART; /* enable system clock to feed the peripheral */
    USART->CR2 &= ~USART_CR2_TEN;  /* transmit disable */
    USART->CR1 = 0;	    /* 8 data bits, no parity */
    USART->CR3 = 0;	    /* 1 stop bit */
    if ((CLK->CKDIVR & CLK_CKDIVR_HSIDIV) == 0)
    {
      USART->BRR2 = 0x0bu;  /* at 16MHz (HSI max) */
      USART->BRR1 = 0x08u;
    }
    else
    {
      USART->BRR2 = 0x01u;  /* at 2MHz (HSI after reset) */
      USART->BRR1 = 0x01u;
    }   
    USART->CR2 = USART_CR2_TEN;  /*  transmit enable */
  #endif /* STM8L10x STM8TL5x */
  
  #if defined(STM8S208) || defined(STM8S207)
    UART1->CR2 &= (uint8_t)(~UART1_CR2_TEN);  /* transmit disable */
    UART1->CR1 = 0u;	      /* 8 data bits, no parity */
    UART1->CR3 = 0u;	      /* 1 stop bit */

    if (CLK->SWR == to_HSE)    /* 115200Bd */
    {
      UART1->BRR2 = 0x00u;     /* at 12MHz (HSE/2 no wait state) */
      UART1->BRR1 = 0x0du;
    }
    else
    {
      if ((CLK->CKDIVR & CLK_CKDIVR_HSIDIV) == 0u)
      {
        UART1->BRR2 = 0x0bu;   /* at 16MHz (HSI max) */
        UART1->BRR1 = 0x08u;
      }
      else
      {
        UART1->BRR2 = 0x01u;    /*  at 2MHz (HSI after reset)*/
        UART1->BRR1 = 0x01u;
      }
    }
    UART1->CR2 = UART1_CR2_TEN;  /* transmit enable */
  #endif /* STM8S208, STM8S207 */
  
  #if defined(STM8S103) || defined(STM8S903)
    UART1->CR2 &= ~UART1_CR2_TEN;  /* transmit disable */
    UART1->CR1 = 0;	    /* 8 data bits, no parity */
    UART1->CR3 = 0;	    /* 1 stop bit */
    if ((CLK->CKDIVR & CLK_CKDIVR_HSIDIV) == 0)
    {
      UART1->BRR2 = 0x0bu;  /* at 16MHz (HSI max) */
      UART1->BRR1 = 0x08u;
    }
    else
    {
      UART1->BRR2 = 0x01u;  /* at 2MHz (HSI after reset) */
      UART1->BRR1 = 0x01u;
    }   
    UART1->CR2 = UART1_CR2_TEN;  /*  transmit enable */
  #endif /* STM8S208, STM8S207, STM8S103, STM8S903 */
  
  #if defined(STM8S105)
    UART2->CR2 &= ~UART2_CR2_TEN;  /* transmit disable */
    UART2->CR1 = 0;		/* 8 data bits, no parity */
    UART2->CR3 = 0;	        /* 1 stop bit */
      
    if (CLK->SWR == to_HSE)      /* 115200Bd */ 
    {
      UART2->BRR2 = 0x0bu;	/* at 16MHz (HSE max) */
      UART2->BRR1 = 0x08u;
    }
    else
    {
      if ((CLK->CKDIVR & CLK_CKDIVR_HSIDIV) == 0)
      {
        UART2->BRR2 = 0x0bu;     /* at 16MHz (HSI max) */
        UART2->BRR1 = 0x08u;
      }
      else
      {
        UART2->BRR2 = 0x01u;	/* at 2MHz (HSI after reset) */
        UART2->BRR1 = 0x01u;
      }
    }
    UART2->CR2 = UART2_CR2_TEN;  /* transmit enable */
  #endif /* STM8S105 */
}
#endif  /* any_verbose_defined */

/* ---------------------------------------------------------------------------*/
/**
  * @brief Verifies the watchdog circuitry by forcing watchdog resets
  * @par Parameters:
  * None
  * @retval
  * None
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * None
  */
void STL_WDGSelfTest(void)
{
  #ifdef STL_VERBOSE_POR
    printf("-> ");
    #if defined(STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD)
      if ((RST->SR & RST_SR_BORF) == RST_SR_BORF)     { printf("BOR "); }
      if ((RST->SR & RST_SR_WWDGF) == RST_SR_WWDGF)   { printf("WWDG "); }
      if ((RST->SR & RST_SR_SWIMF) == RST_SR_SWIMF)   { printf("SWIM "); }
      if ((RST->SR & RST_SR_ILLOPF) == RST_SR_ILLOPF) { printf("ILLOP "); }
      if ((RST->SR & RST_SR_IWDGF) == RST_SR_IWDGF)   { printf("IWDG "); }
      if ((RST->SR & RST_SR_PORF) == RST_SR_PORF)     { printf("POR "); }
    #elif defined(STM8L10X) || defined (STM8TL5X)
      if ((RST->SR & RST_SR_SWIMF) == RST_SR_SWIMF)   { printf("SWIM "); }
      if ((RST->SR & RST_SR_ILLOPF) == RST_SR_ILLOPF) { printf("ILLOP "); }
      if ((RST->SR & RST_SR_IWDGF) == RST_SR_IWDGF)   { printf("IWDG "); }
      if ((RST->SR & RST_SR_PORF) == RST_SR_PORF)     { printf("POR "); }
    #else /* STM8AS */
      if ((RST->SR & RST_SR_EMCF) == RST_SR_EMCF)     { printf("EMC "); }
      if ((RST->SR & RST_SR_SWIMF) == RST_SR_SWIMF)   { printf("SWIM "); }
      if ((RST->SR & RST_SR_ILLOPF) == RST_SR_ILLOPF) { printf("ILLOP "); }
      if ((RST->SR & RST_SR_IWDGF) == RST_SR_IWDGF)   { printf("IWDG "); }
      if ((RST->SR & RST_SR_WWDGF) == RST_SR_WWDGF)   { printf("WWDG "); }
    #endif /* STM8xxx */
    printf("reset\n\r");
  #endif /* STL_VERBOSE_POR */

  /* if not resuming from a independant watchdog reset, start watchdogs test */
  if ((RST->SR & RST_SR_IWDGF) == 0u)
  {
    #ifdef STL_VERBOSE_POR
      printf("... Power-on or WWDG reset, testing IWDG ...\r\n");
    #endif  /* STL_VERBOSE_POR */
		
    #if !defined(STM8L10X)  && !defined(STM8TL5X)
      RST->SR |= RST_SR_WWDGF;   /* Re-test always WWDG if applied */
    #endif /* !STM8L10X && !STM8TL5X */
    
    IWDG->KR = (uint8_t)0xCCu;    /* IWDG Enable */
    IWDG->KR = (uint8_t)0x55u;    /* IWDG WriteAccess Enable */
    IWDG->PR = (uint8_t)0;        /* IWDG Prescaler to 4 */
    IWDG->RLR = (uint8_t)0;       /* set the shortest timeout period */
    refresh_iwdog();         /* reload & start independent wdog */    
    /* Wait for an independent watchdog reset */
    while (1)
    { }
  }
  #if !defined(STM8L10X)  && !defined(STM8TL5X)
    else  /* Watchdog test or software reset triggered by application failure */
    {
      /* If only IWDG flag was set, means that watchdog test sequence is on-going */
      if ((RST->SR & RST_SR_WWDGF) == 0u)
      { /* If IWDG only was set, test WWDG */
        #ifdef STL_VERBOSE_POR
          printf("... IWDG reset from self-test or application, testing WWDG ...\r\n");
        #endif  /* STL_VERBOSE_POR */
        
        refresh_wwdog(0x40u, 0x7Fu);    /* init wwdog */
      /* Wait for window watchdog reset */
        while (1)
        { }
      }
      else  /* If both flags are set, means that watchdog test is completed */
      {
        RST->SR |= (uint8_t)(RST_SR_WWDGF | RST_SR_IWDGF);   /* clear both flags */
        #ifdef STL_VERBOSE_POR
          printf("... IWDG and WWDG reset, WDG test completed ...\r\n");
        #endif  /* STL_VERBOSE_POR */
      } /* End of normal test sequence */
    } /* End of partial WDG test (IWDG test done) */
  #else /* wwdog is not tested at STM8L 8K */
    else /* watchdog test is completed */
    {
      RST->SR |= (uint8_t)(RST_SR_IWDGF);   /* clear the flag */
      #ifdef STL_VERBOSE_POR
        printf("... IWDG reset, WDG test completed ...\r\n");
      #endif  /* STL_VERBOSE_POR */
    } /* End of normal test sequence */
  #endif /* !STM8L10X && !STM8TL5X */

}

/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
