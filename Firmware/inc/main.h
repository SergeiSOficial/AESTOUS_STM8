/**
  ******************************************************************************
  * @file main.h
  * @brief This file contains the external declaration of main.c file.
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
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

#if defined(STL_VERBOSE_POR) || defined(STL_VERBOSE_RUN) || defined(STL_VERBOSE_FAILSAFE)
  #include <stdio.h>
#endif  /* STL_VERBOSE */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Evalboard I/Os configuration */

#if defined(STM8S003)
  #define LEDS_PORT (GPIOD)
  #define TEST_PORT (GPIOB)
#endif /* STM8S105 STM8S103 */
#define TEST_PIN  (0x20u)
#define LED1_PIN  (0x10u)
#define LED2_PIN  (0x10u)
#define LED3_PIN  (0x10u)
#define LED4_PIN  (0x10u)
#define ALL_LEDs  (LED1_PIN)

#define LED_ERR LED4_PIN
#define LED_NVM LED3_PIN
#define LED_VLM LED2_PIN

/* Exported macro ------------------------------------------------------------*/
/* Exported inline functions ------------------------------------------------ */
#ifdef EVAL_BOARD_CONTROL
  /* inline functions to control Class B demo on evaluation board */
  INLINE_FUNCTION void gpio_LED_init(void)
  {
    LEDS_PORT->DDR |= (ALL_LEDs);
    LEDS_PORT->CR1 |= (ALL_LEDs);
    LEDS_PORT->ODR &= ~(ALL_LEDs);
  }
  /* ---------------------------------------------------------------------------*/
  INLINE_FUNCTION void gpio_test_init(void)
  {
    TEST_PORT->DDR |= TEST_PIN;
    TEST_PORT->CR1 |= TEST_PIN;
    TEST_PORT->ODR &= ~TEST_PIN;
  }
  /* ---------------------------------------------------------------------------*/
  INLINE_FUNCTION void toogle_test_pin(void)
  {
    TEST_PORT->ODR ^= TEST_PIN;
  }
  /* ---------------------------------------------------------------------------*/
  INLINE_FUNCTION void BSP_LED_On(uint8_t led)
  {
    LEDS_PORT->ODR |= (led);
  }
  /* ---------------------------------------------------------------------------*/
  INLINE_FUNCTION void BSP_LED_Off(uint8_t led)
  {
    LEDS_PORT->ODR &= ~(led);
  }
  /* ---------------------------------------------------------------------------*/
 INLINE_FUNCTION void BSP_LED_Toogle(uint8_t led)
  {
    LEDS_PORT->ODR ^= (led);
  }
#endif /* EVAL_BOARD_CONTROL */

/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H__ */

/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
