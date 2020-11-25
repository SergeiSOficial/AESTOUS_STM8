/**
  ******************************************************************************
  * @file stm8s_it.h
  * @brief This file contains the headers of the interrupt handlers, for Cosmic
  *	compiler.
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
#ifndef __STM8S_IT_H
#define __STM8S_IT_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#ifdef _COSMIC_
 void _stext(void); /* RESET startup routine */
 @far @interrupt void NonHandledInterrupt(void);
 @far @interrupt void TRAP_IRQHandler(void); /* TRAP */
 @far @interrupt void TLI_IRQHandler(void); /* TLI */
 @far @interrupt void AWU_IRQHandler(void); /* AWU */
 @far @interrupt void CLK_IRQHandler(void); /* CLOCK */
 @far @interrupt void EXTI_PORTA_IRQHandler(void); /* EXTI PORTA */
 @far @interrupt void EXTI_PORTB_IRQHandler(void); /* EXTI PORTB */
 @far @interrupt void EXTI_PORTC_IRQHandler(void); /* EXTI PORTC */
 @far @interrupt void EXTI_PORTD_IRQHandler(void); /* EXTI PORTD */
 @far @interrupt void EXTI_PORTE_IRQHandler(void); /* EXTI PORTE */

#ifdef STM8S903
 @far @interrupt void EXTI_PORTF_IRQHandler(void); /* EXTI PORTF */
#endif /*STM8S903*/

#ifdef STM8S208
 @far @interrupt void CAN_RX_IRQHandler(void); /* CAN RX */
 @far @interrupt void CAN_TX_IRQHandler(void); /* CAN TX/ER/SC */
#endif /*STM8S208*/

 @far @interrupt void SPI_IRQHandler(void); /* SPI */
 @far @interrupt void TIM1_CAP_COM_IRQHandler(void); /* TIM1 CAP/COM */
 @far @interrupt void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void); /* TIM1 UPD/OVF/TRG/BRK */

#ifdef STM8S903
 @far @interrupt void TIM5_UPD_OVF_BRK_TRG_IRQHandler(void); /* TIM5 UPD/OVF/BRK/TRG */
 @far @interrupt void TIM5_CAP_COM_IRQHandler(void); /* TIM5 CAP/COM */
#else /*STM8S208, STM8S207, STM8S105 or STM8S103*/
 @far @interrupt void TIM2_UPD_OVF_BRK_IRQHandler(void); /* TIM2 UPD/OVF/BRK */
 @far @interrupt void TIM2_CAP_COM_IRQHandler(void); /* TIM2 CAP/COM */
#endif /*STM8S903*/

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S105)
 @far @interrupt void TIM3_UPD_OVF_BRK_IRQHandler(void); /* TIM3 UPD/OVF/BRK */
 @far @interrupt void TIM3_CAP_COM_IRQHandler(void); /* TIM3 CAP/COM */
#endif /*STM8S208, STM8S207 or STM8S105*/

#ifndef STM8S105
 @far @interrupt void UART1_TX_IRQHandler(void); /* UART1 TX */
 @far @interrupt void UART1_RX_IRQHandler(void); /* UART1 RX */
#endif /*STM8S105*/

 @far @interrupt void I2C_IRQHandler(void); /* I2C */

#ifdef STM8S105
 @far @interrupt void UART2_RX_IRQHandler(void); /* UART2 RX */
 @far @interrupt void UART2_TX_IRQHandler(void); /* UART2 TX */
#endif /* STM8S105*/

#if defined(STM8S207) || defined(STM8S208)
 @far @interrupt void UART3_RX_IRQHandler(void); /* UART3 RX */
 @far @interrupt void UART3_TX_IRQHandler(void); /* UART3 TX */
#endif /*STM8S207, STM8S208*/

#if defined(STM8S207) || defined(STM8S208)
 @far @interrupt void ADC2_IRQHandler(void); /* ADC2 */
#else /*STM8S105, STM8S103 or STM8S903*/
 @far @interrupt void ADC1_IRQHandler(void); /* ADC1 */
#endif /*STM8S207, STM8S208*/

#ifdef STM8S903
 @far @interrupt void TIM6_UPD_OVF_TRG_IRQHandler(void); /* TIM6 UPD/OVF/TRG */
#else /*STM8S208, STM8S207, STM8S105 or STM8S103*/
 @far @interrupt void TIM4_UPD_OVF_IRQHandler(void); /* TIM4 UPD/OVF */
#endif /*STM8S903*/
 @far @interrupt void EEPROM_EEC_IRQHandler(void); /* EEPROM ECC CORRECTION */
#endif /* _COSMIC_ */
#ifdef _COSMIC_
 void _stext(void); /* RESET startup routine */
 @far @interrupt void NonHandledInterrupt(void);
 @far @interrupt void TRAP_IRQHandler(void); /* TRAP */
 @far @interrupt void TLI_IRQHandler(void); /* TLI */
 @far @interrupt void AWU_IRQHandler(void); /* AWU */
 @far @interrupt void CLK_IRQHandler(void); /* CLOCK */
 @far @interrupt void EXTI_PORTA_IRQHandler(void); /* EXTI PORTA */
 @far @interrupt void EXTI_PORTB_IRQHandler(void); /* EXTI PORTB */
 @far @interrupt void EXTI_PORTC_IRQHandler(void); /* EXTI PORTC */
 @far @interrupt void EXTI_PORTD_IRQHandler(void); /* EXTI PORTD */
 @far @interrupt void EXTI_PORTE_IRQHandler(void); /* EXTI PORTE */

#ifdef STM8S903
 @far @interrupt void EXTI_PORTF_IRQHandler(void); /* EXTI PORTF */
#endif /*STM8S903*/

#ifdef STM8S208
 @far @interrupt void CAN_RX_IRQHandler(void); /* CAN RX */
 @far @interrupt void CAN_TX_IRQHandler(void); /* CAN TX/ER/SC */
#endif /*STM8S208*/

 @far @interrupt void SPI_IRQHandler(void); /* SPI */
 @far @interrupt void TIM1_CAP_COM_IRQHandler(void); /* TIM1 CAP/COM */
 @far @interrupt void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void); /* TIM1 UPD/OVF/TRG/BRK */

#ifdef STM8S903
 @far @interrupt void TIM5_UPD_OVF_BRK_TRG_IRQHandler(void); /* TIM5 UPD/OVF/BRK/TRG */
 @far @interrupt void TIM5_CAP_COM_IRQHandler(void); /* TIM5 CAP/COM */
#else /*STM8S208, STM8S207, STM8S105 or STM8S103*/
 @far @interrupt void TIM2_UPD_OVF_BRK_IRQHandler(void); /* TIM2 UPD/OVF/BRK */
 @far @interrupt void TIM2_CAP_COM_IRQHandler(void); /* TIM2 CAP/COM */
#endif /*STM8S903*/

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S105)
 @far @interrupt void TIM3_UPD_OVF_BRK_IRQHandler(void); /* TIM3 UPD/OVF/BRK */
 @far @interrupt void TIM3_CAP_COM_IRQHandler(void); /* TIM3 CAP/COM */
#endif /*STM8S208, STM8S207 or STM8S105*/

#ifndef STM8S105
 @far @interrupt void UART1_TX_IRQHandler(void); /* UART1 TX */
 @far @interrupt void UART1_RX_IRQHandler(void); /* UART1 RX */
#endif /*STM8S105*/

 @far @interrupt void I2C_IRQHandler(void); /* I2C */

#ifdef STM8S105
 @far @interrupt void UART2_RX_IRQHandler(void); /* UART2 RX */
 @far @interrupt void UART2_TX_IRQHandler(void); /* UART2 TX */
#endif /* STM8S105*/

#if defined(STM8S207) || defined(STM8S208)
 @far @interrupt void UART3_RX_IRQHandler(void); /* UART3 RX */
 @far @interrupt void UART3_TX_IRQHandler(void); /* UART3 TX */
#endif /*STM8S207, STM8S208*/

#if defined(STM8S207) || defined(STM8S208)
 @far @interrupt void ADC2_IRQHandler(void); /* ADC2 */
#else /*STM8S105, STM8S103 or STM8S903*/
 @far @interrupt void ADC1_IRQHandler(void); /* ADC1 */
#endif /*STM8S207, STM8S208*/

#ifdef STM8S903
 @far @interrupt void TIM6_UPD_OVF_TRG_IRQHandler(void); /* TIM6 UPD/OVF/TRG */
#else /*STM8S208, STM8S207, STM8S105 or STM8S103*/
 @far @interrupt void TIM4_UPD_OVF_IRQHandler(void); /* TIM4 UPD/OVF */
#endif /*STM8S903*/
 @far @interrupt void EEPROM_EEC_IRQHandler(void); /* EEPROM ECC CORRECTION */
#endif /* _COSMIC_ */

#ifdef _IAR_
	INTERRUPT_HANDLER_TRAP (TRAP_IRQHandler);
	INTERRUPT_HANDLER ( TLI_IRQHandler, 0 );
	INTERRUPT_HANDLER ( AWU_IRQHandler, 1 );
	INTERRUPT_HANDLER ( CLK_IRQHandler, 2 );
	INTERRUPT_HANDLER ( EXTI_PORTA_IRQHandler, 3 );
	INTERRUPT_HANDLER ( EXTI_PORTB_IRQHandler, 4 );
	INTERRUPT_HANDLER ( EXTI_PORTC_IRQHandler, 5 );
	INTERRUPT_HANDLER ( EXTI_PORTD_IRQHandler, 6 );
	INTERRUPT_HANDLER ( EXTI_PORTE_IRQHandler, 7 );
	INTERRUPT_HANDLER ( EXTI_PORTF_IRQHandler, 8 );
	INTERRUPT_HANDLER ( CAN_RX_IRQHandler, 8 );
	INTERRUPT_HANDLER ( CAN_TX_IRQHandler, 9 );
	INTERRUPT_HANDLER ( SPI_IRQHandler, 10 );
	INTERRUPT_HANDLER ( TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11 );
	INTERRUPT_HANDLER ( TIM1_CAP_COM_IRQHandler, 12 );
	INTERRUPT_HANDLER ( TIM5_UPD_OVF_BRK_TRG_IRQHandler, 13 );
	INTERRUPT_HANDLER ( TIM2_UPD_OVF_BRK_IRQHandler, 13 );
	INTERRUPT_HANDLER ( TIM5_CAP_COM_IRQHandler, 14 );
	INTERRUPT_HANDLER ( TIM2_CAP_COM_IRQHandler, 14 );
	INTERRUPT_HANDLER ( TIM3_UPD_OVF_BRK_IRQHandler, 15 );
	INTERRUPT_HANDLER ( TIM3_CAP_COM_IRQHandler, 16 );
	INTERRUPT_HANDLER ( UART1_TX_IRQHandler, 17 );
	INTERRUPT_HANDLER ( UART1_RX_IRQHandler, 18 );
	INTERRUPT_HANDLER ( I2C_IRQHandler, 19 );
	INTERRUPT_HANDLER ( UART2_TX_IRQHandler, 20 );
	INTERRUPT_HANDLER ( UART2_RX_IRQHandler, 21 );
	INTERRUPT_HANDLER ( UART3_TX_IRQHandler, 20 );
	INTERRUPT_HANDLER ( UART3_RX_IRQHandler, 21 );
	INTERRUPT_HANDLER ( ADC2_IRQHandler, 22 );
	INTERRUPT_HANDLER ( ADC1_IRQHandler, 22 );
	INTERRUPT_HANDLER ( TIM4_UPD_OVF_IRQHandler, 23 );
	INTERRUPT_HANDLER ( TIM6_UPD_OVF_TRG_IRQHandler, 23 );
	INTERRUPT_HANDLER ( EEPROM_EEC_IRQHandler, 24 );
#endif /* _IAR_ */

#endif /* __STM8S_IT_H */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
