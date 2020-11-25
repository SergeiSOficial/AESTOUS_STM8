/**
  ******************************************************************************
  * @file stm8_stl_crc16_IAR.c
  * @brief This file contains computation of crc16 rom memory (provided by IAR)
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
#if defined(STM8L15X_MD) || defined (STM8L15X_MDP) || defined (STM8L15X_HD)
  #include "stm8l15x.h"
#elif defined(STM8L10X)
  #include "stm8l10x.h"
#elif defined(STM8TL5X)
  #include "stm8tl5x.h"
#else
  #include "stm8s.h"
#endif /* STM8xxx */
#include "stm8_stl_lib.h"
#include "stm8_stl_classB_var.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
extern const uint8_t __checksum_begin;
extern const uint8_t __checksum_end;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern const uint16_t __checksum;

/* Private function prototypes -----------------------------------------------*/
uint16_t slow_crc16(uint16_t sum, const uint8_t *p, uint16_t len);
uint16_t fast_crc16(uint16_t sum, const uint8_t *p, uint16_t len);
uint16_t FlashCRC16_StartUp_CheckSum(void);
uint8_t FlashCrc16_Block_test(void);
/* Private functions ---------------------------------------------------------*/
#ifdef SLOW_CRC_FLASH_CHECK
/**
  * @brief This routine calculates CRC16 check sum of defined block at memory
  * @par Parameters:
  * Sum initialization, pointer to begin of block, length of the block
  * @retval
  * CRC16 check sum of calculated block
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * None
  */
uint16_t slow_crc16(uint16_t sum, const uint8_t *p, uint16_t len)
{
  uint8_t i, byte;
  uint16_t oSum;
  
  while (len--)
  {
  /* ==============================================================================*/
  /* MISRA violation of rule 17.4 - pointer arithmetic is used for CRC calculation */
  #pragma diag_suppress=Pm088
    byte = *(p++);
  #pragma diag_default=Pm088
  /* ==============================================================================*/
  
    for (i = 0u; i < 8u; ++i)
    {
      oSum = sum;
      sum <<= 1;
      if (byte & (uint8_t)0x80)
      {
        sum |= 1u;
      }
      if (oSum & 0x8000)
      {
        sum ^= 0x1021u;
      }
      byte <<= 1;
    }
  }
  return sum;
}
#else
/* reference data table for fast_crc16 CRC calculation procedure */
const uint16_t crc16_table[256] =
  {
    0x0000u, 0x1021u, 0x2042u, 0x3063u, 0x4084u, 0x50a5u, 0x60c6u, 0x70e7u, 0x8108u,
    0x9129u, 0xa14au, 0xb16bu, 0xc18cu, 0xd1adu, 0xe1ceu, 0xf1efu, 0x1231u, 0x0210u,
    0x3273u, 0x2252u, 0x52b5u, 0x4294u, 0x72f7u, 0x62d6u, 0x9339u, 0x8318u, 0xb37bu,
    0xa35au, 0xd3bdu, 0xc39cu, 0xf3ffu, 0xe3deu, 0x2462u, 0x3443u, 0x0420u, 0x1401u,
    0x64e6u, 0x74c7u, 0x44a4u, 0x5485u, 0xa56au, 0xb54bu, 0x8528u, 0x9509u, 0xe5eeu,
    0xf5cfu, 0xc5acu, 0xd58du, 0x3653u, 0x2672u, 0x1611u, 0x0630u, 0x76d7u, 0x66f6u,
    0x5695u, 0x46b4u, 0xb75bu, 0xa77au, 0x9719u, 0x8738u, 0xf7dfu, 0xe7feu, 0xd79du,
    0xc7bcu, 0x48c4u, 0x58e5u, 0x6886u, 0x78a7u, 0x0840u, 0x1861u, 0x2802u, 0x3823u,
    0xc9ccu, 0xd9edu, 0xe98eu, 0xf9afu, 0x8948u, 0x9969u, 0xa90au, 0xb92bu, 0x5af5u,
    0x4ad4u, 0x7ab7u, 0x6a96u, 0x1a71u, 0x0a50u, 0x3a33u, 0x2a12u, 0xdbfdu, 0xcbdcu,
    0xfbbfu, 0xeb9eu, 0x9b79u, 0x8b58u, 0xbb3bu, 0xab1au, 0x6ca6u, 0x7c87u, 0x4ce4u,
    0x5cc5u, 0x2c22u, 0x3c03u, 0x0c60u, 0x1c41u, 0xedaeu, 0xfd8fu, 0xcdecu, 0xddcdu,
    0xad2au, 0xbd0bu, 0x8d68u, 0x9d49u, 0x7e97u, 0x6eb6u, 0x5ed5u, 0x4ef4u, 0x3e13u,
    0x2e32u, 0x1e51u, 0x0e70u, 0xff9fu, 0xefbeu, 0xdfddu, 0xcffcu, 0xbf1bu, 0xaf3au,
    0x9f59u, 0x8f78u, 0x9188u, 0x81a9u, 0xb1cau, 0xa1ebu, 0xd10cu, 0xc12du, 0xf14eu,
    0xe16fu, 0x1080u, 0x00a1u, 0x30c2u, 0x20e3u, 0x5004u, 0x4025u, 0x7046u, 0x6067u,
    0x83b9u, 0x9398u, 0xa3fbu, 0xb3dau, 0xc33du, 0xd31cu, 0xe37fu, 0xf35eu, 0x02b1u,
    0x1290u, 0x22f3u, 0x32d2u, 0x4235u, 0x5214u, 0x6277u, 0x7256u, 0xb5eau, 0xa5cbu,
    0x95a8u, 0x8589u, 0xf56eu, 0xe54fu, 0xd52cu, 0xc50du, 0x34e2u, 0x24c3u, 0x14a0u,
    0x0481u, 0x7466u, 0x6447u, 0x5424u, 0x4405u, 0xa7dbu, 0xb7fau, 0x8799u, 0x97b8u,
    0xe75fu, 0xf77eu, 0xc71du, 0xd73cu, 0x26d3u, 0x36f2u, 0x0691u, 0x16b0u, 0x6657u,
    0x7676u, 0x4615u, 0x5634u, 0xd94cu, 0xc96du, 0xf90eu, 0xe92fu, 0x99c8u, 0x89e9u,
    0xb98au, 0xa9abu, 0x5844u, 0x4865u, 0x7806u, 0x6827u, 0x18c0u, 0x08e1u, 0x3882u,
    0x28a3u, 0xcb7du, 0xdb5cu, 0xeb3fu, 0xfb1eu, 0x8bf9u, 0x9bd8u, 0xabbbu, 0xbb9au,
    0x4a75u, 0x5a54u, 0x6a37u, 0x7a16u, 0x0af1u, 0x1ad0u, 0x2ab3u, 0x3a92u, 0xfd2eu,
    0xed0fu, 0xdd6cu, 0xcd4du, 0xbdaau, 0xad8bu, 0x9de8u, 0x8dc9u, 0x7c26u, 0x6c07u,
    0x5c64u, 0x4c45u, 0x3ca2u, 0x2c83u, 0x1ce0u, 0x0cc1u, 0xef1fu, 0xff3eu, 0xcf5du,
    0xdf7cu, 0xaf9bu, 0xbfbau, 0x8fd9u, 0x9ff8u, 0x6e17u, 0x7e36u, 0x4e55u, 0x5e74u,
    0x2e93u, 0x3eb2u, 0x0ed1u, 0x1ef0u
  };

/**
  * @brief This routine calculates CRC16 check sum of defined block at memory
  * @par Parameters:
  * Sum initialization, pointer to begin of block, length of the block
  * @retval
  * CRC16 check sum of calculated block
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * None
  */
uint16_t fast_crc16(uint16_t sum, const uint8_t *p, uint16_t len)
{
  while (len--)
  {
  /* ==============================================================================*/
  /* MISRA violation of rule 17.4 - pointer arithmetic is used for CRC calculation */
  #pragma diag_suppress=Pm088
    sum = (uint16_t)crc16_table[(uint16_t)((uint16_t)(sum >> 8u) ^ *p++)] ^ (uint16_t)(sum << 8u);
  #pragma diag_default=Pm088
  /* ==============================================================================*/
  }
  return sum;
}
#endif /* SLOW_CRC_FLASH_CHECK */
/**
  * @brief This routine compare the result of CRC calculation
  * @par Parameters:
  * Initial sum, pointer to begin of block, length of the block
  * @retval
  * 0 .. calculation OK , 1 .. calculation is bad
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * None
  */
uint16_t FlashCRC16_StartUp_CheckSum(void)
{
  uint16_t calc = 0u;
  uint16_t res = 0u;  /* result is OK by default */

#ifdef SLOW_CRC_FLASH_CHECK
  uint16_t rng = 0u;
  /* Run the checksum algorithm */
  refresh_iwdog();
  rng = (uint16_t)(&__checksum_end) - (uint16_t)(&__checksum_begin) + 1u;
  if(rng > 0x1800)
  calc = slow_crc16(0u, (&__checksum_begin), 0x1800);
  /* it is necessary to refresh watchdog somewhere in the middle of the slow calculation */
  refresh_iwdog();
  calc = slow_crc16(calc, (&__checksum_begin + 0x1800), rng - 0x1800);
  /* Rotate out the result */
  rng = 0;
  calc = slow_crc16(calc, (const uint8_t *)rng, 2u);
#else
  calc = fast_crc16(0u, (const uint8_t *)(&__checksum_begin),\
                   (uint16_t)(&__checksum_end) - (uint16_t)(&__checksum_begin) + 1u);
#endif /* SLOW_CRC_FLASH_CHECK */
  /* Test the checksum result - the result is correct only in case out of debuging  */
  if (calc != __checksum)
  {
    #ifndef DEBUG
      res= CRC_ERROR;
    #else
      res= CRC_OK;
    #endif /* DEBUG */
  }
  return res;
}
/**
  * @brief This routine inits block CRC calculation variables to be performed at run time
  * @par Parameters:
  * None
  * @retval
  * None
  * @par Required preconditions:
  * The tested area range must corresponds to linker setting
  * @par Library functions called:
  * None
  */
void STL_FlashCrc16Init(void)
{
  /* pointer set to CRC test begin at starting address of the Flash */
  p_RunCrc16Chk= (const uint8_t *)(&__checksum_begin);
  p_RunCrc16ChkInv = (uint8_t *)(uint16_t)(~(uint16_t)(p_RunCrc16Chk));
  /* initialization value of the CRC calculation */
  CurrentCrc16= CRC_INIT;
  CurrentCrc16Inv = ~CurrentCrc16;
  /* number of blocks to be calculated by CRC */
  CRCBlockIndex= CRC_NUMB_BLOCKS;
  CRCBlockIndexInv = (uint16_t)(~CRCBlockIndex);
}

/**
  * @brief This routine inits block CRC calculation performed at run time
  * @par Parameters:
  * Initial sum, pointer to begin of block, length of the block
  * @retval
  * None
  * @par Required preconditions:
  * None
  * @par Library functions called:
  * None
  */
uint8_t FlashCrc16_Block_test(void)
{
  uint8_t sts = CRC_ONGOING;
#ifdef SLOW_CRC_FLASH_CHECK
  uint16_t zeros;
#endif /* SLOW_CRC_FLASH_CHECK */
  
  /* check validity of all the Class B variables involved in the test
    (CurrentCrc16 pair is tested at higher level) */
  if((p_RunCrc16Chk == (uint8_t *)((uint16_t)(~(uint16_t)(p_RunCrc16ChkInv))))\
  && (CRCBlockIndex == (uint16_t)(~CRCBlockIndexInv)))
  {
    /* compute CRC of next block */
#ifdef SLOW_CRC_FLASH_CHECK
    CurrentCrc16= slow_crc16(CurrentCrc16, p_RunCrc16Chk, CRC_BLOCKSIZE);
#else
    CurrentCrc16= fast_crc16(CurrentCrc16, p_RunCrc16Chk, CRC_BLOCKSIZE);
#endif /* SLOW_CRC_FLASH_CHECK */
    CurrentCrc16Inv = ~CurrentCrc16;
  /* ==============================================================================*/
  /* MISRA violation of rule 17.4 - pointer arithmetic is used for CRC calculation */
  #pragma diag_suppress=Pm088
    /* increment pointers */
    p_RunCrc16Chk+= CRC_BLOCKSIZE;
  #pragma diag_default=Pm088
  /* ==============================================================================*/
    p_RunCrc16ChkInv = (uint8_t *)(uint16_t)(~(uint16_t)(p_RunCrc16Chk));
    --CRCBlockIndex;
    CRCBlockIndexInv = (uint16_t)(~CRCBlockIndex);
    /* test finished? */
    if(CRCBlockIndex == 0u)
    {
#ifdef SLOW_CRC_FLASH_CHECK
      /* Rotate out the result */
      zeros= 0u;
      CurrentCrc16 = slow_crc16(CurrentCrc16, (const uint8_t *)zeros, 2u);
#endif /* SLOW_CRC_FLASH_CHECK */
      /* Test the checksum */
      if (CurrentCrc16 != __checksum)
      {
        #ifndef DEBUG
          sts= CRC_ERROR;
        #else
          sts= CRC_OK;
        #endif /* DEBUG */
      }
      else
      {
        sts= CRC_OK;
      }
    }
  }
  else
  {
    sts= CRC_ERROR;
  }
  return sts;
}
/**
  * @}
  */

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/