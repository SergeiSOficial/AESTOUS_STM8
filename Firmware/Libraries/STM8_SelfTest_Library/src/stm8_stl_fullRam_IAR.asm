;******************* (C) COPYRIGHT 2017 STMicroelectronics ********************
; File Name          : stm8_stl_fullRam_IAR.s
; Description        : This file contains the RAM functional test to be done at
;                      start-up. This test is destructive and will initialize
;                      the whole RAM to zero.
; Author             : STMicroelectronics - MCD Application Team
; Version            : V2.0.0
; Date               : Dec-2017
;*****************************************************************************
; Redistribution and use in source and binary forms, with or without modification,
; are permitted provided that the following conditions are met:
;   1. Redistributions of source code must retain the above copyright notice,
;      this list of conditions and the following disclaimer.
;   2. Redistributions in binary form must reproduce the above copyright notice,
;      this list of conditions and the following disclaimer in the documentation
;      and/or other materials provided with the distribution.
;   3. Neither the name of STMicroelectronics nor the names of its contributors
;      may be used to endorse or promote products derived from this software
;      without specific prior written permission.
;
; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
; DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
; FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
; DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
; SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
; CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
; OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
; OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;*****************************************************************************
;   
    name    RamMarchCcode
    public  STL_FullRamMarchC
    extern  CtrlFlowCntInv
; !!! CONSTANT FROM ICF FILE TO DEFINE END OF THE RAM REGION !!!
    extern __ICFEDIT_region_RAM_end__
    section .text:code
;
;******************************************************************************
;* Function Name  : STL_FullRamMarchC
;* Description    : This function verifies that RAM is functional,
;*                  using the March C- algorithm.
;* Input          : None
;* Output         : The whole RAM is initialized with 0 when exiting this fct,
;*                  at the exception of CtrlFlowCntInv, set to 0xFFFFFFFF.
;* Return         : {ERROR=0; FULL_RAM_SUCCESS=1}
;******************************************************************************
STL_FullRamMarchC:
    POPW        Y       ; Trick: save stacked return adress into Y
;
    ; Step 1: Write background with addresses increasing
    CLRW  X             ; p = RAM_START
step1:                  ; do {
    CLR  (X)            ;   *p = BCKGRND
    INCW  X				; } while (++p <= RAM_END)
    CPW   X,#__ICFEDIT_region_RAM_end__
    JRULE step1
;
    ; Step 2: Verify background and write inv background with addresses increasing
    CLRW  X             ; p = RAM_START
step2:                  ; do {
    LD    A,(X)         ;   if (*p != BCKGRND)
    JRNE  Error         ;     test fail termination
    LD    A,#255        ;   *p = INV_BCKGRND
    LD   (X),A
    INCW  X             ; } while (++p <= RAM_END)
    CPW   X,#__ICFEDIT_region_RAM_end__
    JRULE step2
;
    ; Step 3: Verify inv background and write background with addresses increasing
    CLRW  X             ; p = RAM_START
step3:                  ; do {
    LD    A,(X)         ;   if (*p != INV_BCKGRND)
    CP    A,#255
    JRNE  Error         ;     test fail termination
    CLR   (X)           ;   *p = BCKGRND
    INCW  X             ; } while (++p <= RAM_END)
    CPW   X,#__ICFEDIT_region_RAM_end__ 
    JRULE step3
;
    ; Step 4: Verify background and write inv background with addresses decreasing
    LDW   X,#__ICFEDIT_region_RAM_end__; p = RAM_END
step4:                  ; do {
    LD    A,(X)         ;   if (*p != BCKGRND)
    JRNE  Error         ;     Early test termination
    LD    A,#255
    LD    (X),A         ;   *p = INV_BCKGRND
    DECW  X
    JRPL  step4         ; } while (++p >= RAM_START)
;
    ; Step 5: Verify inv background and write background with addresses decreasing
    LDW   X,#__ICFEDIT_region_RAM_end__ ; p = RAM_END
step5:                  ; do {
    LD    A,(X)         ;   if (*p != INV_BCKGRND)
    CP    A,#255
    JRNE  Error         ;     Early test termination
    CLR   (X)           ;   *p = BCKGRND
    DECW  X
    JRPL  step5         ; } while (++p >= RAM_START)
;
    ; Step 6: Verify background with addresses increasing
    CLRW  X             ; p = RAM_START
step6:                  ; do {
    LD    A,(X)         ;   if (*p != BCKGRND)
    JRNE  Error         ;     Early test termination
    INCW  X             ; } while (++p <= RAM_END)
    CPW   X,#__ICFEDIT_region_RAM_end__
    JRULE step6
;
    LD    A,#1          ; RAM is OK!
    JP    Exit
Error:
    CLR   A               ; error at test found in RAM 
Exit:
    LDW   X,#0ffffH     ; reinit control flow variables
    LDW   CtrlFlowCntInv,X ; (non inversed variable keeps 0!)
;
    PUSHW Y             ; Trick: restore stacked return adress from Y
  
    CLRW  X         ; copy result in A to X
    TNZ   A
    JREQ  Exitrt
  
    INCW  X
Exitrt:  
    RET
;
    END
;
;******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****
