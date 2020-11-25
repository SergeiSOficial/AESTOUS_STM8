;******************* (C) COPYRIGHT 2017 STMicroelectronics ********************
; File Name          : stm8_stl_cpurun_IAR.s
; Author             : MCD Application Team
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
    name    RunTimeCPUTestcode
    public  STL_RunTimeCPUTest
    extern  CtrlFlowCnt
    extern  CtrlFlowCntInv
    extern  FailSafe
    section .text:code
;
;*******************************************************************************
; Function Name  : STL_RunTimeCPUTest
; Description    : Full STM8 CPU test to be executed during run-time
; Input          : None.
; Output         : jump directly to a Fail Safe routine in case of failure
; Return         : SUCCESS (=1)
; WARNING        : all registers become destroyed when exiting this function
;                  excluding stack pointer
;*******************************************************************************/
STL_RunTimeCPUTest:
  LDW   X,CtrlFlowCnt
  ADDW  X,#11       ; CtrlFlowCnt += CPU_RUN_CALLEE
  LDW   CtrlFlowCnt,x
; If X is not functional, corruption will be detected later

; Check CPU register: A, X, Y
  LD   A,#0AAH
  CP   A,#0AAH
  JRNE ErrorCPU
  LD   A,#55H
  CP   A,#55H
  JRNE ErrorCPU
  LD   A,#11H
    
  LDW  X,#0AAAAH
  CPW  X,#0AAAAH
  JRNE ErrorCPU
  LDW  X,#5555H
  CPW  X,#5555H
  JRNE ErrorCPU
  LDW  X,#1234H

  LDW  Y,#0AAAAH
  CPW  Y,#0AAAAH
  JRNE ErrorCPU
  LDW  Y,#5555H
  CPW  Y,#5555H
  JRNE ErrorCPU
  LDW  Y,#5678H
    
; Verify ramp pattern
  CP   A,#11H
  JRNE ErrorCPU
  CPW  X,#1234H
  JRNE ErrorCPU
  CPW  Y,#5678H
  JRNE ErrorCPU
  JP   Exit
  
ErrorCPU:
  LDW  X,#12H         ; Error Code if spurious return
  JP   FailSafe

Exit:
  LDW   X,CtrlFlowCntInv
  SUBW  X,#11       ; CtrlFlowCntInv -= CPU_RUN_CALLEE
  LDW   CtrlFlowCntInv,X
    
  LD    A,#1            ; Returns success
  RET
    
  END
;******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****
