;******************* (C) COPYRIGHT 2017 STMicroelectronics ********************
; File Name          : stm8_stl_cpustart_IAR.s
; Description        : This file contains STM8 CPU test function executed at Start-up.
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
    name    StartUpCPUTestcode
    public  STL_StartUpCPUTest
    extern  CtrlFlowCnt
    extern  CtrlFlowCntInv
    extern  FailSafe
    section .text:code
;
;*******************************************************************************
; Function Name  : STL_StartUpCPUTest
; Description    : Full STM8 CPU test at start-up
; Input          : None.
; Output         : Jump directly to a Fail Safe routine in case of failure
; Return         : SUCCESS (=1)
; WARNING        : all registers become destroyed when exiting this function
;                  excluding stack pointer
;*******************************************************************************/
STL_StartUpCPUTest:
    LDW X,CtrlFlowCnt
    ADDW    X,#3        ; CtrlFlowCnt += CPU_INIT_CALLEE
    LDW CtrlFlowCnt,x
    ; If X is not functional, corruption will be detected later
;
    ; Check flags of code condition register
    CLR   A               ; Set Z(ero) Flag
    JRNE  ErrorCPU        ; Fails if Z=0
    LD    A,#1            ; Reset Z Flag
    JREQ  ErrorCPU        ; Fails if Z=1
;   
    SUB   A,#2            ; Set N(egative) Flag (A=0xFF)
    JRPL  ErrorCPU        ; Fails if N=0
    ADD   A,#2            ; Reset N and set C Flags (Res=0x101)
    JRMI  ErrorCPU        ; Fails if N=1
;
    RCF                   ; Reset C(arry) Flag
    JRC   ErrorCPU        ; Fails if C=1
    SCF                   ; Set C(arry) Flag
    JRNC  ErrorCPU        ; Fails if C=0
;
    ADD   A,#0FH          ; Set H(alf) carry Flag (A=0x10)
    JRNH  ErrorCPU        ; Fails if H=0
    ADD   A,#0FH          ; Reset H(alf) carry Flag (A=0x1F)
    JRH   ErrorCPU        ; Fails if H=1
;
    LD    A,#80H
    ADD   A,#80H          ; Set V (oVerflow) Flag (Res=0x100)
    JRNV  ErrorCPU        ; Fails if V=0
    RVF                   ; Reset V Flag
    JRV   ErrorCPU        ; Fails if V=1
;
    SIM                   ; Set interrupt mask
    JRNM  ErrorCPU        ; Fails if I0=0
    RIM                   ; Reset I0 bit
    JRNM  skip            ; If I0=O, skip next instruction
  
ErrorCPU:
    LDW   X,#0            ; Error Code if spurious return
    JP    FailSafe
  
skip:
    SIM                 ; Set I0 bit (mask interrupts)
;   
    ; Check CPU register: A, X, Y
    LD    A,#0AAH
    CP    A, #0AAH
    JRNE  ErrorCPU
    LD    A,#55H
    CP    A,#55H
    JRNE  ErrorCPU
    LD    A,#11H
;   
    LDW   X,#0AAAAH
    CPW   X,#0AAAAH
    JRNE  ErrorCPU
    LDW   X,#5555H
    CPW   X,#5555H
    JRNE  ErrorCPU
    LDW   X,#1234H
;
    LDW   Y,#0AAAAH
    CPW   Y,#0AAAAH
    JRNE  ErrorCPU
    LDW   Y,#5555H
    CPW   Y,#5555H
    JRNE  ErrorCPU
    LDW   Y,#5678H
;   
    ; Verify ramp pattern
    CP    A,#11H
    JRNE  ErrorCPU
    CPW   X,#1234H
    JRNE  ErrorCPU
    CPW   Y,#5678H
    JRNE  ErrorCPU
;   
    ; Check Stack pointer
    LDW   Y,SP            ; Save current stack pointer in Y
    LDW   X,#5555H
    LDW   SP,X
    LDW   X,SP
    CPW   X,#5555H
    JRNE  ErrorCPU
    LDW   X,#0AAAAH
    LDW   SP,X
    LDW   X,SP
    CPW   X,#0AAAAH
    JRNE  ErrorCPU
    LDW   SP,Y            ; Restore stack pointer
;
Exit:
    LDW   X,CtrlFlowCntInv
    SUBW  X,#3            ; CtrlFlowCntInv -= CPU_INIT_CALLEE
    LDW   CtrlFlowCntInv,X
    LD    A,#1            ; Returns success
    RET
;
    END
;******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****
