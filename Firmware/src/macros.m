/**************************************************
 *
 * Macros used by the run-time assembler library.
 *
 * Copyright 2010 IAR Systems AB.
 *
 * $Revision: 2075 $
 *
 **************************************************/


#ifndef MACROS_M
#define MACROS_M

#if __CODE_MODEL__ == __SMALL_CODE_MODEL__

XCALL:  MACRO func
        CALL    func
        ENDM

XRET:   MACRO
        RET 
        ENDM

XTAILCALL:      MACRO   func
        JP      func
        ENDM

#define RET_ADDR_SIZE 2

#else

XCALL:  MACRO func
        CALLF   func
        ENDM

XRET:   MACRO
        RETF
        ENDM

XTAILCALL:      MACRO   func
        JPF     func
        ENDM

#define RET_ADDR_SIZE 3

#endif

#endif /* MACROS_M */
