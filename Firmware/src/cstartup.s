/**************************************************
 *
 * System initialization code.
 *
 * Copyright 2010 IAR Systems AB.
 *
 * $Revision: 2079 $
 *
 **************************************************
 *
 * The startup code first calls __low_level_init,
 * where the user can perform application-specific
 * initialization.  Then, unless __low_level_init
 * returned 0, the system initializes global and
 * static data.  Finally, C++ constructors are run
 * if necessary, before the user's main function
 * is called.
 *
 * After main returns, control moves to the exit
 * function, passing the return value from main as
 * a parameter to exit.
 *
 **************************************************/

#include "macros.m"

        MODULE  ?cstartup

#include "vregs.inc"

        PUBLIC  __iar_program_start
        PUBLIC  __iar_start_continue
        PUBLIC  __iar_cstart_call_ctors
        PUBLIC  ??call_ctors

        EXTERN  __low_level_init
        EXTERN  __iar_data_init2
        EXTERN  __intvec

        EXTERN  SHT$$PREINIT_ARRAY$$Base
        EXTERN  SHT$$INIT_ARRAY$$Limit

        EXTERN  __call_ctors

        EXTERN  main
        EXTERN  exit
        EXTERN  STL_StartUp

	RTMODEL	"__rt_version", "4"

        SECTION CSTACK:DATA:NOROOT(0)

        SECTION __DEFAULT_CODE_SECTION__:CODE:NOROOT(0)
	CODE

        REQUIRE __intvec
                
__iar_program_start:
        LDW     X, #SFE(CSTACK)-1
        LDW     SP, X

        XCALL  STL_StartUp 
        
__iar_start_continue:
        LDW     X, #SFE(CSTACK)-1
        LDW     SP, X
        
	XCALL	__low_level_init
	TNZW	X
	JREQ	skip_data_init

        XCALL   __iar_data_init2
skip_data_init:

        SECTION __DEFAULT_CODE_SECTION__:CODE:NOROOT(0)
	CODE

??call_ctors:
__iar_cstart_call_ctors:

#if __DATA_MODEL__ != __LARGE_DATA_MODEL__

	LDW	X, #SHT$$PREINIT_ARRAY$$Base
	LDW	Y, #SHT$$INIT_ARRAY$$Limit
#else

	LDW	X, #LWRD(SHT$$PREINIT_ARRAY$$Base)
	LDW	S:?e0+1, X
	LD	A, #BYTE3(SHT$$PREINIT_ARRAY$$Base)
	LD	S:?e0, A

	LDW	X, #LWRD(SHT$$INIT_ARRAY$$Limit)
	LDW	S:?e1+1, X
	LD	A, #BYTE3(SHT$$INIT_ARRAY$$Limit)
	LD	S:?e1, A
#endif

	XCALL	__call_ctors

        SECTION __DEFAULT_CODE_SECTION__:CODE:ROOT(0)
	CODE

        XCALL   main

        XTAILCALL   exit

        END
