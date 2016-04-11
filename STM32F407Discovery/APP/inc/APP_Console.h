#ifndef __APP_CONSOLE_H
#define __APP_CONSOLE_H


#include "string.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "CDL_Printf.h"
#include "APP_MotorDriver.h"
#include "main.h"


#define MAXARGS     	4
#ifdef TRUE
#undef TRUE
#endif
#ifdef FALSE
#undef FALSE
#endif
#define TRUE			1				/* true							*/
#define FALSE			0				/* false							*/
#define ARG_MASK                0x3f 
#define HEX_ARG         0x00
#define DEC_ARG         0x01
#define STRING_ARG      0x02
#define COMMAND_ARG     0x03
#define CHARACTER_ARG   0x04
#define CONS_SET(MAP,MASK)    ((MAP & (MASK)) == (MASK))
/* Argument modifiers */
#define OPTIONAL_ARG        0x40
#define REQUIRED_ARG        0x80

#define CONS_CMD_DEF(name) \
	const tCOMMAND aCmd_##name =

#define CONS_CMD(name) (const tCOMMAND *) &aCmd_##name

typedef void    (*MyFunc)(float p1,float p2,float p3);

typedef struct
{
    //char           *pName;
    unsigned int   iValue;
}tARGUMENT;

typedef struct
{
    char         *pName;
    MyFunc       pAction;
    tARGUMENT    aArg[MAXARGS];
}tCOMMAND;


void app_CMDShell_Task(void const * argument);




#endif /* __APP_CONSOLE_H */