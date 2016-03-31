
#include "APP_Console.h"

void app_Console_HDLR(void);
void app_Console_Execute(char *aString);
void app_Console_Mobile(void);

extern  tCOMMAND *aCmdTbl[];

void app_CMDShell_Task(void const * argument)
{
  static uint32_t con_t;    
  
  
  while(TRUE)
  {
    if ((xTaskGetTickCount() - con_t) > 1000) 
    {
      con_t = xTaskGetTickCount();
      //app_Console_HDLR();     
      app_Console_Mobile();      
    
    }
  
  }
}

void app_Console_Mobile(void)
{
    uint8_t c;
    static int16_t tempL=0,tempR=0;
    c = cdl_Getc();
      switch(c)
      {
        case '1':       //forward
          tempL +=50;
          tempR +=50;
          drive_Speed(tempL,tempR);
          break;
        case '2':       //backward
          tempL -=50;
          tempR -=50;
          drive_Speed(tempL,tempR);
          break; 
        case '3':       //left
          tempL =-100;
          tempR =100;
          drive_Speed(tempL,tempR);
          tempL = tempR =0;
          break; 
        case '4':       //right
          tempL =100;
          tempR =-100;
          drive_Speed(tempL,tempR);
          tempL = tempR =0;
          break; 
        case '5':
          tempL =0;
          tempR =0;
          drive_Speed(tempL,tempR);
          break; 
        case '8':
          taskFlag = 1;
        break;
      }      
}

void app_Console_HDLR(void)
{
  static int pos= 0;
  static  char cmd_buffer[32];
  uint8_t buf;  
  HAL_StatusTypeDef ret;	
  int8_t *uiString;
        
  ret = HAL_UART_Receive(&huart2, &buf, 1, 1);
  //bsl_LIN_UART_Read(&ch,sizec);
  if(ret == HAL_OK)
  {
    switch(buf)
    {
      case 0x08:	/* BackSpace */ /* clear the previous character */
      case 0x7F:
        if(pos > 0)
        {			
          cdl_Putc('\b');
          cdl_Putc(' ');
          cdl_Putc('\b');
          pos--;
        }
        break;
      case 0x0D: 	/* ENTER */			
        uiString="\r\n";			
        cdl_Puts((uint8_t*)uiString);			
        cmd_buffer[pos++]='\0'; 
       
        app_Console_Execute(cmd_buffer);
        /* parse and execute the command */
        memset(cmd_buffer,0,sizeof(cmd_buffer));
        pos= 0;
        break;
      default: 	/* Other character */
        /* copy the data */
        if(pos>= 32 )
        {
          pos = 0;
          return;
        }
        else
        {
          cmd_buffer[pos++]= buf;
          /* echo the character */
          cdl_Putc(buf);
        }
        break;
    }
  }
}


tCOMMAND * LookUpCmd(char *pString)
{
    tCOMMAND *pCmd;
    unsigned int iCmpLen, iStringLen, iCmdLen;
	unsigned int iCmdidx;
 
	iCmdidx = 0;
	while((pCmd = (tCOMMAND *)aCmdTbl[iCmdidx]) != NULL )
    {
        iStringLen = strlen(pString);
        iCmdLen = strlen(pCmd->pName);
 
        // select longest string as compare length 
        iCmpLen = (iStringLen >= iCmdLen) ? iStringLen : iCmdLen;
 
        if (!strncmp(pString, pCmd->pName, iCmpLen))
        {
            break;
        }
		iCmdidx++;
    }
    if (pCmd->pName == NULL)
    {
        pCmd = NULL;
    }
    return (pCmd);
}

int atohi(char * s, unsigned int * n)
{
  unsigned int val;
 
  if ( s[0] == '0' && (s[1] == 'x' || s[1] == 'X') ){
    s += 2;
  }
 
  if ( *s == '\0' ){
    return FALSE;
  }
 
  for ( val = 0; *s; s++ ){
    val <<= 4;
    if ( '0' <= *s && *s <= '9' ){
      val += *s - '0';
    } else if ( 'a' <= *s && *s <= 'f' ){
      val +=  *s - 'a' + 10;
    } else if ( 'A' <= *s && *s <= 'F' ){
      val +=  *s - 'A' + 10;
    } else {
      return FALSE;
    }
  }
  *n = val;
  return TRUE;
}

int atod(char * s, unsigned int * n)
{
  unsigned int num;
 
  if ( s == NULL )
  {
    return FALSE;
  }
 
  for ( num=0; *s; s++ ){
    if ( !strchr("0123456789",*s) ){
      return FALSE;
    }
 
	num = (num * 10) + (*s-'0');
  }
  *n = num;
 
  return TRUE;
}
 
uint32_t GetWords(char *p, char *words[], unsigned int max)
{
    register unsigned int n;/* Number of words found */
 
    for (n = 0; n < max; n++)
    {
        words[n] = NULL;
    }
 
    n = 0;
 
    while (*p != '\0')
    {
 
        /* Skip lead white space */
        while (*p == ' ' || *p == '\t')
        {
            p++;
        }
 
        if (*p == '\0')
        {
            break;
        }
 
        if (n++ < max)
        {
            words[n - 1] = p;
        }
 
        /* Find end of word */
        while (*p != ' ' && *p != '\t' && *p != '\0')
        {
            p++;
        }
 
        /* Terminate word */
        if (*p != '\0')
        {
            *p++ = '\0';
        }
    }
    return n;
}

void app_Console_Execute(char *aString)
{
	
    char *pChar;    
    char *pArgs[MAXARGS];
    tCOMMAND *pCmd;
    tARGUMENT *pArgDesc;
    unsigned int iArgType;
    unsigned int iUserArgs;
    unsigned int iMaxArgs;
    unsigned int aArgs[MAXARGS];
    unsigned int iArg;
    unsigned char bError;
    //uint8_t uiStr[20];
    //uint32_t outlen;	
	 
 	/* set all aArgs to zero */
    for (iArg = 0; iArg < MAXARGS; iArg++)
    {
        aArgs[iArg] = 0;
    }
 
    /* Get rid of leading blank characters.  */
    for (pChar = aString; *pChar == ' ' || *pChar == '\t'; pChar++)
    {
        ;
    }
 
    /* Break individual arguments out of command */
    iUserArgs = GetWords(aString, pArgs, MAXARGS) - 1;
 
    /* Check for valid command and number of arguments */
    if ((pCmd = LookUpCmd(pArgs[0])) != NULL)
    {
        iMaxArgs = 0;
        for (pArgDesc = pCmd->aArg; pArgDesc->iValue!= 0; pArgDesc++)
        {
            iMaxArgs++;
        }
        if (iUserArgs > iMaxArgs)
        {
        }
        else
        {
            /* Parse arguments according to types stored
             * in command table */
            pArgDesc = pCmd->aArg;
            iArg = 1;
            bError = FALSE;
            while ((iArg <= iUserArgs) && (pArgDesc->iValue!= 0))
            {
                iArgType = (pArgDesc->iValue & ARG_MASK);
                switch (iArgType)
                {
                case HEX_ARG:
                    if (!atohi(pArgs[iArg], &aArgs[iArg]))
                    {
                        bError = TRUE;
                    }
                    break;
 
                case DEC_ARG:
                    if (!atod(pArgs[iArg], &aArgs[iArg]))
                    {
                        bError = TRUE;
                    }
                    break;
 
                case COMMAND_ARG:
                case STRING_ARG:
                    aArgs[iArg] = (int) pArgs[iArg];
                    break;
                case CHARACTER_ARG:
                    aArgs[iArg]= *(char*) pArgs[iArg];
                    break;
                default:
	#if 0
                    {
                        tARGUMENT * pa;
 
                        pa= LookUpArg(iArgType, pArgs[iArg]);
                        aArgs[iArg] = pa->iValue;
                        if (pa == NULL)
                        {
                            bError = TRUE;
                        }
                    }
#endif
              break;
 
                }
                pArgDesc++;
                iArg++;
            }
            /* if not at last argument description */
            if (pArgDesc->iValue!= 0)
            {
                /* if argument is required */
                if (CONS_SET(pArgDesc->iValue, REQUIRED_ARG))
                {
                    bError = TRUE;
                }
            }
 
            if (!bError)
            {
              cdl_Puts((uint8_t*)"\r\ncmd success\r\n");
            	(*(MyFunc)pCmd->pAction)(                          
                          aArgs[1],
                          aArgs[2],
                          aArgs[3]                                               
                          );
            }
            else
              cdl_Puts((uint8_t*)"\r\ncmd error\r\n");
        }
    }
    else
    {
    	if(pArgs[0]!= 0)
        {
        	cdl_Puts((uint8_t*)"\r\ncmd error\r\n");
        }
    }
 
//	outlen = sprintf((char *)uiStr,(char*)"\r\n#%");
//	cdl_Puts(uiStr);
 
	return;
}
