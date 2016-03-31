
#include "APP_Console.h"

void cmd_ckeck_pwr(void)
{
  int data=0;
  drive_getCHKPWR(&data);
   if(data)
  printf("CHKPWR = ON\r\n");
  else
    printf("CHKPWR = OFF\r\n");
}

void cmd_speed(uint16_t leftSpeed, uint16_t rightSpeed )
{
  drive_Speed(leftSpeed,rightSpeed);       
}

void cmd_start(void)
{
   taskFlag = 1;
}

void cmd_getspeed(void)
{
  int s_l=0,s_r=0;
  drive_getDrive(&s_l,&s_r);
   
  printf("driveR = %d\r\n",s_r);
  printf("driveL = %d\r\n",s_l);
        
}

CONS_CMD_DEF(pwr)
{
        "pwr",
        (MyFunc)cmd_ckeck_pwr,
	{    
          (0)          
	}
};
CONS_CMD_DEF(speed)
{
	"speed",
	(MyFunc)cmd_speed,
	{    
          (DEC_ARG | REQUIRED_ARG),
          (DEC_ARG | REQUIRED_ARG)          
	}
};

CONS_CMD_DEF(start)
{
        "start",
	(MyFunc)cmd_start,
	{    
          (0)          
	}
};

CONS_CMD_DEF(getspeed)
{
        "getspeed",
	(MyFunc)cmd_getspeed,
	{    
          (0)          
	}
};


const tCOMMAND * aCmdTbl[] =
{
	CONS_CMD(speed),
        CONS_CMD(start),
        CONS_CMD(getspeed),
        CONS_CMD(pwr),
	// end with NULL, never remove this
	NULL,
};
