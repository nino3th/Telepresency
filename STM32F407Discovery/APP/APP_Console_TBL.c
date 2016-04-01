
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

void cmd_speed(uint16_t leftSpeed, uint16_t rightSpeed)
{
  drive_speed(leftSpeed,rightSpeed);       
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

void cmd_distance(int dis,int vel)
{  
    drive_distance((float)dis, (float)dis, (float)vel);
}

void cmd_angle(int deg,int vel)
{
    drive_angle((float)deg, (float)vel);
}

void cmd_getodom(void)
{
  double x,y,heading,omega,v;
  drive_get_odom_info(&x,&y,&heading,&omega,&v);
  printf("x : %f\t",x);
  printf("y : %f\t",y);
  printf("heading : %f\t",heading);
  printf("omega : %f\t",omega);
  printf("v : %f\r\n",v);
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

CONS_CMD_DEF(dis)
{
        "dis",
	(MyFunc)cmd_distance,
	{    
          (DEC_ARG | REQUIRED_ARG),         
          (DEC_ARG | REQUIRED_ARG)           
	}
};

CONS_CMD_DEF(ang)
{
        "ang",
	(MyFunc)cmd_angle,
	{    
          (DEC_ARG | REQUIRED_ARG),
          (DEC_ARG | REQUIRED_ARG)        
	}
};

CONS_CMD_DEF(getodom)
{
        "getodom",
	(MyFunc)cmd_getodom,
	{    
          (0)  
	}
};


const tCOMMAND * aCmdTbl[] =
{
	CONS_CMD(speed),
        CONS_CMD(start),
        CONS_CMD(getspeed),
        CONS_CMD(getodom),
        CONS_CMD(pwr),
        CONS_CMD(dis),  
        CONS_CMD(ang),  
	// end with NULL, never remove this
	NULL,
};
