
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

void cmd_speed(float leftSpeed, float rightSpeed)
{
  drive_speed((int)leftSpeed,(int)rightSpeed);       
}

void cmd_start(void)
{
   taskFlag = 1;
}

void cmd_stop(void)
{
  osThreadTerminate(STMTaskHandle);
  osThreadTerminate(MotorDriveTaskHandle);
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);        
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  osTimerStop(EncoderTimerID);
  
}

void cmd_getspeed(void)
{
  int s_l=0,s_r=0;
  drive_getDrive(&s_l,&s_r);
   
  printf("driveR = %d\r\n",s_r);
  printf("driveL = %d\r\n",s_l);
        
}

void cmd_distance(float dis,float vel)
{  
    drive_distance(dis, dis, vel);
}

void cmd_angle(float deg,float vel)
{
    drive_angle(deg, vel);
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
CONS_CMD_DEF(spd)
{
	"spd",
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

CONS_CMD_DEF(getspd)
{
        "getspd",
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

CONS_CMD_DEF(stop)
{
        "stop",
	(MyFunc)cmd_stop,
	{    
          (0)  
	}
};


const tCOMMAND * aCmdTbl[] =
{	
        CONS_CMD(start),
        CONS_CMD(stop),
        CONS_CMD(getspd),
        CONS_CMD(getodom),
        CONS_CMD(pwr),
        CONS_CMD(spd),
        CONS_CMD(dis),  
        CONS_CMD(ang),  
        
	// end with NULL, never remove this
	NULL,
};
