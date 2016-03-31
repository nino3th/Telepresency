

#include "APP_MotorDriver.h"


TIM_HandleTypeDef htim1;
TIM_OC_InitTypeDef octim1;


static uint8_t drive_state = SPEED;

extern int abd_ticksL;
extern int abd_ticksR;

static int abd_speedLCalc = 0;  // Calcuted servo speed left
static int abd_speedRCalc = 0;  // Calcuted servo speed Right
static int abd_speedL = 0;      // Requested servo speed left
static int abd_speedR = 0;      // Requested servo speed right


static int tzL = 0, tzR = 0;
static int tpL = 0, tpR = 0, tpL_timer = OFF, tpR_timer = OFF; // prevent speed rapidly grow up!!


volatile static int ckeck_pwr = ON;

static volatile int driveL = 0;
static volatile int driveR = 0;
static int abd_speed_limit = SPEED_LIMIT_DEFAULT;

volatile double g_distancePerCount = DISTANCE_PER_COUNT_DEFAULT;
volatile double g_trackWidth = TRACK_WIDTH_DEFAULT;

static double Heading = 0.0;
static double X = 0.0;
static double Y = 0.0;
static double Omega = 0.0;
static double V = 0.0;


// speed PID control parameters;
static volatile float Kp_l = 0.1;
static volatile float Ki_l = 0.0;
static volatile float Kd_l = 0.0;
static volatile float Kp_r = 0.1;
static volatile float Ki_r = 0.0;
static volatile float Kd_r = 0.0;

void drive_init(void)
{
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = (uint16_t) ((SystemCoreClock /2) / 500000) - 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 10000;      //100Hz
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    
    octim1.OCMode = TIM_OCMODE_PWM1;
    octim1.Pulse = 1500;
    octim1.OCPolarity = TIM_OCPOLARITY_HIGH;
    octim1.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    octim1.OCFastMode = TIM_OCFAST_DISABLE;
    octim1.OCIdleState = TIM_OCIDLESTATE_RESET;
    octim1.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    
    cdl_PWM_Init(&htim1,&octim1,TIM_CHANNEL_1);


    cdl_PWM_Level(&htim1,&octim1,1500,TIM_CHANNEL_1);
    cdl_PWM_Level(&htim1,&octim1,1500,TIM_CHANNEL_2);
    cdl_PWM_Switch(&htim1,&octim1,ON,TIM_CHANNEL_1);
    cdl_PWM_Switch(&htim1,&octim1,ON,TIM_CHANNEL_2);
    
    osDelay(20);
}

void motor_drive_loop(void const * argument)
{  
    static uint32_t t =0;  
    const uint32_t t_loop = (CLKFREQ / CTRL_FREQ_SPEED);
  //const uint32_t t_loop = 500;
    // PID loop
    //volatile static float pre_errorL = 0, pre_errorR = 0;
    const float maxI = 100, minI = -100;
    static  int spL = 0, spR = 0;
    static  float last_spL = 0, last_spR = 0;
    static  float errorL = 0, errorR = 0;
    static  float integralL = 0, integralR = 0;
    static  float derivativeL = 0, derivativeR = 0;
   
  
  drive_init();
  
  //drive_Speed(100,100);
  
  cdl_Printf("Create motor_drive_loop task\r\n");
   t = CNT;


    while (1) {
   //   if (drive_state == POSITION_INIT || drive_state == POSITION_RUN)
        //position_control();
        // PID Speed Control
      if ((CNT - t) >= t_loop) {
        t = CNT;
        drive_getSpeedCalc(&spL, &spR);
      
        // left PID
        errorL = abd_speedL - spL;
        integralL += Ki_l * errorL;
        if (integralL > maxI) integralL = maxI;
        else if (integralL < minI) integralL = minI;
        derivativeL = -Kd_l * (spL - last_spL);
      //  driveL_bias = driveL;
        driveL = driveL +  (int)(Kp_l * errorL + integralL + derivativeL);
    
        if (driveL > 480) {
          driveL = 480;
          if (Ki_l > 1e-6)
            integralL -= Ki_l * errorL; // windup
        }
        else if (driveL < -480) {
          driveL = -480;
          if (Ki_l > 1e-6)
            integralL -= Ki_l * errorL; // windup
        }
        
        // right PID
        errorR = abd_speedR - spR;
        integralR += Ki_r * errorR;
        if (integralR > maxI) integralR = maxI;
        else if (integralR < minI) integralR = minI;
        derivativeR = -Kd_r * (spR - last_spR);      
     //   driveR_bias = driveR;
        driveR = driveR + (int)(Kp_r * errorR + integralR + derivativeR);
     
        //driveR = - driveR;
        if (driveR > 480) {
          driveR = 480;
          if (Ki_r > 1e-6)
            integralR -= Ki_r * errorR; // windup
        }
        else if (driveR < -480) {
          driveR = -480;
          if (Ki_r > 1e-6)
            integralR -= Ki_r * errorR; // windup
        }
        last_spL = spL;
        last_spR = spR;
        if (ckeck_pwr == OFF) {
          if (fabs(driveL) >= 100)
            driveL = (driveL > 0) ? 100 : -100;
          if (fabs(driveR) >= 100)
            driveR = (driveR > 0) ? 100 : -100;
        }
    /*
        printf("driveL = %d\t",driveL);
        printf("spL = %d\t",spL);
        printf("driveR = %d\t",driveR);
        printf("spR = %d\t\r\n",spR);
        */
        cdl_PWM_Level(&htim1,&octim1,1500+driveL,TIM_CHANNEL_1);
        cdl_PWM_Level(&htim1,&octim1,1500+driveR,TIM_CHANNEL_2);
        cdl_PWM_Switch(&htim1,&octim1,ON,TIM_CHANNEL_1);
        cdl_PWM_Switch(&htim1,&octim1,ON,TIM_CHANNEL_2);
       
      }
    }
}

void EncoderCallback(void const * argument)
{
  static int old_abd_ticksL=0;
  static int old_abd_ticksR=0;
  
  taskENTER_CRITICAL();
  
  abd_speedLCalc = abd_ticksL - old_abd_ticksL;
  old_abd_ticksL = abd_ticksL;
  
  abd_speedRCalc = abd_ticksR - old_abd_ticksR;
  old_abd_ticksR = abd_ticksR;
   
  //    check encoder
  if ((xTaskGetTickCount() - tzR > 25) && (abd_speedRCalc == 0))
    {	            
            // Prevent unexpected acceleration
            if (abd_speedR == 0) {  
                    tpR_timer = OFF;
                    driveR = 0;
            }
            else {      
              if (tpR_timer == OFF) { // timer
                tpR = xTaskGetTickCount();
                tpR_timer = ON;
              }
              if (xTaskGetTickCount() - tpR > 1000 && tpR_timer == ON ) {
                ckeck_pwr = OFF;
                tpR_timer = OFF;
              }
            }    
    }
    else 
    {
        if (abd_speedR)
             ckeck_pwr = ON;
    }
    
    if ((xTaskGetTickCount() - tzL > 25) && (abd_speedLCalc == 0))
    {	            
            // Prevent unexpected acceleration
            if (abd_speedL == 0) {  
                    tpL_timer = OFF;
                    driveL = 0;
            }
            else {      
              if (tpL_timer == OFF) { // timer
                tpL = xTaskGetTickCount();
                tpL_timer = ON;
              }
              if (xTaskGetTickCount() - tpL > 1000 && tpL_timer == ON ) {
                ckeck_pwr = OFF;
                tpL_timer = OFF;
              }
            }    
    }
    else 
    {
        if (abd_speedL)
             ckeck_pwr = ON;
    }
  
  taskEXIT_CRITICAL();
  
    
   
}


void drive_setSpeedPID(int wheel_idx, float P, float I, float D)
{
	//const float speed_sample_t = 1.f / CTRL_FREQ_SPEED;
        const float speed_sample_t = 1.f / 40.f;
	if(wheel_idx == WHEEL_LEFT || wheel_idx == WHEEL_BOTH) {
		Kp_l = P * speed_sample_t;
		Ki_l = I * speed_sample_t;
		Kd_l = D / speed_sample_t;
	}

	if(wheel_idx == WHEEL_RIGHT || wheel_idx == WHEEL_BOTH) {
		Kp_r = P * speed_sample_t;
		Ki_r = I * speed_sample_t;
		Kd_r = D / speed_sample_t;
	}
}


void drive_Speed(int left, int right)       
{
	
    drive_state = SPEED;
    //set_drive_speed(left, right);
    if (left > abd_speed_limit) left = abd_speed_limit;
    if (left < -abd_speed_limit) left = -abd_speed_limit;
    if (right > abd_speed_limit) right = abd_speed_limit;
    if (right < -abd_speed_limit) right = -abd_speed_limit;
    abd_speedL = left;
    abd_speedR = right;

}

void drive_update_odom()
{
    static int speedRight = 0;
    static int speedLeft = 0;
    static int ticksLeft = 0;
    static int ticksRight = 0;
    
    int ticksLeftOld = ticksLeft;
    int ticksRightOld = ticksRight;
    drive_getTicks(&ticksLeft, &ticksRight);
    drive_getSpeedCalc(&speedLeft, &speedRight);
    
    int deltaTicksLeft = ticksLeft - ticksLeftOld;
    int deltaTicksRight = ticksRight - ticksRightOld;
    double deltaDistance = 0.5f * (double) (deltaTicksLeft + deltaTicksRight) * g_distancePerCount;
    double deltaX = deltaDistance * (double) cos(Heading);
    double deltaY = deltaDistance * (double) sin(Heading);
    double RadiansPerCount = g_distancePerCount / g_trackWidth;
    double deltaHeading = (double) (deltaTicksRight - deltaTicksLeft) * RadiansPerCount;
    
    X += deltaX;
    Y += deltaY;
    Heading += deltaHeading;
    // limit heading to -Pi <= heading < Pi
    if (Heading > PI) {
            Heading -= 2.0 * PI;
    }
    else if (Heading <= -PI) {
            Heading += 2.0 * PI;
    }
    
    // http://webdelcire.com/wordpress/archives/527
    V = ((speedRight * g_distancePerCount) + (speedLeft * g_distancePerCount)) / 2;
    Omega = ((speedRight * g_distancePerCount) - (speedLeft * g_distancePerCount)) / g_trackWidth;
    
#ifdef DebugMsg
    printf("X : %f\t",X);
    printf("Y : %f\t",Y);
    printf("Heading : %f\t",Heading);
    printf("Omega : %f\t",Omega);
    printf("V : %f\r\n",V);
#endif
  
}


void drive_getTicks(int *left, int *right)
{
    *left = abd_ticksL;
    *right = abd_ticksR;
}

void drive_getSpeedCalc(int *left, int *right)
{ 
    *left  = abd_speedLCalc * 1000 /100;
    *right = abd_speedRCalc * 1000 /100;
	
}


// Debug function
void drive_getDrive(int* drive_l, int* drive_r)
{
	*drive_l = driveL;
	*drive_r = driveR;
}

void drive_getTargetTicks(int* tarL, int* tarR)
{
	//*tarL = pos_ctrl.ticksLtarget;
	//*tarR = pos_ctrl.ticksRtarget;
}


int drive_getCHKPWR()
{
    return ckeck_pwr;
}

