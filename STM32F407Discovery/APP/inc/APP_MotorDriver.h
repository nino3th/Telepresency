#ifndef __APP_DRIVER_MANAGER_H
#define __APP_DRIVER_MANAGER_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "CDL_PWM.h"
#include "CDL_Printf.h"
#include "math.h"
#include "time.h"

#define CNT     xTaskGetTickCount()

// ROBOT type
// #define ROBOT_KAOHSIUNG     1
// #define ROBOT_TAIPEI        1
#define ROBOT_KAOHSIUNG        1

#define MOTOR_ENOCDER_ENABLE

#define CLKFREQ                      1000
#define CTRL_FREQ_SPEED              40

#if defined(ROBOT_TAIPEI)
    #define SPEED_LIMIT_DEFAULT         7500
    #define ABD_SPEED_LIMIT_DEFAULT     7500
    #define ABDR_SPEED_LIMIT_DEFAULT    7500
    #define TRACK_WIDTH_DEFAULT         0.365f
    #define DISTANCE_PER_COUNT_DEFAULT  0.00005477f
#elif defined(ROBOT_GUNDAM)
    #define SPEED_LIMIT_DEFAULT         7500
    #define ABD_SPEED_LIMIT_DEFAULT     7500
    #define ABDR_SPEED_LIMIT_DEFAULT    7500
    #define TRACK_WIDTH_DEFAULT         0.465
    #define DISTANCE_PER_COUNT_DEFAULT  0.0000529f
#elif defined(ROBOT_KAOHSIUNG)
    #define SPEED_LIMIT_DEFAULT         480
    #define ABD_SPEED_LIMIT_DEFAULT     480
    #define ABDR_SPEED_LIMIT_DEFAULT    480
    #define TRACK_WIDTH_DEFAULT         0.403f 
    #define DISTANCE_PER_COUNT_DEFAULT  0.00676f 
#endif

#ifndef OFF
#define OFF 0
#endif

#ifndef ON
#define ON  1
#endif

#define PI 3.14159

enum WHEEL_INDEX 
{
	WHEEL_LEFT=0,
	WHEEL_RIGHT=1,
	WHEEL_BOTH=2
};

enum  DRIVE_STATE
{
	SPEED,
	POSITION_INIT,
	POSITION_RUN,
	IDLE
};

typedef struct position_control_data_
{
	int semaphore;
	int ticksLtarget, ticksRtarget; // target tick
	int lastTicksL, lastTicksR;
	int dsrSpL, dsrSpR; // desire wheel speed
	int dirL, dirR; // forward or backward
	// PID params
	float errL, errR;
	float integralL, integralR;
	float derivativeL, derivativeR;
	float Gp, Gi, Gd;
} pos_ctrl_t;

void motor_drive_loop(void const * argument);
void drive_setSpeedPID(int wheel_idx, float P, float I, float D);
void drive_speed(int left, int right);
void drive_setPosPID(float P, float I, float D);
void drive_angle(float degrees, float angular_vel);
void drive_distance(float dist_left, float dist_right, float vel);

void drive_update_odom();

void drive_getTicks(int *left, int *right);
void drive_getSpeedCalc(int *left, int *right);
void drive_getDrive(int* drive_l, int* drive_r);
void drive_getTargetTicks(int* tarL, int* tarR);
int drive_getCHKPWR();
void drive_get_odom_info(double * x, double * y, double * heading, double * omega, double * v);
void EncoderCallback(void const * argument);

#endif /* __APP_DRIVER_MANAGER_H */