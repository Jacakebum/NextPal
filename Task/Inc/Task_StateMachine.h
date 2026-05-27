#ifndef _TASK_STATEMACHINE_H__
#define _TASK_STATEMACHINE_H__
#include "sysconfig.h"

/******��̬
       վ��
       Walk��̬
			 Trot��̬
			 Crawl��̬
***************/
typedef enum
{
    GaitMode_Stand,	
    GaitMode_Walk,
    GaitMode_Trot,
	  GaitMode_Crawl,
	  GaitMode_Body_Twist
}GaitMode_t;

/******��״̬
       ��ֹ
       ǰ��
			 ����
			 ����ƽ��
			 ����ƽ��
			 ������ת
			 ������ת
***************/
typedef enum
{
	  MainState_Static,
		MainState_Step,
    MainState_Forward,
    MainState_Backward,
    MainState_Move_to_Left,
    MainState_Move_to_Right,
    MainState_Turn_Left,
	  MainState_Turn_Right,
	  MainState_IMU,
	  MainState_Avoid_Obstacle
}MainState_t;

typedef enum
{
	HeadMotion_None = 0,
	HeadMotion_Nod,
	HeadMotion_Shake,
	HeadMotion_Tilt
}HeadMotionMode_t;

void Task_StateMachine_Start(void *parameters);
GaitMode_t GetGaitMode(void);
MainState_t GetMainState(void);
uint8_t GetHeadYaw(void);
uint8_t GetHeadPitch(void);
uint8_t GetHeadRoll(void);
HeadMotionMode_t GetHeadMotionMode(void);
#endif
