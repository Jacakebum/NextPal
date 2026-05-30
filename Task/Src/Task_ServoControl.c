#include "tim.h"
#include "main.h"
#include "stdio.h"
#include "gpio.h"

#include "Inverse_K.h"
#include "math.h"
#include "Task_StateMachine.h"

#define Trot  1
#define Walk  2
#define Crawl 3

#define FORWARD_RIGHT     htim1
#define FORWARD_LEFT      htim2
#define BACKWARD_RIGHT    htim3
#define BACKWARD_LEFT     htim4

#define HIP     TIM_CHANNEL_2
#define THIGH   TIM_CHANNEL_3
#define SHANK   TIM_CHANNEL_4

#define HEAD_YAW_TIM       htim2
#define HEAD_PITCH_TIM     htim3
#define HEAD_ROLL_TIM      htim4
#define HEAD_YAW_CH        TIM_CHANNEL_1
#define HEAD_PITCH_CH      TIM_CHANNEL_1
#define HEAD_ROLL_CH       TIM_CHANNEL_1

#define FL_HIP_Value    150
#define FL_THIGH_Value  115
#define FL_SHANK_Value  120-20
#define FR_HIP_Value    110
#define FR_THIGH_Value  123
#define FR_SHANK_Value  140
#define BL_HIP_Value    180
#define BL_THIGH_Value  153
#define BL_SHANK_Value  180
#define BR_HIP_Value    195
#define BR_THIGH_Value  155
#define BR_SHANK_Value  111-60

/*单腿关节转角*/
extern floatTheta Leg_angle[4];

/*陀螺仪解算出的欧拉角*/
extern float angle[3];

floatXYZ pos;
floatRPY angle_;	
			
static uint8_t Update_Control(void);
static uint8_t Stand(void);
static uint8_t Pos_Attitude_Control(floatXYZ pos,floatRPY angle);
static uint8_t Servo_Control(void);
static uint8_t IMU_Attitude_Control(void);
static float Trans_Angle(float angle);
static float Clamp_PWM(float pwm);
static uint8_t Head_Servo_Update(void);
static float Clamp_Head_Angle(float angle_deg);
static uint16_t Head_Angle_To_PWM(float angle_deg);

#define HEAD_CENTER_DEG            90.0f
#define PITCH_GEAR_GAIN            3.0f   /* 1:3减速，舵机角度放大3倍补偿输出 */
#define NOD_OUTPUT_AMP_DEG         18.0f  /* 头部输出端振幅（度） */
#define SHAKE_OUTPUT_AMP_DEG       25.0f
#define TILT_OUTPUT_AMP_DEG        20.0f
#define NOD_MOTION_STEP            0.16f  /* 比摇头更慢 */
#define SHAKE_MOTION_STEP          0.25f
#define TILT_MOTION_STEP           0.25f

#define LEG_PWM_MIN                50.0f
#define LEG_PWM_MAX                250.0f
#define LEG_PWM_SMOOTH_GAIN        0.28f

/*步态相关函数*/
static uint8_t Trot_Gait(MainState_t MainState);
/*步态相关函数*/

void Task_Control_Start(void)
{
    Stand();
    TickType_t xLastWakeUpTime;
    xLastWakeUpTime = xTaskGetTickCount();
    while (1)
    {
			  //Stand();
        Update_Control();
        vTaskDelayUntil(&xLastWakeUpTime, 10);
    }
}

uint8_t Update_Control(void)
{
	Head_Servo_Update();
	
	
	/*扭动身体*/
	if (GetGaitMode()==GaitMode_Body_Twist)
	{
		static float x=0;
	  static float y=0;
	  static float z=150;
	  static float P=0;
	  static float R=0;
	  static float Y=0;
		if(GetMainState() == MainState_IMU)  /*IMU自稳模式*/
			IMU_Attitude_Control();
		else
		{
			pos.x=x;
			pos.y=y;
			pos.z=z;
			angle_.P=P;
			angle_.R=R;
			angle_.Y=Y;				
			switch(GetMainState()) 
			{		
			 case MainState_Static:
			 {	 
   			 if(z<150) 
					 z++;
				 
         if(y>0) y--;
				 else if(y<0) y++;
				 
         if(x>0) x--;
				 else if(x<0) x++;
				 
				 if(P>0) P=P-0.5;
				 else if(P<0) P=P+0.5;
				 
         if(R>0) R--;
				 else if(R<0) R++;
				 
         if(Y>0) Y--;
				 else if(Y<0) Y++;
			 } 
			 break;
			 case MainState_Step:
			 {				 
				 z--;	
				 if(z<130) z=130;		
			 }				 
			 break;	
			 case MainState_Forward:
			 {				 
				 P=P-0.5;	
				 if(P<-10) P=-10;		
			 }
			 break;
			 case MainState_Backward:
			 {				 
				 P=P+0.5;	
				 if(P>10) P=10;		
			 }		    
			 break;		 
			 case MainState_Move_to_Left:
			 {				 
				 R--;	
				 if(R<-15) R=-15;		
			 }  
			 break;	
			 case MainState_Move_to_Right:
			 {				 
				 R++;	
				 if(R>15) R=15;		
			 }			 
			 break;	
			 case MainState_Turn_Left:
			 {				 
				 Y++;	
				 if(Y>10) Y=10;		
			 }
			 break;		 
			 case MainState_Turn_Right:
			 {				 
				 Y--;	
				 if(Y<-10) Y=-10;		
			 }   
			 break;	
			 default:
			 ;			 
			}		
//printf("%f \t",z);			
			Pos_Attitude_Control(pos,angle_);	 
		}
	}
	
	/*Trot步态*/
	else if (GetGaitMode()==GaitMode_Trot) 
	{
		Trot_Gait(GetMainState());	
	}
	
	
//	/*Crawl步态*/	
//	else if (GetGaitMode()==GaitMode_Crawl)  
//	{
//		switch(GetMainState()) 
//	  {				 
//		 case MainState_Step:	
//			 Step(Crawl);
//		 break;	
//		 case MainState_Forward:
//			 Forward(Crawl);
//		 break;
//		 case MainState_Backward:
//			 Backward(Crawl);    
//		 break;		 
//		 case MainState_Move_to_Left:
//			 Move_to_Left(Crawl);    
//		 break;	
//		 case MainState_Move_to_Right:
//			 Move_to_Right(Crawl);			 
//		 break;	
//		 case MainState_Turn_Left:
//       Turn_Left(Crawl);
//		 break;		 
//		 case MainState_Turn_Right:
// 			 Turn_Right(Crawl);     
//		 break;		 
//     default:
//			 Stand();	 
//	  }		
//	}
	return 1;
}

uint8_t Head_Servo_Update(void)
{
	float yaw = (float)GetHeadYaw();
	float pitch = (float)GetHeadPitch();
	float roll = (float)GetHeadRoll();
	HeadMotionMode_t mode = GetHeadMotionMode();

	static float phase = 0.0f;
	const float motion_cycle = 6.2831853f;
	const float nod_servo_amp = NOD_OUTPUT_AMP_DEG * PITCH_GEAR_GAIN;

	/* 手动Pitch以90度为中心放大偏移量，补偿1:3减速 */
	pitch = HEAD_CENTER_DEG + (pitch - HEAD_CENTER_DEG) * PITCH_GEAR_GAIN;

	if(mode == HeadMotion_Nod)
	{
		phase += NOD_MOTION_STEP;
		if(phase > motion_cycle) phase -= motion_cycle;
		pitch += nod_servo_amp * sinf(phase);
	}
	else if(mode == HeadMotion_Shake)
	{
		phase += SHAKE_MOTION_STEP;
		if(phase > motion_cycle) phase -= motion_cycle;
		yaw += SHAKE_OUTPUT_AMP_DEG * sinf(phase);
	}
	else if(mode == HeadMotion_Tilt)
	{
		phase += TILT_MOTION_STEP;
		if(phase > motion_cycle) phase -= motion_cycle;
		roll += TILT_OUTPUT_AMP_DEG * sinf(phase);
	}
	else
	{
		phase = 0.0f;
	}

	__HAL_TIM_SET_COMPARE(&HEAD_YAW_TIM, HEAD_YAW_CH, Head_Angle_To_PWM(yaw));
	__HAL_TIM_SET_COMPARE(&HEAD_PITCH_TIM, HEAD_PITCH_CH, Head_Angle_To_PWM(pitch));
	__HAL_TIM_SET_COMPARE(&HEAD_ROLL_TIM, HEAD_ROLL_CH, Head_Angle_To_PWM(roll));

	return 1;
}

float Clamp_Head_Angle(float angle_deg)
{
	if(angle_deg < 0.0f)
		return 0.0f;
	if(angle_deg > 180.0f)
		return 180.0f;
	return angle_deg;
}

uint16_t Head_Angle_To_PWM(float angle_deg)
{
	float angle = Clamp_Head_Angle(angle_deg);
	return (uint16_t)(50.0f + (angle / 180.0f) * 200.0f);
}

float Clamp_PWM(float pwm)
{
	if(pwm < LEG_PWM_MIN)
		return LEG_PWM_MIN;
	if(pwm > LEG_PWM_MAX)
		return LEG_PWM_MAX;
	return pwm;
}

uint8_t Stand(void)
{

	__HAL_TIM_SET_COMPARE(&FORWARD_RIGHT, HIP,   FR_HIP_Value);   /* 中值1.5ms/20ms=0.075 0.075*2000ms=150 */
	
	__HAL_TIM_SET_COMPARE(&FORWARD_RIGHT, THIGH, FR_THIGH_Value);
	
	__HAL_TIM_SET_COMPARE(&FORWARD_RIGHT, SHANK, FR_SHANK_Value);
	
	__HAL_TIM_SET_COMPARE(&FORWARD_LEFT, HIP,    FL_HIP_Value);   /* 中值1.5ms/20ms=0.075 0.075*2000ms=150 */
	
	__HAL_TIM_SET_COMPARE(&FORWARD_LEFT, THIGH,  FL_THIGH_Value);
	
	__HAL_TIM_SET_COMPARE(&FORWARD_LEFT, SHANK,  FL_SHANK_Value);	
	
	__HAL_TIM_SET_COMPARE(&BACKWARD_RIGHT, HIP,   BR_HIP_Value);   /* 中值1.5ms/20ms=0.075 0.075*2000ms=150 */
	
	__HAL_TIM_SET_COMPARE(&BACKWARD_RIGHT, THIGH, BR_THIGH_Value);
	
	__HAL_TIM_SET_COMPARE(&BACKWARD_RIGHT, SHANK, BR_SHANK_Value);
	
	__HAL_TIM_SET_COMPARE(&BACKWARD_LEFT, HIP,    BL_HIP_Value);   /* 中值1.5ms/20ms=0.075 0.075*2000ms=150 */
	
	__HAL_TIM_SET_COMPARE(&BACKWARD_LEFT, THIGH,  BL_THIGH_Value);
	
	__HAL_TIM_SET_COMPARE(&BACKWARD_LEFT, SHANK,  BL_SHANK_Value);
//		floatXYZ pos;
//		floatRPY angle;
//		
//		pos.x=0;
//		pos.y=0;
//		pos.z=150;
//		angle.P=0;
//		angle.R=0;
//		angle.Y=0;	
//		Pos_Attitude_Control(pos,angle);
	
	return 1;
}

/*位姿控制*/
uint8_t Pos_Attitude_Control(floatXYZ pos,floatRPY angle)
{
	/*每条腿的位置向量*/
  floatXYZ AB[4];
	AB[0]=Leg_Position_Vector(0,pos,angle);
	AB[1]=Leg_Position_Vector(1,pos,angle);
	AB[2]=Leg_Position_Vector(2,pos,angle);
	AB[3]=Leg_Position_Vector(3,pos,angle);
  inverse_kinematics(AB,4);
	Servo_Control();
	return 1;
}

/*角度转化为舵机PWM值*/
uint8_t Servo_Control(void)
{	
	float PWM[4][3];
	float temp_angle[4];
	static float filtered_PWM[4][3];
	static uint8_t pwm_initialized = 0;
	//Leg_angle[1].theta_3=-120*DEG_TO_RAD;
	/*左前腿*/
	PWM[0][0]=FL_HIP_Value   + ( Leg_angle[0].theta_1*RAD_TO_DEG-0 )/180*200;
	PWM[0][1]=FL_THIGH_Value + ( Leg_angle[0].theta_2*RAD_TO_DEG-0 )/180*200;
	/*转化连杆角度*/
	temp_angle[0]=Trans_Angle(-Leg_angle[0].theta_3-90*DEG_TO_RAD);
	PWM[0][2]=FL_SHANK_Value + ( temp_angle[0]*RAD_TO_DEG-0 )/180*200;
	
	/*右前腿*/
	PWM[1][0]=FR_HIP_Value   - ( Leg_angle[1].theta_1*RAD_TO_DEG-0 )/180*200;
	PWM[1][1]=FR_THIGH_Value - ( Leg_angle[1].theta_2*RAD_TO_DEG-0 )/180*200;
	/*转化连杆角度*/
	temp_angle[1]=Trans_Angle(-Leg_angle[1].theta_3-90*DEG_TO_RAD);
	PWM[1][2]=FR_SHANK_Value - ( temp_angle[1]*RAD_TO_DEG-0 )/180*200;
	
	/*左后腿*/
	PWM[2][0]=BL_HIP_Value   - ( Leg_angle[2].theta_1*RAD_TO_DEG-0 )/180*200;
	PWM[2][1]=BL_THIGH_Value + ( Leg_angle[2].theta_2*RAD_TO_DEG-0 )/180*200;
	/*转化连杆角度*/
	temp_angle[2]=Trans_Angle(-Leg_angle[2].theta_3-90*DEG_TO_RAD);
	PWM[2][2]=BL_SHANK_Value + ( temp_angle[2]*RAD_TO_DEG-0 )/180*200;
	
	/*右后腿*/
	PWM[3][0]=BR_HIP_Value   + ( Leg_angle[3].theta_1*RAD_TO_DEG-0 )/180*200;
	PWM[3][1]=BR_THIGH_Value - ( Leg_angle[3].theta_2*RAD_TO_DEG-0 )/180*200;
	/*转化连杆角度*/
	temp_angle[3]=Trans_Angle(-Leg_angle[3].theta_3-90*DEG_TO_RAD);
	PWM[3][2]=BR_SHANK_Value - ( temp_angle[3]*RAD_TO_DEG-0 )/180*200;	

	for(int i=0; i<4; i++)
	{
		for(int j=0; j<3; j++)
		{
			PWM[i][j] = Clamp_PWM(PWM[i][j]);
			if(pwm_initialized == 0)
				filtered_PWM[i][j] = PWM[i][j];
			else
				filtered_PWM[i][j] += LEG_PWM_SMOOTH_GAIN * (PWM[i][j] - filtered_PWM[i][j]);
		}
	}
	pwm_initialized = 1;

	__HAL_TIM_SET_COMPARE(&FORWARD_LEFT,   HIP,    (uint16_t)filtered_PWM[0][0]);  
	__HAL_TIM_SET_COMPARE(&FORWARD_LEFT,   THIGH,  (uint16_t)filtered_PWM[0][1]);   
	__HAL_TIM_SET_COMPARE(&FORWARD_LEFT,   SHANK,  (uint16_t)filtered_PWM[0][2]); 	
	__HAL_TIM_SET_COMPARE(&FORWARD_RIGHT,  HIP,    (uint16_t)filtered_PWM[1][0]);  
	__HAL_TIM_SET_COMPARE(&FORWARD_RIGHT,  THIGH,  (uint16_t)filtered_PWM[1][1]);   
	__HAL_TIM_SET_COMPARE(&FORWARD_RIGHT,  SHANK,  (uint16_t)filtered_PWM[1][2]);
	__HAL_TIM_SET_COMPARE(&BACKWARD_LEFT,  HIP,    (uint16_t)filtered_PWM[2][0]);  
	__HAL_TIM_SET_COMPARE(&BACKWARD_LEFT,  THIGH,  (uint16_t)filtered_PWM[2][1]);   
	__HAL_TIM_SET_COMPARE(&BACKWARD_LEFT,  SHANK,  (uint16_t)filtered_PWM[2][2]); 	
	__HAL_TIM_SET_COMPARE(&BACKWARD_RIGHT, HIP,    (uint16_t)filtered_PWM[3][0]);  
	__HAL_TIM_SET_COMPARE(&BACKWARD_RIGHT, THIGH,  (uint16_t)filtered_PWM[3][1]);   
	__HAL_TIM_SET_COMPARE(&BACKWARD_RIGHT, SHANK,  (uint16_t)filtered_PWM[3][2]);	
	
	return 1;
}

/*转化连杆角度*/
float Trans_Angle(float angle)
{
	float l,angle_;
	if(angle>0)
	{
		l=50*sin(0.5*angle);
		//printf("l=%f \n",l);
		angle_=asin( ( pow(l,2)+2*71*l )/ ( 48*( 71+l )) );
	}
	else
	{
		l=50*sin(-0.5*angle);
		//printf("l=%f \n",l);
		angle_=asin( ( pow(l,2)-2*71*l )/ ( 48*( 71-l )) );		
	}
	return angle_;
}

/*Trot步态函数*/
uint8_t Trot_Gait(MainState_t MainState)
{
	static float time=0;
	static int GaitTransition_flag=1;
	static MainState_t last_MainState=MainState_Static;

	if(MainState != last_MainState)
	{
		time = 0;
		GaitTransition_flag = 1;
		last_MainState = MainState;
	}

	if(MainState==MainState_Static)
	{
		time = 0;
		GaitTransition_flag = 1;
		Stand();
	}
	else
	{
		if(time>T)
		{
			time=0;
			GaitTransition_flag++;
		}
		floatXYZ AB[4];
			
		if((GaitTransition_flag%2)==1)
		{
			AB[0]=SwayStatus_Trajectory(MainState,0,time);
			AB[3]=SwayStatus_Trajectory(MainState,3,time);
			AB[1]=SupportingStatus_Trajectory(MainState,1,time);
			AB[2]=SupportingStatus_Trajectory(MainState,2,time);
		}
		else
		{
			AB[1]=SwayStatus_Trajectory(MainState,1,time);
			AB[2]=SwayStatus_Trajectory(MainState,2,time);
			AB[0]=SupportingStatus_Trajectory(MainState,0,time);
			AB[3]=SupportingStatus_Trajectory(MainState,3,time);
		}	
		inverse_kinematics(AB,4);
		Servo_Control();
		time=time+1;
	}		

	return 1;
}

/*IMU自稳*/
uint8_t IMU_Attitude_Control(void)
{
	floatXYZ pos;
	pos.x=0;
	pos.y=0;
	pos.z=130;		
	floatRPY euler_angle;
	
	float KP=0.8;
	float KD=0.2;	

	static float err_P=0;
	static float err_last_P=0;
	static float err_R=0;
	static float err_last_R=0;	
	err_P=0-angle[1];
	err_R=0-angle[0];	
	
	euler_angle.P=KP*err_P+KD*(err_P-err_last_P);
	err_last_P=err_P;
	euler_angle.R=KP*err_R+KD*(err_R-err_last_R);
	err_last_R=err_R;	
	
	//euler_angle.Y=KP*angle[2];
	euler_angle.Y=0;
	Pos_Attitude_Control(pos,euler_angle);
	return 1;
}
