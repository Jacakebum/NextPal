#include "Task_StateMachine.h"
#include "stdio.h"
#include "Inverse_K.h"
/*------------------------ state enum --------------------------*/

/*----------------------- state Variables -----------------------*/
GaitMode_t GaitMode;
MainState_t Mainstate;
uint8_t *rdata;
uint8_t *rdata3;

osEvent retval;
osEvent retval3;	
/* ---------------------------- Global Variables ---------------------------- */ 
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern TaskHandle_t TaskHandle_StateMachine;
extern osMailQId myMail01Handle;
extern osMailQId myMail02Handle;
extern uint8_t data[20];
extern uint8_t data2[20];
//extern osMailQId myMail03Handle;

extern uint8_t Re_buf2[10];
uint8_t flag=0;
extern uint8_t data3[10];
float distance=0;
extern volatile uint8_t bluetooth_connected;
extern volatile uint8_t bluetooth_ack_pending;
extern volatile uint8_t us100_measurement_ready;

// 蓝牙连接超时计数器（非static，允许其他文件访问）
uint16_t bluetooth_timeout = 0;

static uint8_t head_yaw = 90;
static uint8_t head_pitch = 90;
static uint8_t head_roll = 90;
static HeadMotionMode_t head_motion_mode = HeadMotion_None;

/*----------------------- state functions -----------------------*/
static uint8_t StateMachine_Init(void);
static uint8_t MainState_Update(void);
static uint8_t GaitMode_Update(void);
static uint8_t Avoid_Obstacle(void);
static void ApplyCommand(uint8_t *cmd);

/**************************
  * @brief  状态机任务
  * @param  unused
  * @retval void
  * @note   5ms执行一次
  */
void Task_StateMachine_Start(void *parameters)
{	
	  StateMachine_Init();
		TickType_t xLastWakeUpTime;
	  HAL_UART_Receive_IT(&huart2,data,2);
	  HAL_UART_Receive_IT(&huart3,data2,1);
	  HAL_UART_Receive_IT(&huart4,data3,1);
	  
	  // 初始化US100，发送触发指令启动测距
	  uint8_t trigger_cmd = 0x55;
	  HAL_UART_Transmit(&huart4, &trigger_cmd, 1, 100);
	  
    xLastWakeUpTime = xTaskGetTickCount();
    
    // 触发计数器，用于持续触发US100
    static uint8_t trigger_count = 0;
    
    // 蓝牙心跳包计数器
    static uint16_t heartbeat_count = 0;
    const char* heartbeat_msg = "PING";
    const uint8_t bluetooth_ack_msg[] = "ok";
    
    while (1)
    {
			retval=osMailGet(myMail01Handle,0);
			if(retval.status==osEventMail)
			{
			 // printf("Receiving queue!\n");
			  rdata=retval.value.p;
			  
			  ApplyCommand(rdata);
			  // 收到蓝牙数据，重置超时计数器
			  bluetooth_timeout = 0;
			  bluetooth_connected = 1;
			  
			  // 调试：保存接收到的数据
			  extern uint8_t data[20];
			  // 释放邮件队列内存
			  osMailFree(myMail01Handle, rdata);
      }
			retval3=osMailGet(myMail02Handle,0);
			if(retval3.status==osEventMail)
			{
				rdata3=retval3.value.p;
				ApplyCommand(rdata3);
				osMailFree(myMail02Handle, rdata3);
			}
			
			Avoid_Obstacle();
			
			// 蓝牙连接超时检测（降低到2秒）
			bluetooth_timeout++;
			if(bluetooth_timeout >= 200)  // 2秒没有收到数据，认为断开
			{
				bluetooth_connected = 0;
			}

			while(bluetooth_ack_pending > 0)
			{
				if(HAL_UART_Transmit(&huart2, (uint8_t*)bluetooth_ack_msg, 2, 100) != HAL_OK)
				{
					break;
				}
				bluetooth_ack_pending--;
			}
			
			// 每1秒发送一次蓝牙心跳包
			heartbeat_count++;
			if(heartbeat_count >= 100)
			{
				HAL_UART_Transmit(&huart2, (uint8_t*)heartbeat_msg, 4, 100);
				heartbeat_count = 0;
			}
			
			// 每100ms触发一次US100测距（提高更新速度）
			trigger_count++;
			if(trigger_count >= 10 && us100_measurement_ready)
			{
				us100_measurement_ready = 0;  // 标记测量开始
				HAL_UART_Transmit(&huart4, &trigger_cmd, 1, 100);
				trigger_count = 0;
			}
			
			vTaskDelayUntil(&xLastWakeUpTime, 10);      /*10ms更新一次*/
		}
}

static void ApplyCommand(uint8_t *cmd)
{
	rdata = cmd;
	MainState_Update();
	GaitMode_Update();
	data[0] = cmd[0];
	data[1] = cmd[1];
}

GaitMode_t GetGaitMode(void)
{
    return GaitMode;
}

MainState_t GetMainState(void)
{
    return Mainstate;
}

uint8_t GetHeadYaw(void)
{
	return head_yaw;
}

uint8_t GetHeadPitch(void)
{
	return head_pitch;
}

uint8_t GetHeadRoll(void)
{
	return head_roll;
}

HeadMotionMode_t GetHeadMotionMode(void)
{
	return head_motion_mode;
}

/*--------------------static Function--------------------*/
/**
 * @brief  初始化状态机
 * @note   默认所有保护模式
 * @retval 
 */
uint8_t StateMachine_Init(void)
{
    GaitMode = GaitMode_Stand;     //站立
    Mainstate = MainState_Static;  //静止
	head_yaw = 90;
	head_pitch = 90;
	head_roll = 90;
	head_motion_mode = HeadMotion_None;
    return 1;
}

uint8_t MainState_Update(void)
{
	if(*rdata==48)     //rdata[0]="0"
	{
	 switch(*(rdata+1)) 
	 {			
		 case 53:	//rdata[1]="5"
	       Mainstate = MainState_Step;
		 break;	
		 case 50://rdata[1]="2"
	       Mainstate = MainState_Forward;
		 break;
		 case 56://rdata[1]="8"
	       Mainstate = MainState_Backward;
		 break;		
		 case 52://rdata[1]="4"
	       Mainstate = MainState_Move_to_Left;
		 break;	
		 case 54://rdata[1]="6"
	       Mainstate = MainState_Move_to_Right;
		 break;	
		 case 49://rdata[1]="1"
	       Mainstate = MainState_Turn_Left;
		 break;		
		 case 51://rdata[1]="3"
	       Mainstate = MainState_Turn_Right;
		 break;	
     case 55://rdata[1]="7"
		 if( GaitMode == GaitMode_Trot ) 
			 flag=1;
       else if( GaitMode == GaitMode_Body_Twist )
			 Mainstate = MainState_IMU;
		 break;
     default:
       ;
	  }
   }
	 else if(*rdata==50)     //rdata[0]="2" - 控制S、T和H参数
	 {
		 switch(*(rdata+1))
		 {
			 case 49:  // rdata[1]="1" - S加5
				 if(S + 5.0f <= 55.0f) S += 5.0f;
			 break;
			 case 50:  // rdata[1]="2" - S减5
				 if(S - 5.0f >= 20.0f) S -= 5.0f;
			 break;
			 case 51:  // rdata[1]="3" - T加1
				 if(T + 1.0f <= 70.0f) T += 1.0f;
			 break;
			 case 52:  // rdata[1]="4" - T减1
				 if(T - 1.0f >= 25.0f) T -= 1.0f;
			 break;
			 case 53:  // rdata[1]="5" - H加2
				 if(H + 2.0f <= 28.0f) H += 2.0f;
			 break;
			 case 54:  // rdata[1]="6" - H减2
				 if(H - 2.0f >= 12.0f) H -= 2.0f;
			 break;
			 default:
			 break;
		 }
	 }
	 else if(*rdata==51)     //rdata[0]="3" - 控制头部三自由度
	 {
		 switch(*(rdata+1))
		 {
			 case 49:  // rdata[1]="1" - Yaw加5
				 if(head_yaw + 5 <= 180) head_yaw += 5;
				 head_motion_mode = HeadMotion_None;
			 break;
			 case 50:  // rdata[1]="2" - Yaw减5
				 if(head_yaw >= 5) head_yaw -= 5;
				 head_motion_mode = HeadMotion_None;
			 break;
			 case 51:  // rdata[1]="3" - Pitch加5
				 if(head_pitch + 5 <= 180) head_pitch += 5;
				 head_motion_mode = HeadMotion_None;
			 break;
			 case 52:  // rdata[1]="4" - Pitch减5
				 if(head_pitch >= 5) head_pitch -= 5;
				 head_motion_mode = HeadMotion_None;
			 break;
			 case 53:  // rdata[1]="5" - Roll加5
				 if(head_roll + 5 <= 180) head_roll += 5;
				 head_motion_mode = HeadMotion_None;
			 break;
			 case 54:  // rdata[1]="6" - Roll减5
				 if(head_roll >= 5) head_roll -= 5;
				 head_motion_mode = HeadMotion_None;
			 break;
			 case 55:  // rdata[1]="7" - 点头动作
				 head_motion_mode = HeadMotion_Nod;
			 break;
			 case 56:  // rdata[1]="8" - 摇头动作
				 head_motion_mode = HeadMotion_Shake;
			 break;
			 case 57:  // rdata[1]="9" - 歪头动作
				 head_motion_mode = HeadMotion_Tilt;
			 break;
			 case 48:  // rdata[1]="0" - 停止动作并回中
				 head_motion_mode = HeadMotion_None;
				 head_yaw = 90;
				 head_pitch = 90;
				 head_roll = 90;
			 break;
			 default:
			 break;
		 }
	 }
	 else if(*rdata==49)     //rdata[0]="1"
	 {
		 flag=0;
		 Mainstate=MainState_Static;
	 }
		 return 1;
}

uint8_t GaitMode_Update(void)
{
	if(*rdata==49)     //rdata[0]="1"
	{
		switch(*(rdata+1))
		 {				 
			 case 48:	//rdata[1]="10"
         GaitMode = GaitMode_Walk;
			 break;	
			 case 49://rdata[1]="11"
         GaitMode = GaitMode_Trot;
			 break;
			 case 50://rdata[1]="12"
         GaitMode = GaitMode_Stand;
			 break;		
			 case 51://rdata[1]="13"
         GaitMode = GaitMode_Body_Twist;
			 break;			 

			 default:
				 GaitMode = GaitMode_Stand;
		 }
	}
  return 1;	
}

uint8_t Avoid_Obstacle(void)
{	
	// US100串口模式：返回2字节数据（高字节+低字节，单位mm）
	// 距离 = (高字节 * 256 + 低字节) / 10 (转换为cm)
	uint16_t distance_mm = (Re_buf2[0] << 8) | Re_buf2[1];
	if(distance_mm > 0 && distance_mm <= 4500)  // 有效范围0-450cm
	{
		distance = distance_mm / 10.0f;  // mm转换为cm，保留小数
	}
	
	// 避障逻辑：只在Trot模式且flag=1时生效
	if(GaitMode==GaitMode_Trot && flag==1)
	{
		if(distance<30&&distance>20)
			Mainstate = MainState_Step;
		else if(distance>0&&distance<=20)
			Mainstate = MainState_Backward;
		else if(distance>=30)
			Mainstate = MainState_Forward;
	}	
	return 1;		
}
