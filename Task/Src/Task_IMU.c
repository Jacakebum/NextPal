#include "Task_init.h"
#include "MPU6050.h"
#include "oled.h"
#include "Task_StateMachine.h"
#include "stdio.h"
#include "Inverse_K.h"

extern volatile uint8_t bluetooth_connected;
extern volatile uint8_t last_rx_cmd[3];

// 速度相关变量
static float calculate_speed(void)
{
    GaitMode_t gait_mode = GetGaitMode();
    MainState_t main_state = GetMainState();
    
    // 如果是静态状态，速度为0
    if(main_state == MainState_Static || main_state == MainState_IMU)
    {
        return 0.0f;
    }
    
    // 根据不同的步态模式计算速度
    switch(gait_mode)
    {
        case GaitMode_Trot:
            // Trot步态：速度 = 步长S * (1000ms / (T * 时间步))
            return S * (100.0f / T);  // mm/s
        case GaitMode_Body_Twist:
            // 扭扭动：速度为0
            return 0.0f;
        default:
            // 其他步态：使用Trot的速度计算
            return S * (100.0f / T);
    }
}

static uint8_t IMU_attitude_update(void);
static const char* GetGaitModeString(GaitMode_t mode);
static const char* GetMainStateString(MainState_t state);
static const char* GetBluetoothStatusString(uint8_t connected);

void Task_IMU_Start(void *parameters)
{
	static int t=0;
	static int display_counter = 0;
  TickType_t xLastWakeUpTime;
  xLastWakeUpTime = xTaskGetTickCount();
  while(1)
  {
	  MUP6050_Data_Process();
	  IMU_attitude_update();
		  
		if(t>20)
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
			t=0;
		}
		t++;
		
		display_counter++;
		if(display_counter >= 5)
		{
			char buf[17];
			
			// 显示俯仰角和距离（合并到第一行），用空格填充到16字符
			sprintf(buf, "P:%.0f          ", angle[1]);
			buf[16] = '\0';
			OLED_ShowString(1, 1, buf);
			
			// 第2行：显示步态模式和动作状态，用空格填充
			const char* gait_str = GetGaitModeString(GetGaitMode());
			const char* action_str = GetMainStateString(GetMainState());
			sprintf(buf, "G:%s A:%s        ", gait_str, action_str);
			buf[16] = '\0';
			OLED_ShowString(2, 1, buf);
			
			// 第3行：显示蓝牙状态、数据和参数，用空格填充
			const char* bt_str = GetBluetoothStatusString(bluetooth_connected);
			// 显示收到的2个字符（直接显示ASCII字符）
			sprintf(buf, "BT:%s D:%s S:%.0f   ", bt_str, (char*)last_rx_cmd, S);
			buf[16] = '\0';
			OLED_ShowString(3, 1, buf);
			
			// 第4行：显示T、H和速度信息
			float speed = calculate_speed();
			sprintf(buf, "T:%.0f H:%.0f SP:%.0f  ", T, H, speed);
			buf[16] = '\0';
			OLED_ShowString(4, 1, buf);
			
			display_counter = 0;
		}
		
	  vTaskDelayUntil(&xLastWakeUpTime, 5);
  }
}

uint8_t IMU_attitude_update(void)
{
	return 1;
}

static const char* GetGaitModeString(GaitMode_t mode)
{
	switch(mode)
	{
		case GaitMode_Stand:
			return "Stand";
		case GaitMode_Walk:
			return "Walk";
		case GaitMode_Trot:
			return "Trot";
		case GaitMode_Crawl:
			return "Crawl";
		case GaitMode_Body_Twist:
			return "Twist";
		default:
			return "Unk";
	}
}

static const char* GetMainStateString(MainState_t state)
{
	switch(state)
	{
		case MainState_Static:
			return "Idle";
		case MainState_Step:
			return "Step";
		case MainState_Forward:
			return "Fwd";
		case MainState_Backward:
			return "Back";
		case MainState_Move_to_Left:
			return "Left";
		case MainState_Move_to_Right:
			return "Right";
		case MainState_Turn_Left:
			return "TLft";
		case MainState_Turn_Right:
			return "TRgt";
		case MainState_IMU:
			return "IMU";
		case MainState_Avoid_Obstacle:
			return "Avoid";
		default:
			return "Unk";
	}
}

static const char* GetBluetoothStatusString(uint8_t connected)
{
	if(connected)
	{
		return "OK";
	}
	else
	{
		return "--";
	}
}

