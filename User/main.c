#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "ASRPRO.h"
#include "TTP223.h"

int main(void)
{
	uint8_t rx_data = 0;
	uint32_t test_count = 0;
	uint32_t send_count = 0;
	uint8_t touch_state = 0;
	uint8_t last_touch_state = 0;
	uint8_t rx_count = 0;
	uint8_t tx_count = 0;

	ASRPRO_Init();
	TTP223_Init();
	OLED_Init();

	OLED_ShowString(1, 1, "Serial Monitor");
	OLED_ShowString(2, 1, "RX: ");
	OLED_ShowString(3, 1, "TX: 99");
	OLED_ShowString(3, 8, "Count:");
	OLED_ShowString(4, 1, "Touch: OFF");

	while (1)
	{
		touch_state = TTP223_Read();

		if (touch_state == 0 && last_touch_state == 1)
		{
			ASRPRO_Send99();
			tx_count++;
			OLED_ShowString(4, 1, "Touch: ON ");
			OLED_ShowNum(3, 14, tx_count, 3);
		}
		else if (touch_state == 1)
		{
			OLED_ShowString(4, 1, "Touch: OFF");
		}

		last_touch_state = touch_state;

		rx_data = ASRPRO_GetLastRxData();

		if (rx_data != 0)
		{
			rx_count++;
			OLED_ShowString(2, 1, "RX: ");
			OLED_ShowString(2, 4, ASRPRO_GetRxBuffer());
			ASRPRO_ClearRxBuffer();
		}

		send_count++;
		if (send_count % 5000 == 0)
		{
			ASRPRO_SendString("Test\r\n");
			tx_count++;
			OLED_ShowString(1, 1, "Sending Test...");
			OLED_ShowNum(3, 14, tx_count, 3);
			Delay_ms(500);
			OLED_ShowString(1, 1, "Serial Monitor");
		}

		test_count++;
		if (test_count % 1000 == 0)
		{
			OLED_ShowNum(4, 8, test_count / 1000, 3);
		}

		Delay_ms(10);
	}
}
