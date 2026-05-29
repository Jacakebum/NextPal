#include "stm32f10x.h"
#include "ASRPRO.h"

uint8_t rx_data_debug = 0;
uint8_t test_mode = 0;
uint8_t last_rx_data = 0;
uint8_t rx_new_data = 0;
char rx_buffer[20] = {0};
uint8_t rx_buffer_index = 0;

void ASRPRO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	USART_Cmd(USART2, ENABLE);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void ASRPRO_SendByte(uint8_t byte)
{
	USART_SendData(USART2, byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void ASRPRO_SendString(char* str)
{
	while (*str != '\0')
	{
		ASRPRO_SendByte(*str++);
	}
}

void ASRPRO_Send99(void)
{
	ASRPRO_SendString("99");
}

uint8_t ASRPRO_GetCommand(void)
{
	return rx_data_debug;
}

uint8_t ASRPRO_GetLastRxData(void)
{
	if (rx_new_data)
	{
		rx_new_data = 0;
		return last_rx_data;
	}
	return 0;
}

char* ASRPRO_GetRxBuffer(void)
{
	return rx_buffer;
}

void ASRPRO_ClearRxBuffer(void)
{
	for (uint8_t i = 0; i < 20; i++)
	{
		rx_buffer[i] = 0;
	}
	rx_buffer_index = 0;
}

void ASRPRO_TestMode(void)
{
	static uint32_t test_counter = 0;
	
	test_counter++;
	
	if (test_counter % 2000 == 0)
	{
		test_mode = !test_mode;
		if (test_mode)
		{
			ASRPRO_SendByte(0x01);
		}
		else
		{
			ASRPRO_SendByte(0x02);
		}
	}
}

void ASRPRO_USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		rx_data_debug = USART_ReceiveData(USART2);
		last_rx_data = rx_data_debug;
		rx_new_data = 1;
		
		if (rx_buffer_index < 19)
		{
			rx_buffer[rx_buffer_index++] = rx_data_debug;
			rx_buffer[rx_buffer_index] = '\0';
		}
		
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}
