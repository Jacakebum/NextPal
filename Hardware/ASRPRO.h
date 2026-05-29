#ifndef __ASRPRO_H
#define __ASRPRO_H

#include "stm32f10x.h"

void ASRPRO_Init(void);
uint8_t ASRPRO_GetCommand(void);
uint8_t ASRPRO_GetLastRxData(void);
char* ASRPRO_GetRxBuffer(void);
void ASRPRO_ClearRxBuffer(void);
void ASRPRO_SendByte(uint8_t byte);
void ASRPRO_SendString(char* str);
void ASRPRO_Send99(void);
void ASRPRO_TestMode(void);
void ASRPRO_USART2_IRQHandler(void);

#endif
