/*
 * URG_04LX.h
 *
 *  Created on: 2014¦~9¤ë17¤é
 *      Author: YunKei
 */

#ifndef URG_04LX_H_
#define URG_04LX_H_

#include <stm32f4xx.h>

extern USART_TypeDef* STDOUT_USART;
extern USART_TypeDef* STDERR_USART;
extern USART_TypeDef* STDIN_USART;

void switch_printf_uart(USART_TypeDef* UARTx);
void URG_04LX_init(void);
uint16_t URG_04LX_get_length(void);
void URG_04LX_read_distance(float start_angle, float end_angle);
uint16_t* URG_04LX_get_distance_buffer(void);
unsigned char URG_04LX_poll(void);

#endif /* URG_04LX_H_ */
