/*
 * URG_04LX.c
 *
 *  Created on: 2014¦~9¤ë17¤é
 *      Author: YunKei
 */

#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <URG_04LX.h>
#include <Delay.h>
#include <stm32f4xx_it.h>
#include <stm32f4xx_syscfg.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <UART.h>

#define URG_04LX_UART	UART4

uint16_t distance[683];
uint16_t returnNumBytes = 0;
uint16_t length = 0;
//Usart* mURG_04LX_UART;

void switch_printf_uart(USART_TypeDef* UARTx){
	STDOUT_USART = UARTx;
	STDERR_USART = UARTx;
	STDIN_USART = UARTx;
}

void URG_04LX_init(void){
//	mURG_04LX_UART = new Usart(URG_04LX_UART, 19200);
	switch_printf_uart(URG_04LX_UART);
	Delay::DelayMS(100);
	printf("S1152000000000\n");
	Delay::DelayMS(100);
//	delete mURG_04LX_UART;
//	mURG_04LX_UART = new Usart(URG_04LX_UART, 115200);
	Delay::DelayMS(100);
}

uint16_t URG_04LX_get_length(void){
	return length;
}

uint16_t* URG_04LX_get_distance_buffer(void){
	return distance;
}

unsigned char URG_04LX_extract_distance(unsigned char* str){

	uint16_t strCount = 0;
	for(int i = 0; i < length; i++){
		if(strCount != 0 && strCount % 65 == 0){
			strCount++;
		}
		distance[i] = (((uint16_t)str[strCount + 12] - 0x30) << 6) + ((uint16_t)str[strCount + 13] - 0x30);
		strCount += 2;
	}
	return str[10];
}

void URG_04LX_read_distance(float start_angle, float end_angle){

	switch_printf_uart(URG_04LX_UART);
	if(fabs(start_angle) > 120.0f || fabs(end_angle) > 120.0f || end_angle < start_angle){
		return;
	}
	int start_pos = round((start_angle + 135.0f) / 270.0f * 768.0f);
	int end_pos = round((end_angle + 135.0f) / 270.0f * 768.0f);
	char start_ch[4];
	char end_ch[4];
	snprintf(start_ch, 4, "%d", start_pos);
	snprintf(end_ch, 4, "%d", end_pos);
	if(start_pos < 10){
		start_ch[2] = start_ch[0];
		start_ch[1] = '0';
		start_ch[0] = '0';
	}
	if(start_pos < 100){
		start_ch[2] = start_ch[1];
		start_ch[1] = start_ch[0];
		start_ch[0] = '0';
	}
	if(end_pos < 10){
		end_ch[2] = end_ch[0];
		end_ch[1] = '0';
		end_ch[0] = '0';
	}
	if(end_pos < 100){
		end_ch[2] = end_ch[1];
		end_ch[1] = end_ch[0];
		end_ch[0] = '0';
	}
	char input_code[10] = {'G', start_ch[0], start_ch[1], start_ch[2], end_ch[0], end_ch[1], end_ch[2], '0', '2', '\n'};
	switch_printf_uart(URG_04LX_UART);
	printf("%s", input_code);
	length = end_pos - start_pos + 1;
	if(length < 32){
		returnNumBytes = 14 + length * 2;
	}
	else{
		returnNumBytes = 14 + (length * 2 / 64) * 65 + length * 2 % 64;
	}
}

unsigned char URG_04LX_poll(void){

//	if(Usart::getInstance(URG_04LX_UART)->getBufferCount() >= returnNumBytes){
		unsigned char str[2048];
//		Usart::getInstance(URG_04LX_UART)->Read(str, returnNumBytes);
		return URG_04LX_extract_distance(str);
//	}
	return -128;
}
