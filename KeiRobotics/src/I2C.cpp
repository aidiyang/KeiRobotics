/*
 * I2C.cpp
 *
 *  Created on: 2014¦~8¤ë18¤é
 *      Author: YunKei
 */

#include <stdio.h>
#include <I2C.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_it.h>
#include <Ticks.h>
#include <Delay.h>
#include <AdditionalTools.h>
#include <Buzzer.h>
#include <Task.h>
#include <App.h>
#include <Led.h>

I2C::I2CConfiguration::I2CConfiguration(I2C_TypeDef* I2Cx, Configuration* scl, Configuration* sda, CLOCK clock) : _I2Cx(I2Cx), _scl(scl), _sda(sda), _clock(clock){
}

I2C::I2C(I2CConfiguration* conf) : ErrorCount(0){
	Conf = conf;
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

	I2C_InitStruct.I2C_ClockSpeed = conf->_clock;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	uint8_t sclSource;
	for(int i = 0; i < 16; i++){
		if(conf->_scl->_pin == _BV(i)){
			sclSource = i;
		}
	}
	uint8_t sdaSource;
	for(int i = 0; i < 16; i++){
		if(conf->_sda->_pin == _BV(i)){
			sdaSource = i;
		}
	}
	if(conf->_I2Cx == I2C1)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	    RCC_AHB1PeriphClockCmd(conf->_scl->_rcc, ENABLE);
		GPIO_InitStruct.GPIO_Pin = conf->_scl->_pin;
		GPIO_PinAFConfig(conf->_scl->_port, sclSource, GPIO_AF_I2C1);
		GPIO_Init(conf->_scl->_port, &GPIO_InitStruct);

	    RCC_AHB1PeriphClockCmd(conf->_sda->_rcc, ENABLE);
		GPIO_InitStruct.GPIO_Pin = conf->_sda->_pin;
		GPIO_PinAFConfig(conf->_sda->_port, sdaSource, GPIO_AF_I2C1);
		GPIO_Init(conf->_sda->_port, &GPIO_InitStruct);
	}
	else if(conf->_I2Cx == I2C2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	    RCC_AHB1PeriphClockCmd(conf->_scl->_rcc, ENABLE);
		GPIO_InitStruct.GPIO_Pin = conf->_scl->_pin;
		GPIO_PinAFConfig(conf->_scl->_port, sclSource, GPIO_AF_I2C2);
		GPIO_Init(conf->_scl->_port, &GPIO_InitStruct);

	    RCC_AHB1PeriphClockCmd(conf->_sda->_rcc, ENABLE);
		GPIO_InitStruct.GPIO_Pin = conf->_sda->_pin;
		GPIO_PinAFConfig(conf->_sda->_port, sdaSource, GPIO_AF_I2C2);
		GPIO_Init(conf->_sda->_port, &GPIO_InitStruct);
	}
	I2C_Init(conf->_I2Cx, &I2C_InitStruct);
	I2C_Cmd(conf->_I2Cx, ENABLE);
}

void I2C::ResetI2C(){
	ErrorCount++;
	I2C_DeInit(Conf->_I2Cx);
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Pin = Conf->_scl->_pin;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(Conf->_scl->_port, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = Conf->_sda->_pin;
	GPIO_Init(Conf->_sda->_port, &GPIO_InitStruct);

	GPIO_WriteBit(Conf->_sda->_port, Conf->_sda->_pin, Bit_RESET);
	GPIO_WriteBit(Conf->_scl->_port, Conf->_scl->_pin, Bit_SET);
	Delay::DelayUS(1);
	GPIO_WriteBit(Conf->_scl->_port, Conf->_scl->_pin, Bit_RESET);
	Delay::DelayUS(1);
	GPIO_WriteBit(Conf->_scl->_port, Conf->_scl->_pin, Bit_SET);
	Delay::DelayUS(1);
	if(App::mApp->mI2C1 == this){
		I2C mI2C1(Conf);
	}
	else if(App::mApp->mI2C2 == this){
		I2C mI2C2(Conf);
	}
	printf("I2C Error\n");
}

bool I2C::Write(uint8_t addr, uint8_t reg, uint8_t data){

	App::mApp->mTicks->setTimeout(3);
	while(I2C_GetFlagStatus(Conf->_I2Cx, I2C_FLAG_BUSY)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_GenerateSTART(Conf->_I2Cx, ENABLE);

	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_Send7bitAddress(Conf->_I2Cx, addr << 1, I2C_Direction_Transmitter);

	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_SendData(Conf->_I2Cx, reg);
	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_SendData(Conf->_I2Cx, data);
	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_GenerateSTOP(Conf->_I2Cx, ENABLE);
	return true;
}

uint8_t I2C::Read(uint8_t addr, uint8_t reg){

	App::mApp->mTicks->setTimeout(3);
	while(I2C_GetFlagStatus(Conf->_I2Cx, I2C_FLAG_BUSY)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return 0;
		}
	}
	I2C_GenerateSTART(Conf->_I2Cx, ENABLE);

	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return 0;
		}
	}
	I2C_Send7bitAddress(Conf->_I2Cx, addr << 1, I2C_Direction_Transmitter);

	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return 0;
		}
	}

	I2C_SendData(Conf->_I2Cx, reg);

	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return 0;
		}
	}

	I2C_GenerateSTART(Conf->_I2Cx, ENABLE);

	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return 0;
		}
	}
	I2C_Send7bitAddress(Conf->_I2Cx, addr << 1, I2C_Direction_Receiver);

	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return 0;
		}
	}

	I2C_AcknowledgeConfig(Conf->_I2Cx, DISABLE);

	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return 0;
		}
	}
	uint8_t value = I2C_ReceiveData(Conf->_I2Cx);
	I2C_GenerateSTOP(Conf->_I2Cx, ENABLE);

	return value;
}

bool I2C::BurstWrite(uint8_t addr, uint8_t reg, uint8_t length, uint8_t* pdata){

	App::mApp->mTicks->setTimeout(3);
	while(I2C_GetFlagStatus(Conf->_I2Cx, I2C_FLAG_BUSY)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
			}
	}
	I2C_GenerateSTART(Conf->_I2Cx, ENABLE);

	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_Send7bitAddress(Conf->_I2Cx, addr << 1, I2C_Direction_Transmitter);
	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_SendData(Conf->_I2Cx, reg);

	for(int i = 0; i < length; i++){
		App::mApp->mTicks->setTimeout(3);
		while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
			if(App::mApp->mTicks->Timeout()){
				ResetI2C();
				return false;
			}
		}
		I2C_SendData(Conf->_I2Cx, pdata[i]);
	}
	I2C_GenerateSTOP(Conf->_I2Cx, ENABLE);
	return true;
}

bool I2C::BurstRead(uint8_t addr, uint8_t reg, uint8_t length, uint8_t* pdata){
	App::mApp->mTicks->setTimeout(3);
	while(I2C_GetFlagStatus(Conf->_I2Cx, I2C_FLAG_BUSY)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_GenerateSTART(Conf->_I2Cx, ENABLE);

	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_Send7bitAddress(Conf->_I2Cx, addr << 1, I2C_Direction_Transmitter);

	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
		}
	}

	I2C_SendData(Conf->_I2Cx, reg);

	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
		}
	}

	I2C_GenerateSTART(Conf->_I2Cx, ENABLE);
	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
		}
	}
	I2C_Send7bitAddress(Conf->_I2Cx, addr << 1, I2C_Direction_Receiver);

	App::mApp->mTicks->setTimeout(3);
	while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
		if(App::mApp->mTicks->Timeout()){
			ResetI2C();
			return false;
		}
	}

	for(int i = 0; i < length; i++){
		if(i == length - 1){
			I2C_AcknowledgeConfig(Conf->_I2Cx, DISABLE);
		}
		else if(i < length - 1){
			I2C_AcknowledgeConfig(Conf->_I2Cx, ENABLE);
		}
		App::mApp->mTicks->setTimeout(3);
		while(!I2C_CheckEvent(Conf->_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)){
			if(App::mApp->mTicks->Timeout()){
				ResetI2C();
				return false;
			}
		}
		pdata[i] = I2C_ReceiveData(Conf->_I2Cx);
	}
	I2C_GenerateSTOP(Conf->_I2Cx, ENABLE);
	return true;
}
