/*
 * Spi.cpp
 *
 *  Created on: 2014¦~8¤ë7¤é
 *      Author: YunKei
 */

#include <App.h>
#include <Spi.h>
#include <Delay.h>
#include <stdio.h>
#include <stdarg.h>
#include <stm32f4xx_spi.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <Ticks.h>
#include <Task.h>
#include <UART.h>
#include <stm32f4xx_it.h>
#include <AdditionalTools.h>

using namespace Communication;
using namespace Utility;

//void DMA1_Stream3_IRQHandler(){
//	if(DMA_GetITStatus(DMA1_Stream3, DMA_IT_TCIF3) == SET)
//	{
//		DMA_ClearITPendingBit (DMA1_Stream3, DMA_IT_TCIF3);
//		DMA_Cmd(DMA1_Stream3, DISABLE);
//		App::mApp->mTicks->setTimeout(3);
//		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){
//			if(App::mApp->mTicks->Timeout()){
//				return;
//			}
//		}
//		for(int i = 0; i < 5; i++){
//			App::mApp->mSpi2->Buffer[App::mApp->mSpi2->BufferCount++] = App::mApp->mSpi2->rxBuffer[i];
//			App::mApp->mSpi2->AvailableLength++;
//			if(App::mApp->mSpi2->BufferCount >= 2047){
//				App::mApp->mSpi2->BufferCount = 0;
//			}
//		}
//		DMA_Cmd(DMA1_Stream3, ENABLE);
//	}
//}

void DMA1_Stream4_IRQHandler(){
	if(DMA_GetITStatus(DMA1_Stream4, DMA_IT_TCIF4) == SET)
	{
		DMA_ClearITPendingBit (DMA1_Stream4, DMA_IT_TCIF4);
		DMA_Cmd(DMA1_Stream4, DISABLE);
		App::mApp->mTicks->setTimeout(3);
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){
			if(App::mApp->mTicks->Timeout()){
				return;
			}
		}
		App::mApp->mUART3->isDmaBusy = false;
	}
}

void SPI1_IRQHandler()
{
	App::mApp->mTicks->setTimeout(3);
	if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == SET){
		App::mApp->mSpi1->Buffer[App::mApp->mSpi1->BufferCount++] = (char)(0x00ff & SPI_I2S_ReceiveData(SPI1));
		App::mApp->mSpi1->AvailableLength++;
		if(App::mApp->mSpi1->BufferCount >= 2047){
			App::mApp->mSpi1->BufferCount = 0;
			App::mApp->mSpi1->AvailableLength = 0;
		}
		if(App::mApp->mSpi1->Conf->IsSlave){
			if(App::mApp->mSpi1->SlaveTxLength > 0){
				App::mApp->mTicks->setTimeout(3);
				while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){
					if(App::mApp->mTicks->Timeout()){
						return;
					}
				}
				SPI_I2S_SendData(SPI1, App::mApp->mSpi1->SlaveTxBuffer[App::mApp->mSpi1->SlaveTxBufferCount++]);
				App::mApp->mSpi1->SlaveTxLength--;
				if(App::mApp->mSpi1->SlaveTxBufferCount >= 2047){
					App::mApp->mSpi1->SlaveTxBufferCount = 0;
					App::mApp->mSpi1->SlaveTxLength = 0;
				}
			}
		}
	}

}

void SPI2_IRQHandler()
{
	App::mApp->mTicks->setTimeout(3);
	if(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == SET){
		App::mApp->mSpi2->Buffer[App::mApp->mSpi2->BufferCount++] = (char)(0x00ff & SPI_I2S_ReceiveData(SPI2));
		App::mApp->mSpi2->AvailableLength++;
		if(App::mApp->mSpi2->BufferCount >= 2047){
			App::mApp->mSpi2->BufferCount = 0;
			App::mApp->mSpi2->AvailableLength = 0;
		}
		if(App::mApp->mSpi2->Conf->IsSlave){
			if(App::mApp->mSpi2->SlaveTxLength > 0){
				App::mApp->mTicks->setTimeout(3);
				while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){
					if(App::mApp->mTicks->Timeout()){
						return;
					}
				}
				SPI_I2S_SendData(SPI2, App::mApp->mSpi2->SlaveTxBuffer[App::mApp->mSpi2->SlaveTxBufferCount++]);
				App::mApp->mSpi2->SlaveTxLength--;
				if(App::mApp->mSpi2->SlaveTxBufferCount >= 2047){
					App::mApp->mSpi2->SlaveTxBufferCount = 0;
					App::mApp->mSpi2->SlaveTxLength = 0;
				}
			}
		}
	}
}

void Spi::setSlaveTxBuffer(char* data, int length){
	if(SlaveTxBufferCount + SlaveTxLength >= 2047){
		SlaveTxBufferCount = 0;
		SlaveTxLength = 0;
	}
	pSlaveTxBuffer = &SlaveTxBuffer[SlaveTxBufferCount];
	for(int i = 0; i < length; i++){
		if(pSlaveTxBuffer + i >= SlaveTxBuffer + 2047){
			pSlaveTxBuffer = SlaveTxBuffer;
		}
		pSlaveTxBuffer[i] = *(data++);
	}
	SlaveTxLength += length;
}

Spi::SpiConfiguration::SpiConfiguration(SPIConfx spiConfx,
		PRESCALER prescaler,
		SPIMODE spimode,
		Configuration* sclk,
		Configuration* miso,
		Configuration* mosi,
		Configuration** cs,
		bool isSlave,
		int numOfDevices) : SpiConfx(spiConfx),
				Prescaler(prescaler),
				Spimode(spimode),
				SCLK(sclk),
				MOSI(mosi),
				MISO(miso),
				CS(cs),
				IsSlave(isSlave),
				NumOfDevices(numOfDevices){
}

Spi::Spi(SpiConfiguration* conf) : isDmaBusy(false), Conf(conf), Spix(0), BufferCount(0), pBuffer(Buffer), AvailableLength(0), pSlaveTxBuffer(SlaveTxBuffer), SlaveTxLength(0), SlaveTxBufferCount(0){
	Initialize(conf);
}

void Spi::Initialize(SpiConfiguration* conf){
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	uint16_t prescaler;
	uint8_t GPIO_AF_SPIx;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	if(conf->SpiConfx == SpiConfiguration::SpiConf1){
		Spix = SPI1;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
		GPIO_AF_SPIx = GPIO_AF_SPI1;
	}
	else if(conf->SpiConfx == SpiConfiguration::SpiConf2){
		Spix = SPI2;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		//NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
		GPIO_AF_SPIx = GPIO_AF_SPI2;

		DMA_DeInit(DMA1_Stream4);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

		DMA_InitStructure.DMA_BufferSize = 32;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI2->DR)) ;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;

		DMA_InitStructure.DMA_Channel = DMA_Channel_0 ;
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
		DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)(txBuffer) ;
		DMA_Init(DMA1_Stream4, &DMA_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init (&NVIC_InitStructure);

		SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
		DMA_ITConfig (DMA1_Stream0, DMA_IT_TC, ENABLE);

		DMA_DeInit(DMA1_Stream3);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

		DMA_InitStructure.DMA_BufferSize = 5;
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI2->DR)) ;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;

		DMA_InitStructure.DMA_Channel = DMA_Channel_0 ;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
		DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)(rxBuffer) ;
		DMA_Init(DMA1_Stream4, &DMA_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init (&NVIC_InitStructure);

		SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
		DMA_ITConfig (DMA1_Stream0, DMA_IT_TC, ENABLE);
	}

	SPI_I2S_DeInit(Spix);
	switch(conf->Prescaler)
	{
		case SpiConfiguration::PRESCALER2:
			prescaler = SPI_BaudRatePrescaler_2;
			break;
		case SpiConfiguration::PRESCALER4:
			prescaler = SPI_BaudRatePrescaler_4;
			break;
		case SpiConfiguration::PRESCALER8:
			prescaler = SPI_BaudRatePrescaler_8;
			break;
		case SpiConfiguration::PRESCALER16:
			prescaler = SPI_BaudRatePrescaler_16;
			break;
		case SpiConfiguration::PRESCALER32:
			prescaler = SPI_BaudRatePrescaler_32;
			break;
		case SpiConfiguration::PRESCALER64:
			prescaler = SPI_BaudRatePrescaler_64;
			break;
		case SpiConfiguration::PRESCALER128:
			prescaler = SPI_BaudRatePrescaler_128;
			break;
		case SpiConfiguration::PRESCALER256:
			prescaler = SPI_BaudRatePrescaler_256;
			break;
		default:
			break;
	}

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	if(conf->IsSlave){
		SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
		RCC_AHB1PeriphClockCmd(conf->CS[0]->_rcc, ENABLE);
		GPIO_InitStructure.GPIO_Pin = conf->CS[0]->_pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(conf->CS[0]->_port, &GPIO_InitStructure);
		uint8_t csSource;
		for(int i = 0; i < 16; i++){
			if(conf->CS[0]->_pin == _BV(i)){
				csSource = i;
			}
		}
		GPIO_PinAFConfig(conf->CS[0]->_port, csSource, GPIO_AF_SPIx);
	}
	else{
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		for(int i = 0; i < conf->NumOfDevices; i++){
			RCC_AHB1PeriphClockCmd(conf->CS[i]->_rcc, ENABLE);
			GPIO_InitStructure.GPIO_Pin = conf->CS[i]->_pin;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_Init(conf->CS[i]->_port, &GPIO_InitStructure);
			GPIO_SetBits(conf->CS[i]->_port, conf->CS[i]->_pin);
		}
	}

	switch(conf->Spimode){

		case SpiConfiguration::SPIMODE0:
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
			break;

		case SpiConfiguration::SPIMODE1:
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
			break;

		case SpiConfiguration::SPIMODE2:
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
			break;

		case SpiConfiguration::SPIMODE3:
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
			break;

		default:
			break;
	}

	SPI_InitStructure.SPI_BaudRatePrescaler = prescaler;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	RCC_AHB1PeriphClockCmd(conf->SCLK->_rcc, ENABLE);
	GPIO_InitStructure.GPIO_Pin = conf->SCLK->_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(conf->SCLK->_port, &GPIO_InitStructure);
	uint8_t clkSource;
	for(int i = 0; i < 16; i++){
		if(conf->SCLK->_pin == _BV(i)){
			clkSource = i;
		}
	}
	GPIO_PinAFConfig(conf->SCLK->_port, clkSource, GPIO_AF_SPIx);

	RCC_AHB1PeriphClockCmd(conf->MISO->_rcc, ENABLE);
	GPIO_InitStructure.GPIO_Pin = conf->MISO->_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(conf->MISO->_port, &GPIO_InitStructure);
	uint8_t misoSource;
	for(int i = 0; i < 16; i++){
		if(conf->MISO->_pin == _BV(i)){
			misoSource = i;
		}
	}
	GPIO_PinAFConfig(conf->MISO->_port, misoSource, GPIO_AF_SPIx);


	RCC_AHB1PeriphClockCmd(conf->MOSI->_rcc, ENABLE);
	GPIO_InitStructure.GPIO_Pin = conf->MOSI->_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_Init(conf->MOSI->_port, &GPIO_InitStructure);
	uint8_t mosiSource;
	for(int i = 0; i < 16; i++){
		if(conf->MOSI->_pin == _BV(i)){
			mosiSource = i;
		}
	}
	GPIO_PinAFConfig(conf->MOSI->_port, mosiSource, GPIO_AF_SPIx);


//	SPI_I2S_ITConfig(Spix, SPI_I2S_IT_RXNE, ENABLE);

	SPI_Init(Spix, &SPI_InitStructure);
	SPI_Cmd(Spix, ENABLE);
}

bool Spi::SendByte(uint8_t byte){
	App::mApp->mTicks->setTimeout(3);
	while(SPI_I2S_GetFlagStatus(Spix, SPI_I2S_FLAG_TXE) == RESET){
		if(App::mApp->mTicks->Timeout()){
			return false;
		}
	}
	SPI_I2S_SendData(Spix, byte);
	return true;
}

bool Spi::Byte(uint8_t byte, uint8_t* data){
	App::mApp->mTicks->setTimeout(3);
	while(SPI_I2S_GetFlagStatus(Spix, SPI_I2S_FLAG_TXE) == RESET){
		if(App::mApp->mTicks->Timeout()){
			return false;
		}
	}

	SPI_I2S_SendData(Spix, byte);

	App::mApp->mTicks->setTimeout(3);
	while(SPI_I2S_GetFlagStatus(Spix, SPI_I2S_FLAG_RXNE) == RESET){
		if(App::mApp->mTicks->Timeout()){
			return false;
		}
	}
	*data = (uint8_t)SPI_I2S_ReceiveData(Spix);
	return true;
}

int Spi::WriteRead(int index, uint8_t data){

	uint8_t value = 0;
	ChipSelect(index);
	Byte(data, &value);
	ChipDeSelect(index);
	return value;
}

bool Spi::WriteCmd(int index, uint8_t reg, uint8_t cmd){

	uint8_t v = 0;
	ChipSelect(index);
	if(!Byte(reg, &v)){
		return false;
	}
	if(!Byte(cmd, &v)){
		return false;
	}
	ChipDeSelect(index);
	return true;
}

void Spi::Print(int index, const char* pstr, ...)
{
	int length = 0;
	va_list arglist;
	char* fp;
	for(int i = 0; i < 128; i++){
		txBuffer[i] = 0;
	}
	va_start(arglist, pstr);
	vsprintf(txBuffer, pstr, arglist);
	va_end(arglist);

	fp = txBuffer;

	while(*(fp++)){
		length++;
	}
	if(Conf->IsSlave){
		//setSlaveTxBuffer(txBuffer, length);
		DMA_SetCurrDataCounter(DMA1_Stream4, length);
		DMA_Cmd(DMA1_Stream4, ENABLE);
	}
	else{
		for(int i = 0; i < length; i++){
			Transfer(index, txBuffer[i]);
		}
	}
}

bool Spi::Transfer(int index, uint8_t data){
	uint8_t value = 0;
	ChipSelect(index);
	if(!SendByte(data)){
		return false;
	}
	ChipDeSelect(index);
	return true;
}

int Spi::SlaveTransfer(){

	uint8_t value = 0;
	if(!Byte(0, &value)){
		return 0;
	}
	return value;
}

bool Spi::ReadData(int index, uint8_t reg, uint8_t* value){

	uint8_t v = 0;
	ChipSelect(index);
	if(!Byte(reg, &v)){
		return false;
	}

	if(!Byte(0x00, value)){
		return false;
	}
	ChipDeSelect(index);
	return true;
}

bool Spi::WriteNBytes(int index, uint8_t reg, uint8_t length, uint8_t* pData){

	int i = 0;
	uint8_t v = 0;
	ChipSelect(index);
	if(!Byte(reg, &v)){
		return false;
	}
	for(i = 0; i < length; i++){
		if(!Byte(*(pData + i), &v)){
			return false;
		}
	}
	ChipDeSelect(index);
	return true;
}

bool Spi::ReadNBytes(int index, uint8_t reg, uint8_t length, uint8_t* pData){

	int i = 0;
	uint8_t v = 0;
	ChipSelect(index);
	if(!Byte(reg, &v)){
		return false;
	}
	for(i = 0; i < length; i++){
		if(!Byte(0x00, (pData + i))){
			return false;
		}
	}
	ChipDeSelect(index);
	return true;
}

void Spi::ChipSelect(int index){
	GPIO_ResetBits(Conf->CS[index]->_port, Conf->CS[index]->_pin);
	Delay::DelayUS(10);
}

void Spi::ChipDeSelect(int index){
	Delay::DelayUS(10);
	GPIO_SetBits(Conf->CS[index]->_port, Conf->CS[index]->_pin);
}

int Spi::Read(char* buffer, int length){
	pBuffer = &Buffer[BufferCount - AvailableLength];
	for(int i = 0; i < length; i++){
		if(pBuffer >= Buffer + 2047){
			pBuffer = Buffer;
			BufferCount = 0;
			AvailableLength = 0;
			return -1;
		}
		buffer[i] = *(pBuffer++);
	}
	buffer[length] = '\0';
	AvailableLength -= length;
	return AvailableLength;
}
