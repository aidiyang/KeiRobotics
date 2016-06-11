/*
 * NRF905.cpp
 *
 *  Created on: 2014¦~11¤ë20¤é
 *      Author: YunKei
 */

#include <NRF905.h>
#include <Delay.h>
#include <Spi.h>
#include <stm32f4xx_gpio.h>
#include <Task.h>
#include <stm32f4xx_it.h>
#include <Buzzer.h>
#include <App.h>
#include <UART.h>

NRF905 * _mNRF905;

void NRF905::setPwr(){
	GPIO_SetBits(PWR_GPIO, PWR_PIN);
}

void NRF905::clrPwr(){
	GPIO_ResetBits(PWR_GPIO, PWR_PIN);
}

void NRF905::setTrxce(){
	GPIO_SetBits(TRXCE_GPIO, TRXCE_PIN);
}

void NRF905::clrTrxce(){
	GPIO_ResetBits(TRXCE_GPIO, TRXCE_PIN);
}

void NRF905::setTxen(){
	GPIO_SetBits(TXEN_GPIO, TXEN_PIN);
}

void NRF905::clrTxen(){
	GPIO_ResetBits(TXEN_GPIO, TXEN_PIN);
}

uint8_t NRF905::readDr(){
	return GPIO_ReadInputDataBit(DR_GPIO, DR_PIN);
}

int rfDelayCount;

void resetNRF905Task(){
	if(rfDelayCount++ > 20){
		rfDelayCount = 0;
		int channel = NRF905::getInstance()->getChannel();
		NRF905::FREQ freq = NRF905::getInstance()->getFreq();
		NRF905::POWER power = NRF905::getInstance()->getPower();
		NRF905::RX_POWER rxPower = NRF905::getInstance()->getRxPower();
		NRF905::RETRAN retrans = NRF905::getInstance()->getReTran();
		uint8_t* rxAddress = NRF905::getInstance()->getRxAddress();
		uint8_t* txAddress = NRF905::getInstance()->getTxAddress();
		int rxpw = NRF905::getInstance()->getRXPW();
		int txpw = NRF905::getInstance()->getTXPW();

		NRF905(channel,	freq, power, rxPower, retrans, rxAddress, txAddress, rxpw, txpw, true);
		App::mApp->mTask->DeAttach(resetNRF905Task);
	}
}

void NRF905::resetNRF905(){
	clrPwr();
	clrTxen();
	clrTrxce();
//	Spi::getInstance(RF_SPI)->resetSpi();
	rfDelayCount = 0;
	App::mApp->mTask->Attach(10, 0, resetNRF905Task, true, -1);
}

void NRF905::setStandby(){
	clrTxen();
	clrTrxce();
}

void NRF905::setTransmit(void){
	setTxen();
	setTrxce();
	Delay::DelayUS(650);
}

void NRF905::setReceive(void){
	setTrxce();
	clrTxen();
	Delay::DelayUS(650);
}

NRF905* NRF905::getInstance(){
	return _mNRF905;
}

int NRF905::getChannel(){
	return _channel;
}

NRF905::FREQ NRF905::getFreq(){
	return _freq;
}

NRF905::POWER NRF905::getPower(){
	return _power;
}

NRF905::RX_POWER NRF905::getRxPower(){
	return _rxPower;
}

NRF905::RETRAN NRF905::getReTran(){
	return _retrans;
}

uint8_t* NRF905::getRxAddress(){
	return _rxAddress;
}

uint8_t* NRF905::getTxAddress(){
	return _txAddress;
}

NRF905::NRF905(int channel, FREQ freq, POWER power, RX_POWER rxPower, RETRAN retrans, uint8_t* rxAddress, uint8_t* txAddress, int rxLength, int txLength, bool createdInstance) : Count(0), NRF905Spi(0), _channel(channel), _freq(freq), _power(power), _rxPower(rxPower), _retrans(retrans), _rxAddress(rxAddress), _txAddress(txAddress), pBuffer(Buffer), NRF905_RXPW(rxLength), NRF905_TXPW(txLength), isTransmitting(false), ErrorCount(0), BufferCount(0){

	if(!createdInstance){
		_mNRF905 = this;
	}
	GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Pin = PWR_PIN;
	GPIO_Init(PWR_GPIO, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = TXEN_PIN;
	GPIO_Init(TXEN_GPIO, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = TRXCE_PIN;
	GPIO_Init(TRXCE_GPIO, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = DR_PIN;
	GPIO_Init(DR_GPIO, &GPIO_InitStruct);

	if(NRF905Spi){
		delete NRF905Spi;
	}

//	NRF905Spi = new Spi(RF_SPI, Spi::PRESCALER8, Spi::SPIMODE0);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Pin = PWR_PIN;
	GPIO_Init(PWR_GPIO, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = TXEN_PIN;
	GPIO_Init(TXEN_GPIO, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = TRXCE_PIN;
	GPIO_Init(TRXCE_GPIO, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Pin = DR_PIN;
	GPIO_Init(DR_GPIO, &GPIO_InitStruct);

	EXTI_DeInit();
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource);
	EXTI_InitStructure.EXTI_Line = EXTI_Line_CH;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQ_CH;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	setPwr();
//	NRF905Spi->WriteCmd(WC, channel);
//	NRF905Spi->WriteCmd(WC + 1, freq | power | rxPower | retrans);
//	NRF905Spi->WriteNBytes(WC + 5, 4, (uint8_t*)rxAddress);
//	NRF905Spi->WriteCmd(WC + 9, UP_CLK_FREQ_500K | UP_CLK_EN | XOF_16M | CRC_EN | CRC_16);
//	NRF905Spi->WriteNBytes(WTA, 4, (uint8_t*)txAddress);
//	NRF905Spi->WriteCmd(RX_PW, NRF905_RXPW);
//	NRF905Spi->WriteCmd(TX_PW, NRF905_TXPW);

	setReceive();
}

bool NRF905::Write(const char* pstr, ...)
{
	if(getIsTransmiting() && ErrorCount < 5){
		ErrorCount++;
		return false;
	}
	ErrorCount = 0;
	va_list arglist;
	char buf[32];
	for(int i = 0; i < 32; i++){
		buf[i] = 0;
	}
	char* fp;
	va_start(arglist, pstr);
	vsprintf(buf, pstr, arglist);
	va_end(arglist);

	fp = buf;

//	NRF905Spi->WriteNBytes(WTP, NRF905_TXPW, (uint8_t*)fp);
	NRF905::getInstance()->setIsTransmiting(true);
	setTransmit();
	return true;
}

int NRF905::Read(unsigned char* _Buffer, int length){

	for(int i = 0; i < length; i++){
		if(pBuffer >= Buffer + 2047){
			pBuffer = Buffer;
		}
		_Buffer[i] = *(pBuffer++);
	}
	_Buffer[length] = '\0';
	BufferCount -= length;
	return BufferCount;
}

bool NRF905::getIsTransmiting(){
	return isTransmitting;
}

void NRF905::setIsTransmiting(bool value){
	isTransmitting = value;
}

char* NRF905::getBuffer(){
	return pBuffer;
}

void NRF905::setBufferCount(int count){
	BufferCount = count;
}

int NRF905::getBufferCount(){
	return BufferCount;
}

void EXTI_IRQHandler()
{
	NRF905::getInstance()->Count++;
    if(EXTI_GetFlagStatus(EXTI_Line_CH) == SET)
	{
    	if(NRF905::getInstance()->getIsTransmiting()){

    		NRF905::getInstance()->setIsTransmiting(false);
    	}
    	else{
    		App::mApp->mLed2->Toggle();
    		if(NRF905::getInstance()->getBufferCount() + NRF905::getInstance()->getRXPW() >= 2047){
    			NRF905::getInstance()->setBufferCount(0);
    		}
    		NRF905::getInstance()->setStandby();
//    		Spi::getInstance(RF_SPI)->ReadNBytes(NRF905::RRP, NRF905::getInstance()->getRXPW(), (uint8_t*)(NRF905::getInstance()->getBuffer() + NRF905::getInstance()->getBufferCount()));
    		NRF905::getInstance()->setBufferCount(NRF905::getInstance()->getBufferCount() + NRF905::getInstance()->getRXPW());
    	}
		NRF905::getInstance()->setReceive();
    }
	EXTI_ClearITPendingBit(EXTI_Line_CH);
}

int NRF905::getRXPW(){
	return NRF905_RXPW;
}

void NRF905::setRXPW(int rxpw){
	NRF905_RXPW = rxpw;
}

int NRF905::getTXPW(){
	return NRF905_TXPW;
}
