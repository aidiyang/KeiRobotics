/*
 * CAN.cpp
 *
 *  Created on: 2016¦~6¤ë27¤é
 *      Author: wongy
 */

#include <CAN.h>
#include <AdditionalTools.h>
#include <stdio.h>
#include <stdarg.h>
#include <ticks.h>
#include <App.h>

using namespace Communication;

CANConfiguration::CANConfiguration(CANConfx CANx, CANBAUDRATE baudrate, Configuration* tx, Configuration* rx) : _CANx(CANx), _baudrate(baudrate), _tx(tx), _rx(rx){

}

CAN::CAN(CANConfiguration* conf) : TxID(0), RxLength(0), TxLength(0), RxID(0), AvailablePackageCount(0), AvailablePackage(0), SendLength(0), pTxBuffer(txBuffer), txBufferCount(0), LineFeedCount(0), Conf(conf), BufferCount(0), pBuffer(Buffer), AvailableLength(0){
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t prescaler;
	uint8_t GPIO_AF_CANx;

	if(conf->_CANx == CANConfiguration::CANConf1){
		CANx = CAN1;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
		GPIO_AF_CANx = GPIO_AF_CAN1;
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	}
	else if(conf->_CANx == CANConfiguration::CANConf2){
		CANx = CAN2;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
		GPIO_AF_CANx = GPIO_AF_CAN2;
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
	}

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	switch(conf->_baudrate){
		case CANConfiguration::CANBAUDRATE2M:
			prescaler = 1;
			break;
		case CANConfiguration::CANBAUDRATE1M:
			prescaler = 2;
			break;
		case CANConfiguration::CANBAUDRATE500K:
			prescaler = 4;
			break;
		case CANConfiguration::CANBAUDRATE400K:
			prescaler = 5;
			break;
		case CANConfiguration::CANBAUDRATE250K:
			prescaler = 8;
			break;
		case CANConfiguration::CANBAUDRATE200K:
			prescaler = 10;
			break;
		case CANConfiguration::CANBAUDRATE125K:
			prescaler = 16;
			break;
		case CANConfiguration::CANBAUDRATE100K:
			prescaler = 20;
			break;
		case CANConfiguration::CANBAUDRATE50K:
			prescaler = 40;
			break;
		case CANConfiguration::CANBAUDRATE40K:
			prescaler = 80;
			break;
		case CANConfiguration::CANBAUDRATE10K:
			prescaler = 200;
			break;
	}

	CAN_DeInit(CANx);

	CAN_InitStructure.CAN_Prescaler = prescaler;
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = ENABLE;
	CAN_InitStructure.CAN_TXFP = ENABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_12tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;

	CAN_Init(CANx, &CAN_InitStructure);

	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	RCC_AHB1PeriphClockCmd(conf->_tx->_rcc, ENABLE);
	GPIO_InitStructure.GPIO_Pin = conf->_tx->_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(conf->_tx->_port, &GPIO_InitStructure);
	uint8_t _txSource;
	for(int i = 0; i < 16; i++){
		if(conf->_tx->_pin == _BV(i)){
			_txSource = i;
		}
	}
	GPIO_PinAFConfig(conf->_tx->_port, _txSource, GPIO_AF_CANx);

	RCC_AHB1PeriphClockCmd(conf->_rx->_rcc, ENABLE);
	GPIO_InitStructure.GPIO_Pin = conf->_rx->_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(conf->_rx->_port, &GPIO_InitStructure);
	uint8_t _rxSource;
	for(int i = 0; i < 16; i++){
		if(conf->_rx->_pin == _BV(i)){
			_rxSource = i;
		}
	}
	GPIO_PinAFConfig(conf->_rx->_port, _rxSource, GPIO_AF_CANx);

	TxMsg.StdId = 0x0;
	TxMsg.ExtId = 0x00;
	TxMsg.RTR = CAN_RTR_DATA;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.DLC = 0;

	RxMsg.StdId = 0x0;
	RxMsg.ExtId = 0x00;
	RxMsg.RTR = CAN_RTR_DATA;
	RxMsg.IDE = CAN_ID_STD;
	RxMsg.DLC = 0;
	RxMsg.FMI = 0;

	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
}

void CAN::StartReceive(){
	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
}

int CAN::Transmit(uint8_t* data){
	TxMsg.StdId = TxID;
	TxMsg.DLC = TxLength;
	for(int i = 0; i < TxLength; i++){
		TxMsg.Data[i] = data[i];
	}
	int TransMailbox = CAN_Transmit(CANx, &TxMsg);
	App::mApp->mTicks->setTimeout(3);
	while (CAN_TransmitStatus(CANx, TransMailbox) != CAN_TxStatus_Ok){
		if(App::mApp->mTicks->Timeout()){
			return 0;
		}
	}
	return TransMailbox;
}

bool CAN::Receive(uint8_t* data){
	RxMsg.DLC = RxLength;
	if(RxLength > 0){
		if(CAN_MessagePending(CANx, CAN_FIFO0) > 0){
			CAN_Receive(CANx, CAN_FIFO0, &RxMsg);
			if(AvailablePackageCount >= 2047){
				AvailablePackageCount = 0;
				AvailablePackage = 0;
				BufferCount = 0;
				AvailableLength = 0;
				return false;
			}
			ID[AvailablePackageCount] = RxMsg.StdId;
			Length[AvailablePackageCount] = RxLength;
			for(int i = 0; i < RxLength; i++){
				data[i] = RxMsg.Data[i];
				RxMsg.Data[i] = 0;
			}
			AvailablePackageCount++;
			AvailablePackage++;
		}
	}
	return true;
}

void CAN::SendPoll(){
	if(SendLength > TxLength){
		char ch[TxLength + 1];
		for(int i = 0; i < TxLength; i++){
			ch[i] = txBuffer[txBufferCount - SendLength + i];
		}
		ch[TxLength] = '\0';
		printf("%s", ch);
		Transmit((uint8_t*)ch);
		SendLength -= TxLength;
	}
}

void CAN::ReceivePoll(){
	uint8_t data[RxLength];
	Receive(data);
	for(int i = 0; i < RxLength; i++){
		if(BufferCount >= 2047){
			BufferCount = 0;
		}
		Buffer[BufferCount++] = data[i];
		if(data[i] == '\n'){
			LineFeedCount++;
		}
	}
	AvailableLength += RxLength;
}

int CAN::Read(char* buffer, int length){
	pBuffer = &Buffer[BufferCount - AvailableLength];
	for(int i = 0; i < length; i++){
		if(pBuffer >= Buffer + 2047){
			pBuffer = Buffer;
			BufferCount = 0;
		}
		buffer[i] = *(pBuffer++);
	}
	buffer[length] = '\0';
	AvailableLength -= length;
	return AvailableLength;
}

int CAN::ReadLine(char* buffer){
	pBuffer = &Buffer[BufferCount - AvailableLength];
	int count = 0;
	char ch;
	printf("LineFeedCount:%d\r\n", LineFeedCount);
	if(LineFeedCount > 0){
		do{
			if(pBuffer >= Buffer + 2047){
				pBuffer = Buffer;
			}
			ch = *(pBuffer++);
			if(ch != '\n'){
				buffer[count] = ch;
			}
			count++;
		}while(ch != '\n');
		buffer[count] = '\0';
		AvailableLength -= count;
		LineFeedCount--;
	}
	printf("count:%d\r\n", count);
	return AvailableLength;
}

uint32_t CAN::ReadPackage(uint8_t* data){
	int currentCount = AvailablePackageCount - AvailablePackage;
	Read((char*)data, Length[currentCount]);
	AvailablePackage--;
	return ID[currentCount];
}

void CAN::Print(const char* pstr, ...)
{
	int length = 0;
	va_list arglist;
	char* fp;
	char buffer[64];
	for(int i = 0; i < 64; i++){
		buffer[i] = 0;
	}
	va_start(arglist, pstr);
	vsprintf(buffer, pstr, arglist);
	va_end(arglist);
	fp = buffer;
	while(*(fp++)){
		length++;
	}
	fp = buffer;
	pTxBuffer = &txBuffer[txBufferCount];
	for(int i = 0; i < length; i++){
		if(pTxBuffer >= txBuffer + 2047){
			pTxBuffer = txBuffer;
			txBufferCount = 0;
		}
		*(pTxBuffer++) = buffer[i];
		txBufferCount++;
		SendLength++;
	}
}

void CAN1_RX0_IRQHandler(){
	if(App::mApp->mCAN1 != 0){
		uint8_t data[App::mApp->mCAN1->RxLength];
		if(App::mApp->mCAN1->Receive(data)){
			for(int i = 0; i < App::mApp->mCAN1->RxLength; i++){
				if(App::mApp->mCAN1->BufferCount >= 2047){
					App::mApp->mCAN1->BufferCount = 0;
				}
				App::mApp->mCAN1->Buffer[App::mApp->mCAN1->BufferCount++] = data[i];
				if(data[i] == '\n'){
					App::mApp->mCAN1->LineFeedCount++;
				}
			}
			App::mApp->mCAN1->AvailableLength += App::mApp->mCAN1->RxLength;
		}
	}
}

void CAN::SendPackage(uint32_t id, uint8_t* data, int length){
	TxID = id;
	App::mApp->mCAN1->TxLength = length;
	Transmit(data);
}
