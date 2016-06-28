/*
 * CAN.h
 *
 *  Created on: 2016¦~6¤ë27¤é
 *      Author: wongy
 */

#ifndef CAN_H_
#define CAN_H_

#include <Configuration.h>
#include <stm32f4xx_can.h>

using namespace System;

namespace Communication{

	class CANConfiguration{
		public:
			enum CANConfx{CANConf1,CANConf2};
			enum CANBAUDRATE{CANBAUDRATE2M,CANBAUDRATE1M,CANBAUDRATE500K,CANBAUDRATE400K,CANBAUDRATE250K,CANBAUDRATE200K,CANBAUDRATE125K,CANBAUDRATE100K,CANBAUDRATE50K,CANBAUDRATE40K,CANBAUDRATE10K};
			CANConfiguration(CANConfx CANx, CANBAUDRATE baudrate, Configuration* tx, Configuration* rx);
			CANConfx _CANx;
			CANBAUDRATE _baudrate;
			Configuration* _tx;
			Configuration* _rx;
	};

	class CAN{
		public:
			CAN(CANConfiguration* conf);
			int Transmit(uint8_t id, uint8_t* data, int length);
			int Receive(uint8_t* data);
			void SendPoll();
			void ReceivePoll();
			void Print(int id, const char* pstr, ...);
			int Read(char*, int);
			void Print(const char*, ...);
			CANConfiguration* Conf;
			CAN_TypeDef* CANx;
			CanTxMsg TxMsg;
			CanRxMsg RxMsg;
			char Buffer[2048];
			char txBuffer[64];
			char* pBuffer;
			int BufferCount;
			int AvailableLength;
	};
};

#endif /* CAN_H_ */
