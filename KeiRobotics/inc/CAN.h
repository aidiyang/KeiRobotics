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
			void StartReceive();
			int Transmit(uint8_t* data);
			bool Receive(uint8_t* data);
			void SendPoll();
			void ReceivePoll();
			void Print(int id, const char* pstr, ...);
			int Read(char*, int);
			int ReadLine(char* buffer);
			uint32_t ReadPackage(uint8_t* data);
			void SendPackage(uint32_t id, uint8_t* data, int length);
			void Print(const char*, ...);
			CANConfiguration* Conf;
			CAN_TypeDef* CANx;
			CanTxMsg TxMsg;
			CanRxMsg RxMsg;
			char Buffer[2048];
			char txBuffer[2048];
			char* pBuffer;
			char* pTxBuffer;
			int BufferCount;
			int txBufferCount;
			int SendLength;
			int AvailableLength;
			int TxLength;
			int RxLength;
			uint32_t TxID;
			uint32_t RxID;
			int LineFeedCount;
			uint32_t ID[2048];
			uint8_t Length[2048];
			int AvailablePackage;
			int AvailablePackageCount;
	};
};

#endif /* CAN_H_ */
