/*
 * UART.h
 *
 *  Created on: 2014¦~8¤ë4¤é
 *      Author: YunKei
 */

#ifndef UART_H_
#define UART_H_

#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <Configuration.h>

using namespace System;

namespace Communication{

	class UART{

		public:
			class UARTConfiguration{
				public:
					enum UARTConfx{UARTConf1,UARTConf2,UARTConf3,UARTConf4,UARTConf5};
					UARTConfiguration(UARTConfx UARTx, uint32_t baudrate, Configuration* tx, Configuration* rx, bool UseDMA = false);
					UARTConfx _UARTx;
					uint32_t _baudrate;
					Configuration* _tx;
					Configuration* _rx;
					bool _UseDMA;
				private:
			};

			UART(UARTConfiguration* conf);
			int Read(char*, int);
//			void setBufferCount(int);
//			int getBufferCount();
//			char* getBuffer();
			void setPrintUART();
			void Print(const char*, ...);
//			bool getIsDmaBusy();
//			void setIsDmaBusy(bool);
//			char* getRxBuffer();
//			void reset();
//			uint32_t getBaudrate();

			UARTConfiguration* Conf;
			char Buffer[2048];
			char txBuffer[2048];
			char rxBuffer[7];
			bool isDmaBusy;
			char* pBuffer;
			int BufferCount;
			int AvailableLength;
			int UARTConf;
		private:
			USART_TypeDef* getUARTx();
	};
};

#endif /* UART_H_ */
