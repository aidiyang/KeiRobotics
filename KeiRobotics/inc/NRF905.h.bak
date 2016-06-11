/*
 * NRF905.h
 *
 *  Created on: 2014¦~11¤ë20¤é
 *      Author: YunKei
 */

#ifndef NRF905_H_
#define NRF905_H_

#include <stm32f4xx.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <Spi.h>

#define MAINBOARD_V3

#ifdef MAINBOARD_V3
#define RF_SPI			SPI1
#define PWR_GPIO		GPIOA
#define PWR_PIN			GPIO_Pin_13
#define TRXCE_GPIO		GPIOA
#define TRXCE_PIN		GPIO_Pin_14
#define TXEN_GPIO		GPIOA
#define TXEN_PIN		GPIO_Pin_2
#define CD_GPIO			GPIOA
#define CD_PIN			GPIO_Pin_11
#define AM_GPIO			GPIOA
#define AM_PIN			GPIO_Pin_12
#define DR_GPIO			GPIOA
#define DR_PIN			GPIO_Pin_8
#define EXTI_PinSource	EXTI_PinSource8
#define EXTI_Line_CH	EXTI_Line8
#define EXTI_IRQ_CH		EXTI9_5_IRQn
#define EXTI_IRQHandler	EXTI9_5_IRQHandler
#else
#define RF_SPI			SPI2
#define PWR_GPIO		GPIOA
#define PWR_PIN			GPIO_Pin_11
#define TRXCE_GPIO		GPIOA
#define TRXCE_PIN		GPIO_Pin_8
#define TXEN_GPIO		GPIOA
#define TXEN_PIN		GPIO_Pin_12
#define CD_GPIO			GPIOA
#define CD_PIN			GPIO_Pin_14
#define AM_GPIO			GPIOA
#define AM_PIN			GPIO_Pin_13
#define DR_GPIO			GPIOA
#define DR_PIN			GPIO_Pin_2
#define EXTI_PinSource	EXTI_PinSource2
#define EXTI_Line_CH	EXTI_Line2
#define EXTI_IRQ_CH		EXTI2_IRQn
#define EXTI_IRQHandler	EXTI2_IRQHandler
#endif

namespace Communication{

	class NRF905{

		public:

			enum FREQ{FREQ_433M = (0 << 1),
						FREQ_868M = (1 << 1)};
			enum POWER{PWR_NEG_10dBM = (0 << 2),
						PWR_NEG_2dBM = (1 << 2),
						PWR_POS_6dBM = (2 << 2),
						PWR_POS_10dBM = (3 << 2)};
			enum RX_POWER{RX_NORMAL_PWR = (0 << 4),
						RX_REDUCE_PWR = (1 << 4)};
			enum RETRAN{NO_RETRAN = (0 << 5),
						AUTO_RETRAN = (1 << 5)};
			enum UP_CLK{UP_CLK_FREQ_4M = (0 << 0),
						UP_CLK_FREQ_2M = (1 << 0),
						UP_CLK_FREQ_1M = (2 << 0),
						UP_CLK_FREQ_500K = (3 << 0),
						UP_CLK_DIS = (0 << 2),
						UP_CLK_EN = (1 << 2)};
			enum XOF{XOF_4M = (0 << 3),
						XOF_8M = (1 << 4),
						XOF_12M = (2 << 3),
						XOF_16M = (3 << 3),
						XOF_20M = (4 << 3)};
			enum CRC_CONFIG{CRC_DIS = (0 << 6),
						CRC_EN = (1 << 6),
						CRC_8 = (0 << 7),
						CRC_16 = (1 << 7)};
			enum CMD{WC = 0x00,
						RX_PW = 0x03,
						TX_PW = 0x04,
						RC = 0x10,
						WTP = 0x20,
						RTP = 0x21,
						WTA = 0x22,
						RTA = 0x23,
						RRP = 0x24};
			enum STATE{NRF905_NORMAL = 0x01,
						NRF905_RX_IRQ = 0x02,
						NRF905_RX = 0x10,
						NRF905_TX = 0x20};

			NRF905(int , FREQ, POWER, RX_POWER, RETRAN, uint8_t*, uint8_t*, int, int, bool createdInstance = false);
			void resetNRF905();
			bool Write(const char*, ...);
			int Read(unsigned char*, int);
			bool getIsTransmiting();
			void setIsTransmiting(bool);
			static NRF905* getInstance();
			char* getBuffer();
			void setBufferCount(int);
			int getBufferCount();
			void setStandby();
			void setTransmit();
			void setReceive();
			int getRXPW();
			void setRXPW(int);
			void setPwr();
			void clrPwr();

			int getChannel();
			FREQ getFreq();
			POWER getPower();
			RX_POWER getRxPower();
			RETRAN getReTran();
			uint8_t* getRxAddress();
			uint8_t* getTxAddress();
			int getTXPW();
			int Count;

		private:
			Spi* NRF905Spi;
			int _channel;
			FREQ _freq;
			POWER _power;
			RX_POWER _rxPower;
			RETRAN _retrans;
			uint8_t* _rxAddress;
			uint8_t* _txAddress;
			int NRF905_RXPW;
			int NRF905_TXPW;
			bool isTransmitting;
			char Buffer[2048];
			char * pBuffer;
			int BufferCount;
			int ErrorCount;
			void setTrxce();
			void clrTrxce();
			void setTxen();
			void clrTxen();
			uint8_t readDr();
	};
};

#endif /* NRF905_H_ */
