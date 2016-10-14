/*
 * nRF24L01.h
 *
 *  Created on: 2016¦~10¤ë12¤é
 *      Author: wongy
 */

#ifndef NRF24L01_H_
#define NRF24L01_H_

#include <Spi.h>
#include <GPIO.h>
#include <ExternalInterrupt.h>
#include <Delay.h>

namespace Communication{
	class nRF24L01Configuration{
		public:
			nRF24L01Configuration(Spi* mSpi, int SpiIndex, GPIO* mCE, GPIO* mIRQ, int TxLength, int RxLength, uint8_t* TxAddr, uint8_t* RxAddr, uint8_t txChannel, uint8_t rxChannel);
			Spi* _mSpi;
			int _SpiIndex;
			GPIO* _mCE;
			GPIO* _mIRQ;
			int _TxLength;
			int _RxLength;
			uint8_t* _TxAddr;
			uint8_t* _RxAddr;
			uint8_t _txChannel;
			uint8_t _rxChannel;
	};
	class nRF24L01{
		public:
			enum CMD{nRF24_CMD_RREG = 0x00,  // R_REGISTER -> Read command and status registers
					 nRF24_CMD_WREG = 0x20,  // W_REGISTER -> Write command and status registers
					 nRF24_CMD_R_RX_PAYLOAD = 0x61,  // R_RX_PAYLOAD -> Read RX payload
					 nRF24_CMD_W_TX_PAYLOAD = 0xA0,  // W_TX_PAYLOAD -> Write TX payload
					 nRF24_CMD_FLUSH_TX = 0xE1,  // FLUSH_TX -> Flush TX FIFO
					 nRF24_CMD_FLUSH_RX = 0xE2,  // FLUSH_RX -> Flush RX FIFO
					 nRF24_CMD_REUSE_TX_PL = 0xE3,  // REUSE_TX_PL -> Reuse last transmitted payload
					 nRF24_CMD_NOP = 0xFF  // No operation (to read status register)
				 };
			/* nRF24L0 registers */
			enum REG{nRF24_REG_CONFIG = 0x00,  // Configuration register
					 nRF24_REG_EN_AA = 0x01,  // Enable "Auto acknowledgment"
					 nRF24_REG_EN_RXADDR = 0x02,  // Enable RX addresses
					 nRF24_REG_SETUP_AW = 0x03,  // Setup of address widths
					 nRF24_REG_SETUP_RETR = 0x04,  // Setup of automatic retranslation
					 nRF24_REG_RF_CH = 0x05,  // RF channel
					 nRF24_REG_RF_SETUP = 0x06,  // RF setup register
					 nRF24_REG_STATUS = 0x07,  // Status register
					 nRF24_REG_OBSERVE_TX = 0x08,  // Transmit observe register
					 nRF24_REG_CD = 0x09,  // Carrier detect
					 nRF24_REG_RX_ADDR_P0 = 0x0A,  // Receive address data pipe 0
					 nRF24_REG_RX_ADDR_P1 = 0x0B,  // Receive address data pipe 1
					 nRF24_REG_RX_ADDR_P2 = 0x0C, // Receive address data pipe 2
					 nRF24_REG_RX_ADDR_P3 = 0x0D,  // Receive address data pipe 3
					 nRF24_REG_RX_ADDR_P4 = 0x0E,  // Receive address data pipe 4
					 nRF24_REG_RX_ADDR_P5 = 0x0F,  // Receive address data pipe 5
					 nRF24_REG_TX_ADDR = 0x10,  // Transmit address
					 nRF24_REG_RX_PW_P0 = 0x11,  // Number of bytes in RX payload id data pipe 0
					 nRF24_REG_RX_PW_P1 = 0x12,  // Number of bytes in RX payload id data pipe 1
					 nRF24_REG_RX_PW_P2 = 0x13,  // Number of bytes in RX payload id data pipe 2
					 nRF24_REG_RX_PW_P3 = 0x14,  // Number of bytes in RX payload id data pipe 3
					 nRF24_REG_RX_PW_P4 = 0x15,  // Number of bytes in RX payload id data pipe 4
					 nRF24_REG_RX_PW_P5 = 0x16,  // Number of bytes in RX payload id data pipe 5
					 nRF24_REG_FIFO_STATUS = 0x17,  // FIFO status register
					 nRF24_REG_DYNPD = 0x1C,  // Enable dynamic payload length
					 nRF24_REG_FEATURE = 0x1D  // Feature register
				 };
			/* nRF24L0 bits */
			enum Bits{nRF24_MASK_RX_DR = 0x40,  // Mask interrupt caused by RX_DR
					  nRF24_MASK_TX_DS = 0x20,  // Mask interrupt caused by TX_DS
					  nRF24_MASK_MAX_RT = 0x10,  // Mask interrupt caused by MAX_RT
					  nRF24_FIFO_RX_EMPTY = 0x01,  // RX FIFO empty flag
					  nRF24_FIFO_RX_FULL = 0x02  // RX FIFO full flag
					};

			nRF24L01(nRF24L01Configuration* conf);

			uint8_t ReadWrite(uint8_t data);
			uint8_t RWReg(uint8_t reg, uint8_t value);
			uint8_t ReadReg(uint8_t reg);
			void WriteCmd(uint8_t cmd);
			uint8_t ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t count);
			uint8_t WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t count);
			uint8_t Check(void);

			void RxMode(uint8_t RX_PAYLOAD);
			void TxMode(void);
			uint8_t DataReady(void);
			uint8_t TxPacket(uint8_t * pBuf, uint8_t TX_PAYLOAD);
			uint8_t RxPacket(uint8_t* pBuf, uint8_t RX_PAYLOAD);
			void ClearIRQFlags(void);
			void ReceivePoll();
			int Read(char* buffer, int length);
			void Print(const char* pstr, ...);
			int TxLength;
			int RxLength;
			uint8_t* RxAddr;
			uint8_t* TxAddr;
			Spi* mSpi;
			int SpiIndex;
			GPIO* mCE;
			GPIO* mIRQ;
			nRF24L01Configuration* Conf;
			uint8_t TxChannel;
			uint8_t RxChannel;
			char Buffer[2048];
			int BufferCount;
			char* pBuffer;
			int AvailableLength;
			char txBuffer[2048];

		private:
	};
};
#endif /* NRF24L01_H_ */
