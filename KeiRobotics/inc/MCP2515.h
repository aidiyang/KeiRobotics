/*
 * MCP2515.h
 *
 *  Created on: 2016¦~7¤ë28¤é
 *      Author: wongy
 */

#ifndef MCP2515_H_
#define MCP2515_H_

#include <Spi.h>

namespace Communication{

	class MCP2515{
		public:
			enum REG{
				RESET = 0xc0,
				READ = 0x03,
				READTXBUFFER = 0x90,
				WRITE = 0x02,
				LOADTXBUFFER = 0x40,
				RTS = 0x80,
				READSTATUS = 0xa0,
				RXSTATUS = 0xb0,
				BITMOD = 0x05,
				CANCTRL = 0x0f,
				CANSTAT = 0x0e
			};
			MCP2515(Spi* mSpi);
			Spi* _mSpi;
	};

};

#endif /* MCP2515_H_ */
