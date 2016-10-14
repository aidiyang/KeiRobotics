/*
 * nRF24L01.cpp
 *
 *  Created on: 2016¦~10¤ë12¤é
 *      Author: wongy
 */

#include <nRF24L01.h>
#include <stdio.h>
#include <stdarg.h>
#include <App.h>

using namespace Communication;

nRF24L01Configuration::nRF24L01Configuration(Spi* mSpi,
		int SpiIndex,
		GPIO* mCE,
		GPIO* mIRQ,
		int TxLength,
		int RxLength,
		uint8_t* TxAddr,
		uint8_t* RxAddr, uint8_t txChannel, uint8_t rxChannel) : _mSpi(mSpi),
						   _SpiIndex(SpiIndex),
						   _mCE(mCE),
						   _mIRQ(mIRQ),
						   _TxLength(TxLength),
						   _RxLength(RxLength),
						   _TxAddr(TxAddr),
						   _RxAddr(RxAddr),
						   _txChannel(txChannel),
						   _rxChannel(rxChannel){

}

nRF24L01::nRF24L01(nRF24L01Configuration* conf) : Conf(conf),
		SpiIndex(conf->_SpiIndex),
		mSpi(conf->_mSpi),
		mCE(conf->_mCE),
		mIRQ(conf->_mIRQ),
		TxLength(conf->_TxLength),
		RxLength(conf->_RxLength),
		TxAddr(conf->_TxAddr),
		RxAddr(conf->_RxAddr),
		TxChannel(conf->_txChannel),
		RxChannel(conf->_rxChannel),
		AvailableLength(0),
		BufferCount(0),
		pBuffer(Buffer){
	mCE->GPIOControl(false);
	RxMode(RxLength);
	ClearIRQFlags();
}

uint8_t nRF24L01::ReadWrite(uint8_t data) {
	uint8_t value = 0;
	if(!mSpi->Byte(data, &value)){
		return 255;
	}
	return value;
}

void nRF24L01::WriteCmd(uint8_t cmd) {
	uint8_t value = 0;
	mSpi->ChipSelect(SpiIndex);
	if(!mSpi->Byte(cmd, &value)){
		return;
	}
	mSpi->ChipDeSelect(SpiIndex);
}

uint8_t nRF24L01::RWReg(uint8_t reg, uint8_t value){
	uint8_t status;
	mSpi->ChipSelect(SpiIndex);
	status = ReadWrite(reg); // Select register
	ReadWrite(value); // Write value to register
	mSpi->ChipDeSelect(SpiIndex);
	return status;
}

uint8_t nRF24L01::ReadReg(uint8_t reg){
	uint8_t value;
	mSpi->ChipSelect(SpiIndex);
	ReadWrite(reg);
	value = ReadWrite(0x00);
	mSpi->ChipDeSelect(SpiIndex);
	return value;
}

uint8_t nRF24L01::ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	uint8_t status,i;

	mSpi->ChipSelect(SpiIndex);
	status = ReadWrite(reg);
	for (i = 0; i < count; i++) {
		pBuf[i] = ReadWrite(0);
	}
	mSpi->ChipDeSelect(SpiIndex);

	return status;
}

uint8_t nRF24L01::WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	uint8_t status,i;

	mSpi->ChipSelect(SpiIndex);
	status = ReadWrite(reg);
	for (i = 0; i < count; i++) {
		ReadWrite(*pBuf++);
	}
	mSpi->ChipDeSelect(SpiIndex);

	return status;
}

uint8_t nRF24L01::Check(void) {
	uint8_t txbuf[5] = { 0xA9,0xA9,0xA9,0xA9,0xA9 };
	uint8_t rxbuf[5];
	uint8_t i;

	WriteBuf(nRF24_CMD_WREG | nRF24_REG_TX_ADDR,txbuf,5); // Write fake TX address
    ReadBuf(nRF24_REG_TX_ADDR,rxbuf,5); // Try to read TX_ADDR register
    for (i = 0; i < 5; i++) if (rxbuf[i] != txbuf[i]) return 1;

    return 0;
}

void nRF24L01::RxMode(uint8_t RX_PAYLOAD) {
	mCE->GPIOControl(false);
	WriteBuf(nRF24_CMD_WREG | nRF24_REG_RX_ADDR_P0, RxAddr, RxLength); // Set static RX address
	RWReg(nRF24_CMD_WREG | nRF24_REG_EN_AA,0x01); // Enable ShockBurst for data pipe 0
	RWReg(nRF24_CMD_WREG | nRF24_REG_EN_RXADDR,0x01); // Enable data pipe 0
	RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,RxChannel); // Set frequency channel 110 (2.510MHz)
	RWReg(nRF24_CMD_WREG | nRF24_REG_RX_PW_P0,RX_PAYLOAD); // Set RX payload length (10 bytes)
	RWReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,0x07); // Setup: 1Mbps, 0dBm, LNA off
	RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x0F); // Config: CRC on (2 bytes), Power UP, RX/TX ctl = PRX
	mCE->GPIOControl(true);
	Delay::DelayUS(10);
}

void nRF24L01::TxMode(void) {
	mCE->GPIOControl(false);
	RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x02); // Config: Power UP
	RWReg(nRF24_CMD_WREG | nRF24_REG_EN_AA,0x01); // Enable ShockBurst for data pipe 0
	RWReg(nRF24_CMD_WREG | nRF24_REG_SETUP_RETR,0x1A); // Auto retransmit: wait 500us, 10 retries
	RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,TxChannel); // Set frequency channel 110 (2.510MHz)
	RWReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,0x06); // Setup: 1Mbps, 0dBm, LNA off
	RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x0E); // Config: CRC on (2 bytes), Power UP, RX/TX ctl = PTX
}

uint8_t nRF24L01::DataReady(void) {
    uint8_t status;

    status = ReadReg(nRF24_REG_STATUS);
    if (status & nRF24_MASK_RX_DR) return 1;

    // Checking RX_DR isn't good enough, there's can be some data in FIFO
    status = ReadReg(nRF24_REG_FIFO_STATUS);

    return (status & nRF24_FIFO_RX_EMPTY) ? 0 : 1;
}

uint8_t nRF24L01::RxPacket(uint8_t* pBuf, uint8_t RX_PAYLOAD) {
	uint8_t status;
	mCE->GPIOControl(false);
    status = ReadReg(nRF24_REG_STATUS); // Read status register
    if (status & nRF24_MASK_RX_DR) {
    	if ((status & 0x0E) == 0) {
    		// pipe 0
    		ReadBuf(nRF24_CMD_R_RX_PAYLOAD,pBuf,RX_PAYLOAD); // read received payload from RX FIFO buffer
    	}
    	WriteCmd(nRF24_CMD_FLUSH_RX); // Flush RX FIFO buffer
		RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
	    //return nRF24_MASK_RX_DR;
	    return status;
    }

    // Some banana happens
    WriteCmd(nRF24_CMD_FLUSH_RX); // Flush RX FIFO buffer
	RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
    return status;
}

uint8_t nRF24L01::TxPacket(uint8_t * pBuf, uint8_t TX_PAYLOAD) {
    uint8_t status;
	mCE->GPIOControl(false);
    WriteBuf(nRF24_CMD_WREG | nRF24_REG_TX_ADDR,TxAddr,TxLength); // Set static TX address
    WriteBuf(nRF24_CMD_WREG | nRF24_REG_RX_ADDR_P0,RxAddr,RxLength); // Set static RX address for auto ack
    RWReg(nRF24_CMD_WREG | nRF24_REG_EN_AA,0x01); // Enable auto acknowledgement for data pipe 0
    RWReg(nRF24_CMD_WREG | nRF24_REG_SETUP_RETR,0x10); // Automatic retransmission: wait 500us, 10 retries
	RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,TxChannel); // Set frequency channel 110 (2.510MHz)
    RWReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,0x07); // Setup: 1Mbps, 0dBm, LNA on
    WriteBuf(nRF24_CMD_W_TX_PAYLOAD,pBuf,TX_PAYLOAD); // Write specified buffer to FIFO
    RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x0E); // Config: CRC on (2 bytes), Power UP, RX/TX ctl = PTX
	mCE->GPIOControl(true); // CE pin high => Start transmit
    Delay::DelayUS(10);
    App::mApp->mTicks->setTimeout(3);
    while(mIRQ->GPIORead()){
    	if(App::mApp->mTicks->Timeout()){
    		return 0;
    	}
    } // Wait for IRQ from nRF24L01
	mCE->GPIOControl(false);
    status = ReadReg(nRF24_REG_STATUS); // Read status register
	RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
    if (status & nRF24_MASK_MAX_RT) {
        // Auto retransmit counter exceeds the programmed maximum limit. FIFO is not removed.
        RWReg(nRF24_CMD_FLUSH_TX,0xFF); // Flush TX FIFO buffer
        return nRF24_MASK_MAX_RT;
    };
    if (status & nRF24_MASK_TX_DS) {
        // Transmit ok
        RWReg(nRF24_CMD_FLUSH_TX,0xFF); // Flush TX FIFO buffer
        return nRF24_MASK_TX_DS;
    }

    // Some banana happens
    return status;
}

void nRF24L01::ClearIRQFlags(void) {
	uint8_t status;

    status = ReadReg(nRF24_REG_STATUS);
	RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
}

void nRF24L01::ReceivePoll(){
	if(DataReady()){
		RxPacket((uint8_t*)&Buffer[BufferCount], RxLength);
		AvailableLength += RxLength;
		BufferCount += RxLength;
		RxMode(RxLength);
		ClearIRQFlags();
	}
}

int nRF24L01::Read(char* buffer, int length){
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

void nRF24L01::Print(const char* pstr, ...)
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
	char* pTxBuffer = txBuffer;
	for(int i = 0; i < length / TxLength; i++){
		TxPacket((uint8_t*)pTxBuffer, TxLength);
		pTxBuffer += TxLength;
	}
	RxMode(RxLength);
	ClearIRQFlags();
}
