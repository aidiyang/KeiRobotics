/*
 * Config.cpp
 *
 *  Created on: 2015¦~11¤ë27¤é
 *      Author: wongy
 */

#include <Config.h>

Config::Config(){

	CAN1Conf1 = new CANConfiguration(CANConfiguration::CANConf1, CANConfiguration::CANBAUDRATE500K, new Configuration(GPIOD, GPIO_Pin_1), new Configuration(GPIOD, GPIO_Pin_0));

	GPIO1Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOD, GPIO_Pin_12), Bit_SET);
	GPIO2Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOD, GPIO_Pin_13), Bit_SET);
	GPIO3Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOD, GPIO_Pin_14), Bit_SET);
	GPIO4Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOD, GPIO_Pin_15), Bit_SET);
//	GPIO5Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOE, GPIO_Pin_4), Bit_SET);
//	GPIO6Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOE, GPIO_Pin_5), Bit_SET);
//	GPIO7Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOE, GPIO_Pin_6), Bit_SET);
//	GPIO8Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOE, GPIO_Pin_7), Bit_SET);


	UART3Conf1 = new UART::UARTConfiguration(UART::UARTConfiguration::UARTConf3, 115200, new Configuration(GPIOB, GPIO_Pin_10), new Configuration(GPIOB, GPIO_Pin_11), true);
	UART4Conf1 = new UART::UARTConfiguration(UART::UARTConfiguration::UARTConf4, 115200, new Configuration(GPIOC, GPIO_Pin_10), new Configuration(GPIOC, GPIO_Pin_11));

	mInputCaptureConf1 = new InputCaptureConfiguration(new Configuration(GPIOE, GPIO_Pin_9),
													   new Configuration(GPIOE, GPIO_Pin_11),
													   new Configuration(GPIOE, GPIO_Pin_13),
													   new Configuration(GPIOE, GPIO_Pin_14),
													   InputCaptureConfiguration::TimerConf1,
													   42000);

	mPWMConf1 = new PWM::PWMConfiguration(new Configuration(GPIOA, GPIO_Pin_2),
										  new Configuration(GPIOA, GPIO_Pin_3),
										  new Configuration(GPIOB, GPIO_Pin_14),
										  new Configuration(GPIOB, GPIO_Pin_15),
										  10000);
//
//	ADCConf1 = new ADConverter::ADCConfiguration(new Configuration(RCC_AHB1Periph_GPIOA, GPIOA, GPIO_Pin_3), ADC_Channel_3, ADC_SampleTime_480Cycles);
//
//
	Configuration** CS = new Configuration*[6];
//	CS[0] = new Configuration(GPIOE, GPIO_Pin_0);
//	CS[1] = new Configuration(GPIOE, GPIO_Pin_1);
//	CS[2] = new Configuration(GPIOE, GPIO_Pin_2);
//	CS[3] = new Configuration(GPIOE, GPIO_Pin_3);
//	CS[4] = new Configuration(GPIOE, GPIO_Pin_4);
//	CS[5] = new Configuration(GPIOE, GPIO_Pin_5);

	CS[0] = new Configuration(GPIOE, GPIO_Pin_2);
	CS[1] = new Configuration(GPIOE, GPIO_Pin_5);
	CS[2] = new Configuration(GPIOE, GPIO_Pin_2);
	CS[3] = new Configuration(GPIOE, GPIO_Pin_5);
	CS[4] = new Configuration(GPIOE, GPIO_Pin_2);
	CS[5] = new Configuration(GPIOE, GPIO_Pin_5);
	Spi1Conf1 = new Spi::SpiConfiguration(Spi::SpiConfiguration::SpiConf1, Spi::SpiConfiguration::PRESCALER256, Spi::SpiConfiguration::SPIMODE0,
										  new Configuration(GPIOA, GPIO_Pin_5),
										  new Configuration(GPIOA, GPIO_Pin_6),
										  new Configuration(GPIOA, GPIO_Pin_7), CS, false, 6);

	Configuration** slaveCS = new Configuration*[0];
	slaveCS[0] = new Configuration(GPIOB, GPIO_Pin_12);
	Spi2Conf1 = new Spi::SpiConfiguration(Spi::SpiConfiguration::SpiConf2, Spi::SpiConfiguration::PRESCALER2, Spi::SpiConfiguration::SPIMODE0,
											  new Configuration(GPIOB, GPIO_Pin_13),
											  new Configuration(GPIOC, GPIO_Pin_2),
											  new Configuration(GPIOC, GPIO_Pin_3), slaveCS, true, 1);
//
////	I2C1Conf1 = new I2C::I2CConfiguration(I2C1, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_6), GPIO_PinSource6, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_7), GPIO_PinSource7, I2C::I2CConfiguration::SPEED_400K);
	I2C1Conf2 = new I2C::I2CConfiguration(I2C1, new Configuration(GPIOB, GPIO_Pin_8), new Configuration(GPIOB, GPIO_Pin_9), I2C::I2CConfiguration::SPEED_400K);
////	I2C2Conf1 = new I2C::I2CConfiguration(I2C2, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_10), GPIO_PinSource10, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_11), GPIO_PinSource11, I2C::I2CConfiguration::SPEED_400K);
////	I2C2Conf2 = new I2C::I2CConfiguration(I2C2, new Configuration(RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_1), GPIO_PinSource1, new Configuration(RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_0), GPIO_PinSource0, I2C::I2CConfiguration::SPEED_400K);


//	SonicConf1 = new Sonic::SonicConfiguration(GPIO5Conf1, new Configuration(GPIOE, GPIO_Pin_8));
//	SonicConf2 = new Sonic::SonicConfiguration(GPIO6Conf1, new Configuration(GPIOE, GPIO_Pin_9));
//	SonicConf3 = new Sonic::SonicConfiguration(GPIO7Conf1, new Configuration(GPIOE, GPIO_Pin_10));
//	SonicConf4 = new Sonic::SonicConfiguration(GPIO8Conf1, new Configuration(GPIOE, GPIO_Pin_11));
}
