/*
 * Config.cpp
 *
 *  Created on: 2015¦~11¤ë27¤é
 *      Author: wongy
 */

#include <Config.h>

Config::Config(){

//	CAN1Conf1 = new CANConfiguration(CANConfiguration::CANConf1, CANConfiguration::CANBAUDRATE500K, new Configuration(GPIOD, GPIO_Pin_1), new Configuration(GPIOD, GPIO_Pin_0));

	Encoder1Conf1 = new Encoder::EncoderConfiguration(new Configuration(GPIOA, GPIO_Pin_8),
														  new Configuration(GPIOA, GPIO_Pin_9), Encoder::EncoderConfiguration::TimerConf1, 42000000);

	Encoder2Conf1 = new Encoder::EncoderConfiguration(new Configuration(GPIOC, GPIO_Pin_6),
													  new Configuration(GPIOC, GPIO_Pin_7), Encoder::EncoderConfiguration::TimerConf2, 42000000);

	Encoder3Conf1 = new Encoder::EncoderConfiguration(new Configuration(GPIOA, GPIO_Pin_15),
														  new Configuration(GPIOB, GPIO_Pin_3), Encoder::EncoderConfiguration::TimerConf3, 5250000);

	Encoder4Conf1 = new Encoder::EncoderConfiguration(new Configuration(GPIOB, GPIO_Pin_4),
														  new Configuration(GPIOB, GPIO_Pin_5), Encoder::EncoderConfiguration::TimerConf4, 5250000);

	Encoder5Conf1 = new Encoder::EncoderConfiguration(new Configuration(GPIOB, GPIO_Pin_6),
														  new Configuration(GPIOB, GPIO_Pin_7), Encoder::EncoderConfiguration::TimerConf5, 5250000);

	Encoder6Conf1 = new Encoder::EncoderConfiguration(new Configuration(GPIOA, GPIO_Pin_0),
															  new Configuration(GPIOA, GPIO_Pin_1), Encoder::EncoderConfiguration::TimerConf6, 5250000);

	LedConf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOA, GPIO_Pin_10), true, Bit_RESET);
	LedConf2 = new GPIO::GPIOConfiguration(new Configuration(GPIOD, GPIO_Pin_14), true, Bit_RESET);

	GPIO1Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOC, GPIO_Pin_4), true, Bit_SET);
	GPIO2Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOC, GPIO_Pin_5), true, Bit_SET);
	GPIO3Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOC, GPIO_Pin_8), true, Bit_SET);
	GPIO4Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOC, GPIO_Pin_9), true, Bit_SET);

//	GPIO1Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOA, GPIO_Pin_6), true, Bit_RESET);
//	GPIO2Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOA, GPIO_Pin_7), true, Bit_RESET);
//	GPIO3Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOB, GPIO_Pin_6), true, Bit_SET);
//	GPIO4Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOB, GPIO_Pin_8), false, Bit_SET);
//	GPIO5Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOE, GPIO_Pin_4), Bit_SET);
//	GPIO6Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOE, GPIO_Pin_5), Bit_SET);
//	GPIO7Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOE, GPIO_Pin_6), Bit_SET);
//	GPIO8Conf1 = new GPIO::GPIOConfiguration(new Configuration(GPIOE, GPIO_Pin_7), Bit_SET);

//	UART1Conf1 = new UART::UARTConfiguration(UART::UARTConfiguration::UARTConf1, 115200, new Configuration(GPIOA, GPIO_Pin_9), new Configuration(GPIOA, GPIO_Pin_10), true);
//	UART1Conf1 = new UART::UARTConfiguration(UART::UARTConfiguration::UARTConf1, 38400, new Configuration(GPIOA, GPIO_Pin_9), new Configuration(GPIOA, GPIO_Pin_10), false);
//	UART3Conf1 = new UART::UARTConfiguration(UART::UARTConfiguration::UARTConf3, 115200, new Configuration(GPIOB, GPIO_Pin_10), new Configuration(GPIOB, GPIO_Pin_11), false);
//	UART3Conf2 = new UART::UARTConfiguration(UART::UARTConfiguration::UARTConf3, 115200, new Configuration(GPIOD, GPIO_Pin_8), new Configuration(GPIOD, GPIO_Pin_9), true);
	UART4Conf1 = new UART::UARTConfiguration(UART::UARTConfiguration::UARTConf4, 115200, new Configuration(GPIOC, GPIO_Pin_10), new Configuration(GPIOC, GPIO_Pin_11));

//	mInputCaptureConf1 = new InputCaptureConfiguration(new Configuration(GPIOE, GPIO_Pin_9),
//													   new Configuration(GPIOE, GPIO_Pin_11),
//													   new Configuration(GPIOE, GPIO_Pin_13),
//													   new Configuration(GPIOE, GPIO_Pin_14),
//													   InputCaptureConfiguration::TimerConf1,
//													   42000);

	mPWMConf1 = new PWM::PWMConfiguration(new Configuration(GPIOA, GPIO_Pin_2),
										  new Configuration(GPIOA, GPIO_Pin_3),
										  new Configuration(GPIOB, GPIO_Pin_14),
										  new Configuration(GPIOB, GPIO_Pin_15),
										  10000, false);

//	ADCConf1 = new ADCConfiguration(new Configuration(GPIOA, GPIO_Pin_0), ADC_Channel_0, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf2 = new ADCConfiguration(new Configuration(GPIOA, GPIO_Pin_1), ADC_Channel_1, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf3 = new ADCConfiguration(new Configuration(GPIOA, GPIO_Pin_2), ADC_Channel_2, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf4 = new ADCConfiguration(new Configuration(GPIOA, GPIO_Pin_3), ADC_Channel_3, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf5 = new ADCConfiguration(new Configuration(GPIOA, GPIO_Pin_4), ADC_Channel_4, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf6 = new ADCConfiguration(new Configuration(GPIOA, GPIO_Pin_5), ADC_Channel_5, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf7 = new ADCConfiguration(new Configuration(GPIOA, GPIO_Pin_6), ADC_Channel_6, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf8 = new ADCConfiguration(new Configuration(GPIOA, GPIO_Pin_7), ADC_Channel_7, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf9 = new ADCConfiguration(new Configuration(GPIOB, GPIO_Pin_0), ADC_Channel_8, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf10 = new ADCConfiguration(new Configuration(GPIOB, GPIO_Pin_1), ADC_Channel_9, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf11 = new ADCConfiguration(new Configuration(GPIOC, GPIO_Pin_0), ADC_Channel_10, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf12 = new ADCConfiguration(new Configuration(GPIOC, GPIO_Pin_1), ADC_Channel_11, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf13 = new ADCConfiguration(new Configuration(GPIOC, GPIO_Pin_2), ADC_Channel_12, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf14 = new ADCConfiguration(new Configuration(GPIOC, GPIO_Pin_3), ADC_Channel_13, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf15 = new ADCConfiguration(new Configuration(GPIOC, GPIO_Pin_4), ADC_Channel_14, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);
//	ADCConf16 = new ADCConfiguration(new Configuration(GPIOC, GPIO_Pin_5), ADC_Channel_15, ADC_SampleTime_480Cycles, ADCConfiguration::OneShot);

//	Configuration** CS = new Configuration*[6];
//	CS[0] = new Configuration(GPIOB, GPIO_Pin_7);
//	CS[1] = new Configuration(GPIOE, GPIO_Pin_1);
//	CS[2] = new Configuration(GPIOE, GPIO_Pin_2);
//	CS[3] = new Configuration(GPIOE, GPIO_Pin_3);
//	CS[4] = new Configuration(GPIOE, GPIO_Pin_4);
//	CS[5] = new Configuration(GPIOE, GPIO_Pin_5);

//	Spi1Conf1 = new Spi::SpiConfiguration(Spi::SpiConfiguration::SpiConf1, Spi::SpiConfiguration::PRESCALER8, Spi::SpiConfiguration::SPIMODE0,
////										  new Configuration(GPIOA, GPIO_Pin_5),
////										  new Configuration(GPIOA, GPIO_Pin_6),
////										  new Configuration(GPIOA, GPIO_Pin_7), CS, false, 6);
//										  new Configuration(GPIOB, GPIO_Pin_3),
//										  new Configuration(GPIOB, GPIO_Pin_4),
//										  new Configuration(GPIOB, GPIO_Pin_5), CS, false, 1);
//	Configuration** slaveCS = new Configuration*[0];
//	slaveCS[0] = new Configuration(GPIOB, GPIO_Pin_12);
//	Spi2Conf1 = new Spi::SpiConfiguration(Spi::SpiConfiguration::SpiConf2, Spi::SpiConfiguration::PRESCALER2, Spi::SpiConfiguration::SPIMODE0,
//											  new Configuration(GPIOB, GPIO_Pin_13),
//											  new Configuration(GPIOC, GPIO_Pin_2),
//											  new Configuration(GPIOC, GPIO_Pin_3), slaveCS, true, 1);
//
////	I2C1Conf1 = new I2C::I2CConfiguration(I2C1, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_6), GPIO_PinSource6, new Configuration(RCC_AHB1Periph_GPIOB, GPIOB, GPIO_Pin_7), GPIO_PinSource7, I2C::I2CConfiguration::SPEED_400K);
//	I2C1Conf2 = new I2C::I2CConfiguration(I2C1, new Configuration(GPIOB, GPIO_Pin_8), new Configuration(GPIOB, GPIO_Pin_9), I2C::I2CConfiguration::SPEED_400K);
	I2C2Conf1 = new I2C::I2CConfiguration(I2C2, new Configuration(GPIOB, GPIO_Pin_10), new Configuration(GPIOB, GPIO_Pin_11), I2C::I2CConfiguration::SPEED_400K);
////	I2C2Conf2 = new I2C::I2CConfiguration(I2C2, new Configuration(RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_1), GPIO_PinSource1, new Configuration(RCC_AHB1Periph_GPIOF, GPIOF, GPIO_Pin_0), GPIO_PinSource0, I2C::I2CConfiguration::SPEED_400K);


//	SonicConf1 = new Sonic::SonicConfiguration(GPIO5Conf1, new Configuration(GPIOE, GPIO_Pin_8));
//	SonicConf2 = new Sonic::SonicConfiguration(GPIO6Conf1, new Configuration(GPIOE, GPIO_Pin_9));
//	SonicConf3 = new Sonic::SonicConfiguration(GPIO7Conf1, new Configuration(GPIOE, GPIO_Pin_10));
//	SonicConf4 = new Sonic::SonicConfiguration(GPIO8Conf1, new Configuration(GPIOE, GPIO_Pin_11));
}
