/*
 * Communicating.h
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#ifndef COMMUNICATING_H_
#define COMMUNICATING_H_

#include <App.h>
#include <UART.h>
#include <stm32f4xx.h>
#include <inttypes.h>
#include <stm32f4xx_usart.h>
#include <stdio.h>
#include <PWM.h>
#include <Controlling.h>
#include <Pid.h>
#include <Quaternion.h>
#include <Localization.h>
#include <math.h>
#include <MathTools.h>
#include <Task.h>
#include <inttypes.h>
#include <Delay.h>
#include <Led.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <MPU6050.h>
#include <Spi.h>

namespace Communication{

	class Com{
		public:
			enum Interface{__UART, __SPI, __I2C};
			Com(Interface interface, uint32_t addr, int index = 0);
			Interface _interface;
			UART* _UART;
			Spi* _Spi;
			int Index;
//				I2C* _I2C;
		private:
	};

	class Communicating{

		public:

			enum COM{COM1,COM2,COM3,COM4,COM5,COM6};

			enum CMD{
				WATCHDOG,
				PRINT_MODE,
				HIGH,
				LOW,
				RESET_ALL,
				START,
				STOP,
				POWER,
				ROLL_OFFSET,
				PITCH_OFFSET,
				YAW_OFFSET,
				ROLL_KP,
				ROLL_KI,
				ROLL_KD,
				PITCH_KP,
				PITCH_KI,
				PITCH_KD,
				YAW_KP,
				YAW_KI,
				YAW_KD,
				MAX_LIFT_VALUE,
				MIN_LIFT_VALUE,
				LIFT,
				TARGET_ROLL,
				TARGET_PITCH,
				TARGET_YAW,
				SET_X_TRANSLATION,
				SET_Y_TRANSLATION,
				MOTOR_KP,
				MOTOR_KI,
	            FORWARD,
	            BACKWARD,
	            LEFT,
	            RIGHT,
				PAUSE,
	            CW,
				CCW,
				MOVE,
				MANUAL_MODE,
				RETURN_HOME,
				TEST,
				CLAMPER_STOP_ALL,
				CLAMPER_RESET,
				CLAMPER_START,
				CLAMPER_SET_MOTOR1_TARGET,
				CLAMPER_SET_MOTOR2_TARGET,
				CLAMPER_SET_MOTOR3_TARGET,
				CLAMPER_WATCHDOG,
				SUCCESS,
				CLAMPER_SET_HORIZONTAL,
				SET_SPEED,
				SET_X_POS,
				SET_Y_POS,
				SET_YAW,
				SET_SONIC_CAL_FL,
				SET_SONIC_CAL_FR,
				SET_SONIC_CAL_L,
				SET_SONIC_CAL_R,
				SET_SONIC_CAL_FL_VALUE,
				SET_SONIC_CAL_FR_VALUE,
				SET_SONIC_CAL_L_VALUE,
				SET_SONIC_CAL_R_VALUE,
				SET_X_CAL,
				SET_Y_CAL,
				SET_X_CAL_VALUE,
				SET_Y_CAL_VALUE,
				TRIGGER,
				CLAMPER_STOP_ALL_RUN,
				CLAMPER_RESET_RUN,
				CLAMPER_START_RUN,
				CLAMPER_SET_MOTOR1_TARGET_RUN,
				CLAMPER_SET_MOTOR2_TARGET_RUN,
				CLAMPER_SET_MOTOR3_TARGET_RUN,
				CLAMPER_WATCHDOG_RUN,
				CLAMPER_SET_HORIZONTAL_RUN,
				AUTO_MODE,
				NEXT
			};

			Communicating(Com* com);
			void ReceivePoll();
			void SendPoll();
			void Execute(int, float);
			void Send(int, float);
			void SendCmd(char, float);
			int getTxBufferCount();
			void Acknowledgement();
			int PrintType;
			int Length;
			int count;
			int txBufferCount;
			int BufferCount;
			int PeriodicCmd;
			float PeriodicData;
			bool readyToSend;
			Com* _com;

		private:
			int WatchDog;
			bool isToken;
			char Buffer[2048];
			char txBuffer[2048];
			char Bytes[7];
			float CmdData;
			int Cmd;
			float Data;
			int index;
	};
};

#endif /* COMMUNICATING_H_ */
