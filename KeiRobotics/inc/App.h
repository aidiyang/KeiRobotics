/*
 * App.h
 *
 *  Created on: 2015¦~11¤ë27¤é
 *      Author: wongy
 */

#ifndef APP_H_
#define APP_H_

#include <stm32f4xx.h>
#include <inttypes.h>
#include <MathTools.h>
#include <stdio.h>
#include <Delay.h>
#include <Config.h>
#include <Communicating.h>
#include <Ticks.h>
#include <Task.h>
#include <GPIO.h>
#include <UART.h>
#include <PWM.h>
#include <ADConverter.h>
#include <Sonic.h>
#include <Spi.h>
#include <I2C.h>
#include <Kalman.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <Acceleration.h>
#include <Omega.h>
#include <Compass.h>
#include <Quaternion.h>
#include <AdditionalTools.h>
#include <Controlling.h>
#include <MovingWindowAverageFilter.h>
#include <Encoder.h>
#include <Localization.h>
#include <EncoderYaw.h>
#include <ExternalInterrupt.h>
#include <Eigen/Eigen>
#include <CAN.h>

using namespace Time;
using namespace Math;
using namespace Sensors;
using namespace Inertia;
using namespace Control;
using namespace Communication;
using namespace Debug;

namespace System{
	class Config;
	class GPIO;
};

namespace Control{
	class Controlling;
};

namespace Communication{
	class Communicating;
	class Com;
};

namespace Sensors{
	class MPU6050;
	class HMC5883L;
	class Encoder;
};

namespace Inertia{
	class Acceleration;
	class Omega;
	class Compass;
	class EncoderYaw;
};

namespace Math{
	class Quaternion;
	class Localization;
};


typedef struct Point{
	float speed;
	float x;
	float y;
	float yaw;
	bool SonicCalFL;
	bool SonicCalFR;
	bool SonicCalL;
	bool SonicCalR;
	float FL;
	float FR;
	float L;
	float R;
	bool CalX;
	bool CalY;
	float CalXValue;
	float CalYValue;
} PT;

namespace System{
	class App{
		public:
			App();
			static void SPITest(Bundle* bundle);
			static void SPIReceive(Bundle* bundle);
			static void ReceiveTask(Bundle* bundle);
			static void SendTask(Bundle* bundle);
			static void Print(Bundle* bundle);
			static void TaskDurationPrint(Bundle* bundle);
			static App* mApp;
			static Ticks* mTicks;
			static Task* mTask;
			Config* mConfig;
			ExternalInterrupt* mExti[16];
			GPIO* mLed1;
			GPIO* mLed2;
			GPIO* mLed3;
			GPIO* mLed4;

			GPIO* mSonicTrigger[16];

			GPIO* mGPIO1;
			GPIO* mGPIO2;
			GPIO* mGPIO3;
			GPIO* mGPIO4;
			GPIO* mGPIO5;
			GPIO* mGPIO6;
			GPIO* mGPIO7;
			GPIO* mGPIO8;
			UART* mUART1;
			UART* mUART2;
			UART* mUART3;
			static UART* mUART4;
			static CAN* mCAN1;
			UART* mUART5;
			static Com* Com1;
			static Com* Com2;
			static Com* Com3;
			static Communicating* mCommunicating1;
			static Communicating* mCommunicating2;
			static Communicating* mCommunicating3;
			Communicating* mCommunicating4;
			PWM* mPWM;
			ADConverter* mADC;
			Sonic* mSonic1;
			Sonic* mSonic2;
			Sonic* mSonic3;
			Sonic* mSonic4;
			Sonic* mSonic5;
			Sonic* mSonic6;
			Sonic* mSonic7;
			Sonic* mSonic8;
			Sonic* mSonic9;
			Sonic* mSonic10;
			Sonic* mSonic11;
			Sonic* mSonic12;
			Sonic* mSonic13;
			Sonic* mSonic14;
			Sonic* mSonic15;
			Sonic* mSonic16;
			static Spi* mSpi1;
			static Spi* mSpi2;
			I2C* mI2C1;
			I2C* mI2C2;
			MPU6050* mMPU6050;
			HMC5883L* mHMC5883L;
			Acceleration* mAcceleration;
			Omega* mOmega;
			Compass* mCompass;
			Quaternion* mQuaternion;
			Controlling* mControlling;
			MovingWindowAverageFilter* mADCFilter;
			Encoder* mEncoder1;
			Encoder* mEncoder2;
			Encoder* mEncoder3;
			Encoder* mEncoder4;
			Encoder* mEncoder5;
			Encoder* mEncoder6;
			Localization* mLocalization;
			EncoderYaw* mEncoderYaw;
			int PathState;
			int PeriodicCmd;
			float PeriodicData;
			int PeriodicCmd2;
			float PeriodicData2;
			PT currentPT;
			PT nextPT;
			bool trigger;
			bool arrived;

			Pid* Motor1PID;
			Pid* Motor2PID;
			Pid* Motor3PID;
			float Motor1Target;
			float Motor2Target;
			float Motor3Target;
			int IsCal1;
			int IsCal2;
			int IsCal3;
			bool ControlStart;
			int debugCount;
			uint32_t* funcAddr;
			uint32_t funcNum;
			float error;
		private:
	};
};

#endif /* APP_H_ */
