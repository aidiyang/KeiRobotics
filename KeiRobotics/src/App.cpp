/*
 * App.cpp
 *
 *  Created on: 2015¦~11¤ë27¤é
 *      Author: wongy
 */

#include <App.h>
#include <stdio.h>
#include <stdlib.h>
using namespace std;
using namespace System;
using namespace Sensors;
using namespace Utility;

App* App::mApp = 0;
Ticks* App::mTicks = 0;
Task* App::mTask = 0;
UART* App::mUART4 = 0;
Spi* App::mSpi1 = 0;
Spi* App::mSpi2 = 0;
Communicating* App::mCommunicating1 = 0;
Communicating* App::mCommunicating2 = 0;
Communicating* App::mCommunicating3 = 0;
Com* App::Com1 = 0;
Com* App::Com2 = 0;
Com* App::Com3 = 0;

void ControlTask(){
	App::mApp->mControlling->ControllingPoll();
}

void initUpdate(){
	App::mApp->mMPU6050->Update();
	App::mApp->mAcceleration->Update();
	App::mApp->mOmega->Update();
}

void initCompassUpdate(){
	App::mApp->mHMC5883L->Update();
	App::mApp->mCompass->Update();
}

void UpdateTask(){
	App::mApp->mMPU6050->Update();
	App::mApp->mAcceleration->Update();
	App::mApp->mOmega->Update();
	App::mApp->mQuaternion->Update();

}

void MPU6050UpdateTask(){
	App::mApp->mMPU6050->Update();

}

void AccelerationUpdateTask(){
	App::mApp->mAcceleration->Update();

}

void OmegaUpdateTask(){
	App::mApp->mOmega->Update();
}

void QuaternionUpdateTask(){
	App::mApp->mQuaternion->Update();

}

void CompassUpdate(){
	App::mApp->mHMC5883L->Update();
	App::mApp->mCompass->Update();
}

void App::ReceiveTask(Bundle* bundle){
	App::mCommunicating1->ReceivePoll();
	App::mCommunicating2->ReceivePoll();
	App::mCommunicating3->ReceivePoll();
}

void App::SendTask(Bundle* bundle){
	App::mApp->mCommunicating1->SendPoll();
	App::mApp->mCommunicating2->SendPoll();
	App::mApp->mCommunicating3->SendPoll();
}

void print(){
	static int index = 0;
	switch(App::mApp->mCommunicating1->PrintType){
		case 0:
			if(index < 3){
				if(index == 0){
					App::mApp->mCommunicating1->Send(index, (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[index]) - App::mApp->mControlling->RollOffset));
				}
				else if(index == 1){
					App::mApp->mCommunicating1->Send(index, (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[index]) - App::mApp->mControlling->PitchOffset));
				}
				else if(index == 2){
					App::mApp->mCommunicating1->Send(index, (float)(MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[index]) - App::mApp->mControlling->YawOffset));
				}
			}
			break;
		case 1:
			if(index < 3){
				App::mApp->mCommunicating1->Send(index, (float)(App::mApp->mMPU6050->getRawOmega()[index]));
			}
			break;
		case 2:
			if(index == 0){
				App::mApp->mCommunicating1->Send(0, App::mApp->mControlling->Motor1PWM);
			}
			else if(index == 1){
				App::mApp->mCommunicating1->Send(1, App::mApp->mControlling->Motor2PWM);
			}
			else if(index == 2){
				App::mApp->mCommunicating1->Send(2, App::mApp->mControlling->Motor3PWM);
			}
			else if(index == 3){
				App::mApp->mCommunicating1->Send(3, App::mApp->mControlling->Motor4PWM);
			}
			break;
	}
	if(index == 4){
		index = 0;
	}
	else{
		index++;
	}
}

void BatteryPrint(){
	App::mApp->mADCFilter->Update(App::mApp->mADC->getReading() * 3.3 / 4096.0);
	App::mApp->mCommunicating1->Send(0, App::mApp->mADCFilter->getAverage());
//	printf("%g\n", App::mApp->mADC->getReading());
}

void LocalizationUpdate(){
	App::mApp->mLocalization->LocalizationCalc();
}

void CompassCalTask(){
	App::mApp->mHMC5883L->CalibrationPrint();
}

void CompassCalPrintResult(){
	App::mApp->mHMC5883L->CalibrationResultPrint();
}

void printfBufferTask(){
	AdditionalTools::printfBuffer(0, 4);
}

void Task100Hz(){
	UpdateTask();
	ControlTask();
//	ReceiveTask();
//	SendTask();
}

void Task60Hz(){
	UpdateTask();
	ControlTask();
//	ReceiveTask();
//	SendTask();
}

void Task50Hz(){
	print();
}

void App::SPITest(Bundle* bundle){
	mCommunicating2->Send(0, mTicks->getTicks());
	printf("%d\r\n", mTicks->getTicks());
}

void App::Print(Bundle* bundle){
	printf("App::mCommunicating3->BufferCount:%d\r\n", App::mCommunicating3->BufferCount);
}

void App::SPIReceive(Bundle* bundle){
	if(mSpi2->AvailableLength > 0){
		char ch[32];
		mSpi2->Read(ch, mSpi2->AvailableLength);
		printf("%s", ch);
	}
}

void App::TaskDurationPrint(Bundle* bundle){
	mTask->printDeration();
}

App::App() : debugCount(0), arrived(false), PeriodicData(0), PeriodicCmd(0), PeriodicData2(0), PeriodicCmd2(0), trigger(false), Motor1Target(0), Motor2Target(0), Motor3Target(0), mQuaternion(0), mCompass(0), mEncoderYaw(0), PathState(0){
	Delay::DelayMS(10);
	mApp = this;
	for(int i = 0; i < 16; i++){
		mExti[i] = 0;
	}

	mConfig = new Config();

	mTicks = new Ticks(false);
	mTask = new Task();

	mUART4 = new UART(mConfig->UART4Conf1);
	mSpi1 = new Spi(mConfig->Spi1Conf1);
	mSpi2 = new Spi(mConfig->Spi2Conf1);
	Com1 = new Com(Com::__UART, (uint64_t)mUART4);
	Com2 = new Com(Com::__SPI, (uint64_t)mSpi1, 0);
	Com3 = new Com(Com::__SPI, (uint64_t)mSpi2, 0);
	mCommunicating1 = new Communicating(Com1);
	mCommunicating2 = new Communicating(Com2);
	mCommunicating3 = new Communicating(Com3);
	mLed1 = new GPIO(mConfig->GPIO1Conf1);
	mLed2 = new GPIO(mConfig->GPIO2Conf1);
	mLed3 = new GPIO(mConfig->GPIO3Conf1);
	mLed4 = new GPIO(mConfig->GPIO4Conf1);
	printf("Started\r\n");
	mTask->Attach(10, SendTask, "SendTask", true);
	mTask->Attach(10, ReceiveTask, "ReceiveTask", true);
	mLed1->Blink(mLed1, true, 100);
	mLed2->Blink(mLed2, true, 200);
	mLed3->Blink(mLed3, true, 300);
	mLed4->Blink(mLed4, true, 400);

//	mTask->Attach(20, SPITest, "SPITest", true);
//	mTask->Attach(100, Print, "Print", true);
	mTask->Run(true);
//
//	mUART4 = new UART(mConfig->UART4Conf1);
//	mCommunicating1 = new Communicating(new Communicating::Com(Communicating::Com::__UART, (uint32_t)mUART4));
//
//	mPWM = new PWM(mConfig->mPWMConf1);
//	App::mApp->mPWM->Control1(0);
//	App::mApp->mPWM->Control2(0);
//	App::mApp->mPWM->Control3(0);
//	App::mApp->mPWM->Control4(0);
//
//	mControlling = new Controlling(mPWM);
//
//	mI2C1 = new I2C(mConfig->I2C1Conf2);
//	mMPU6050 = new MPU6050(mI2C1);
//	mAcceleration = new Acceleration(mMPU6050);
//	mOmega = new Omega(mMPU6050);
//	mTask->Attach(16, 0, initUpdate, "initUpdate", false, 60, false);
//
//	Delay::DelayMS(10);
//	mTask->Run();
//	mQuaternion = new Quaternion(mAcceleration, mOmega);
//	mQuaternion->Reset();
//	mTask->Attach(10, 0, Task100Hz, "Task100Hz", true);
////	mTask->Attach(16, 0, MPU6050UpdateTask, "MPU6050UpdateTask", true);
////	mTask->Attach(16, 0, AccelerationUpdateTask, "AccelerationUpdateTask", true);
////	mTask->Attach(16, 0, OmegaUpdateTask, "OmegaUpdateTask", true);
////	mTask->Attach(16, 0, QuaternionUpdateTask, "QuaternionUpdateTask", true);
//	mTask->Attach(20, 0, Task50Hz, "Task50Hz", true);
//
//	mLed1->Blink(true, 500, 2);
//	mLed2->Blink(true, 500, 2);
//	mLed3->Blink(true, 500, 2);
//	mLed4->Blink(true, 500, 2);
//	printf("Started\n");
//	mTask->Run(true);
}

void HardFault_Handler(){
	printf("HF:%s:%d\r\n", App::mApp->mTask->mTaskObj[App::mApp->mTask->currentTaskNum]->TaskName.c_str(), App::mApp->debugCount);
	while(true);
}
