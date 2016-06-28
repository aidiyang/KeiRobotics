/*
 * mpu6050.cpp
 *
 *  Created on: 2014¦~8¤ë23¤é
 *      Author: YunKei
 */

#include <MPU6050.h>
#include <stdio.h>

using namespace System;
using namespace Sensors;
using namespace Math;

float MPU6050::RawAccPosX =	10.1f;
float MPU6050::RawAccNegX =	-9.6f;
float MPU6050::RawAccPosY =	9.8f;
float MPU6050::RawAccNegY =	-9.9f;
float MPU6050::RawAccPosZ =	10.0f;
float MPU6050::RawAccNegZ =	-10.0f;

void MPU6050::setI2CBypass(bool onState){
	uint8_t data = 0;
	if(onState){
		data = 0x02;
	}
	App::mApp->mTicks->setTimeout(3);
	while(!i2cx->Write(ADDRESS,RA_INT_PIN_CFG,data)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}
}

MPU6050::MPU6050(Communication::I2C* i2c) : i2cx(i2c), isValided(false){

	RawAccScale[0] = 2.0f * Inertia::Acceleration::Gravity / (RawAccPosX - RawAccNegX);
	RawAccScale[1] = 2.0f * Inertia::Acceleration::Gravity / (RawAccPosY - RawAccNegY);
	RawAccScale[2] = 2.0f * Inertia::Acceleration::Gravity / (RawAccPosZ - RawAccNegZ);
	RawAccOffset[0] = RawAccPosX * RawAccScale[0] - Inertia::Acceleration::Gravity;
	RawAccOffset[1] = RawAccPosY * RawAccScale[1] - Inertia::Acceleration::Gravity;
	RawAccOffset[2] = RawAccPosZ * RawAccScale[2] - Inertia::Acceleration::Gravity;
	FastInitialization();

	RawOmegaScale[0] = 1.0f;
	RawOmegaScale[1] = 1.0f;
	RawOmegaScale[2] = 1.0f;
	RawOmegaOffset[0] = RawOmegaScale[0] * 1.5f;
	RawOmegaOffset[1] = RawOmegaScale[1] * 7.8f;
	RawOmegaOffset[2] = RawOmegaScale[2] * -0.4;
	Update();
}

void MPU6050::FastInitialization(){

	App::mApp->mTicks->setTimeout(3);
	while(!i2cx->Write(ADDRESS,RA_PWR_MGMT_1,0x00)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}

	App::mApp->mTicks->setTimeout(3);
	while(!i2cx->Write(ADDRESS,RA_SMPLRT_DIV,0x07)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}

	App::mApp->mTicks->setTimeout(3);
	while(!i2cx->Write(ADDRESS,RA_CONFIG,0x00)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}
	App::mApp->mTicks->setTimeout(3);
	while(!i2cx->Write(ADDRESS,RA_GYRO_CONFIG,0x18)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}
	App::mApp->mTicks->setTimeout(3);
	while(!i2cx->Write(ADDRESS,RA_ACCEL_CONFIG,0x18)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}
}

bool MPU6050::Update(){

	uint8_t data[14];
	int16_t temp;
	if(!i2cx->BurstRead(ADDRESS, RA_ACCEL_XOUT_H, 14, data)){
		FastInitialization();
		isValided = false;
		return false;
	}

	for(int i = 0; i < 14; i += 2){
		if(i >= 0 && i <= 5){
			int j = i / 2;
			temp = (data[i + 1] | (data[i] << 8));
			RawAcc[j] = (float)temp * 0.0047884033203125f;
		}
		else if(i >= 6 && i <= 7){
			temp = (data[i + 1] | (data[i] << 8));
			temperature = (float)temp / 340.0f + 36.53f;
		}
		else if(i >= 8 && i <= 13){
			temp = data[i + 1] | (data[i] << 8);
			RawOmega[(i - 8) / 2] = (float)temp * 0.06097560975609756097560975609756f;//0.03048780487804878048780487804878;//0.00763359f;// * 0.015267f;
		}
	}

	for(int i = 0; i < 3; i++){
		if((RawAcc[i] != RawAcc[i]) || (RawOmega[i] != RawOmega[i])){
			isValided = false;
			return false;
		}
	}

	for(int i = 0; i < 3; i++){
		RawAcc[i] *= RawAccScale[i];
		RawAcc[i] -= RawAccOffset[i];
//		RawOmega[i] -= getGyroTemperatureCompensation(i, temperature);
		RawOmega[i] *= RawOmegaScale[i];
		RawOmega[i] -= RawOmegaOffset[i];
		RawOmega[i] = MathTools::CutOff(RawOmega[i], 0.0f, 1.0f);
	}

	isValided = true;
	return true;
}

bool MPU6050::getIsValided(){
	return isValided;
}

void MPU6050::setTemperature(float value){
	temperature = value;
}

float MPU6050::getTemperature(){
	return temperature;
}

void MPU6050::setRawOmegaOffset(Vector3f value){
	RawOmegaOffset = value;
}

Vector3f MPU6050::getRawOmegaOffset(){
	return RawOmegaOffset;
}

void MPU6050::setRawOmega(Vector3f value){
	RawOmega = value;
}

Vector3f MPU6050::getRawOmega(){
	return RawOmega;
}

void MPU6050::setRawAcc(Vector3f value){
	RawAcc = value;
}

Vector3f MPU6050::getRawAcc(){
	return RawAcc;
}
