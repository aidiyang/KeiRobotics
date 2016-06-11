/*
 * HMC5883L.cpp
 *
 *  Created on: 2014¦~8¤ë23¤é
 *      Author: YunKei
 */

#include <HMC5883L.h>
using namespace Sensors;

float HMC5883L::RawMagPosX = 726.35;
float HMC5883L::RawMagNegX = -740.22;
float HMC5883L::RawMagPosY = 708.1;
float HMC5883L::RawMagNegY = -778.91;
float HMC5883L::RawMagPosZ = 873.77;
float HMC5883L::RawMagNegZ = -889.87;

//float HMC5883L::RawMagPosX = 1.0f;
//float HMC5883L::RawMagNegX = -1.0f;
//float HMC5883L::RawMagPosY = 1.0f;
//float HMC5883L::RawMagNegY = -1.0f;
//float HMC5883L::RawMagPosZ = 1.0f;
//float HMC5883L::RawMagNegZ = -1.0f;

void HMC5883L::CalibrationPrint(){
	static int count = 0;
	for(int i = 0; i < 3; i++){
		MaxMag[i] = MaxMag[i] < RawMag[i] ? RawMag[i] : MaxMag[i];
		MinMag[i] = MinMag[i] > RawMag[i] ? RawMag[i] : MinMag[i];
	}
	printf("%d\n", count);
	if(count++ >= 1499){
		Delay::DelayMS(100);
		CalibrationResultPrint();
	}
}

void HMC5883L::CalibrationResultPrint(){
	printf("float HMC5883L::RawMagPosX = %g;\n", MaxMag[0]);
	printf("float HMC5883L::RawMagNegX = %g;\n", MinMag[0]);
	printf("float HMC5883L::RawMagPosY = %g;\n", MaxMag[1]);
	printf("float HMC5883L::RawMagNegY = %g;\n", MinMag[1]);
	printf("float HMC5883L::RawMagPosZ = %g;\n", MaxMag[2]);
	printf("float HMC5883L::RawMagNegZ = %g;\n", MinMag[2]);
}

HMC5883L::HMC5883L(MPU6050* mMPU6050) : _mMPU6050(mMPU6050), i2cx(mMPU6050->i2cx), isValided(false){

	MaxMag << -99999, -99999, -99999;
	MinMag << 99999, 99999, 99999;

	RawMagScale[0] = 2.0f / (RawMagPosX - RawMagNegX);
	RawMagScale[1] = 2.0f / (RawMagPosY - RawMagNegY);
	RawMagScale[2] = 2.0f / (RawMagPosZ - RawMagNegZ);
	RawMagOffset[0] = RawMagPosX * RawMagScale[0] - 1.0f;
	RawMagOffset[1] = RawMagPosY * RawMagScale[1] - 1.0f;
	RawMagOffset[2] = RawMagPosZ * RawMagScale[2] - 1.0f;
	FastInitialization();
}

void HMC5883L::FastInitialization(){

	_mMPU6050->setI2CBypass(true);
	App::mApp->mTicks->setTimeout(3);
	while(!i2cx->Write(ADDRESS,CFG_REG_A,0x18)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}
	App::mApp->mTicks->setTimeout(3);
	while(!i2cx->Write(ADDRESS,CFG_REG_B,0x00)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}
	App::mApp->mTicks->setTimeout(3);
	while(!i2cx->Write(ADDRESS,MODE_REG,0x00)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}

	_mMPU6050->setI2CBypass(false);
}

bool HMC5883L::Update(){

	uint8_t data[6];
	int16_t temp;

	_mMPU6050->setI2CBypass(true);
	if(!i2cx->BurstRead(ADDRESS, DATA_OUT_X_MSB, 6, data)){
		FastInitialization();
		isValided = false;
		return false;
	}

	for(int i = 0; i < 6; i += 2){
		int j = i / 2;
		temp = (data[i + 1] | (data[i] << 8));
		RawMag[j] = (float)temp * 0.73f;
	}
	float swap = RawMag[1];
	RawMag[1] = RawMag[2];
	RawMag[2] = swap;
	for(int i = 0; i < 3; i++){
		if(RawMag[i] != RawMag[i]){
			isValided = false;
			return false;
		}
	}

	for(int i = 0; i < 3; i++){
		RawMag[i] *= RawMagScale[i];
	}
	RawMag -= RawMagOffset;
	isValided = true;
	_mMPU6050->setI2CBypass(false);
	return true;
}

bool HMC5883L::getIsValided(){
	return isValided;
}

void HMC5883L::setRawMag(Vector3f value){
	RawMag = value;
}

Vector3f HMC5883L::getRawMag(){
	return RawMag;
}
