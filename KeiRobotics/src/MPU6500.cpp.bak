/*
 * MPU6500.cpp
 *
 *  Created on: 2014¦~8¤ë23¤é
 *      Author: YunKei
 */

#include <MPU6500.h>
#include <Spi.h>
#include <Task.h>
#include <stdio.h>
#include <math.h>
#include <MathTools.h>
#include <Acceleration.h>
#include <Delay.h>
#include <App.h>
#include <UART.h>

MPU6500* _mMPU6500[6];

MPU6500::MPU6500(int index, Spi* spi) : Sensitivity(SENSITIVITY_2000), DevIndex(index), spix(spi), isValided(false){

	_mMPU6500[index] = this;

	RawAccPosX = getAccMax(DevIndex, 0);
	RawAccNegX = getAccMin(DevIndex, 0);
	RawAccPosY = getAccMax(DevIndex, 1);
	RawAccNegY = getAccMin(DevIndex, 1);
	RawAccPosZ = getAccMax(DevIndex, 2);
	RawAccNegZ = getAccMin(DevIndex, 2);

	RawAccScale[0] = 2.0*GRAVITY / (RawAccPosX - RawAccNegX);
	RawAccScale[1] = 2.0*GRAVITY / (RawAccPosY - RawAccNegY);
	RawAccScale[2] = 2.0*GRAVITY / (RawAccPosZ - RawAccNegZ);
	RawAccOffset[0] = RawAccPosX * RawAccScale[0] - GRAVITY;
	RawAccOffset[1] = RawAccPosY * RawAccScale[1] - GRAVITY;
	RawAccOffset[2] = RawAccPosZ * RawAccScale[2] - GRAVITY;

	FastInitialization();

	RawOmegaOffset[0] = 0;
	RawOmegaOffset[1] = 0;
	RawOmegaOffset[2] = 0;

	for(int i = 0; i < 32; i++){
		Update();
		Delay::DelayMS(2);
	}
}

void MPU6500::setSensitivity(SENSITIVITY value){
	Sensitivity = value;
	switch(Sensitivity){
		case SENSITIVITY::SENSITIVITY_250:
			App::mApp->mTicks->setTimeout(3);
			while(!spix->WriteCmd(DevIndex, GYRO_CONFIG, 0x00)){
				if(App::mApp->mTicks->Timeout()){
					return;
				}
			}
			break;

		case SENSITIVITY::SENSITIVITY_500:
			App::mApp->mTicks->setTimeout(3);
			while(!spix->WriteCmd(DevIndex, GYRO_CONFIG, 0x08)){
				if(App::mApp->mTicks->Timeout()){
					return;
				}
			}
			break;

		case SENSITIVITY::SENSITIVITY_1000:
			App::mApp->mTicks->setTimeout(3);
			while(!spix->WriteCmd(DevIndex, GYRO_CONFIG, 0x10)){
				if(App::mApp->mTicks->Timeout()){
					return;
				}
			}
			break;

		case SENSITIVITY::SENSITIVITY_2000:
			App::mApp->mTicks->setTimeout(3);
			while(!spix->WriteCmd(DevIndex, GYRO_CONFIG, 0x18)){
				if(App::mApp->mTicks->Timeout()){
					return;
				}
			}
			break;
	}
}

void MPU6500::FastInitialization(){

	App::mApp->mTicks->setTimeout(3);
	while(!spix->WriteCmd(DevIndex, PWR_MGMT_1, 0x00)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}

	App::mApp->mTicks->setTimeout(3);
	while(!spix->WriteCmd(DevIndex, SMPLRT_DIV, 0x07)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}

	App::mApp->mTicks->setTimeout(3);
	while(!spix->WriteCmd(DevIndex, CONFIG, 0x00)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}

	App::mApp->mTicks->setTimeout(3);
	while(!spix->WriteCmd(DevIndex, GYRO_CONFIG, 0x18)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}

	App::mApp->mTicks->setTimeout(3);
	while(!spix->WriteCmd(DevIndex, ACCEL_CONFIG, 0x18)){
		if(App::mApp->mTicks->Timeout()){
			return;
		}
	}
}

MPU6500* MPU6500::getInstance(int index){

	return _mMPU6500[index];
}

bool MPU6500::Update(){

	uint8_t data[14];
	int16_t temp;

	float scale;

	switch(Sensitivity){
		case SENSITIVITY::SENSITIVITY_250:
			scale = 0.0076335877862595f;
			break;
		case SENSITIVITY::SENSITIVITY_500:
			scale = 0.0152671755725191f;
			break;
		case SENSITIVITY::SENSITIVITY_1000:
			scale = 0.0304878048780488f;
			break;
		case SENSITIVITY::SENSITIVITY_2000:
			scale = 0.0609756097560976f;
			break;
	}

	if(!spix->ReadNBytes(DevIndex, ACCEL_XOUT_H | 0x80, 14, data)){
		FastInitialization();
		isValided = false;
		return false;
	}

	for(int i = 0; i < 14; i += 2){

		if(i >= 0 && i <= 5){
			int j = i / 2;
			temp = (data[i + 1] | (data[i] << 8));
			RawAcc[j] = (float)temp * 0.00048828125f * GRAVITY;
		}
		else if(i >= 6 && i <= 7){
			temp = (data[i + 1] | (data[i] << 8));
			temperature = (float)temp / 333.87f + 21.0f;
		}
		else if(i >= 8 && i <= 13){
			temp = data[i + 1] | (data[i] << 8);
			RawOmega[(i - 8) / 2] = (float)temp * scale;
		}
	}

	float swap;

	swap = RawAcc[0];
	RawAcc[0] = -RawAcc[1];
	RawAcc[1] = swap;
	RawAcc[2] = -RawAcc[2];

	swap = RawOmega[0];
	RawOmega[0] = RawOmega[1];
	RawOmega[1] = -swap;

	for(int i = 0; i < 3; i++){
		RawAcc[i] *= RawAccScale[i];
		RawAcc[i] -= RawAccOffset[i];
		RawOmega[i] -= getGyroTemperatureCompensation(DevIndex, i, temperature);
		RawOmega[i] = MathTools::CutOff(RawOmega[i], 0.0f, 2.0f);
	}

	isValided = true;
	return true;
}

float MPU6500::getGyroTemperatureCompensation(int index, int channel, float temp){
	float value = 0;
	switch(index){
		case 0:
			switch(channel){
				case 0:
					value = 0.0005f*temp*temp - 0.0799f*temp + 12.212f;
					break;
				case 1:
					value = -0.0002f*temp*temp - 0.0039f*temp + 4.6503f;
					break;
				case 2:
					value = 0.00007*temp*temp + 0.0047f*temp - 0.8594f;
					break;
			}
			break;
		case 1:
			switch(channel){
				case 0:
					value = 0.001f*temp*temp - 0.0992f*temp + 6.6046f;
					break;
				case 1:
					value = 0.00002f*temp*temp + 0.032f*temp - 0.3928f;
					break;
				case 2:
					value = 0.00006f*temp*temp + 0.0005f*temp - 0.0949f;
					break;
			}
			break;
		case 2:
			switch(channel){
				case 0:
					value = 0.0011f*temp*temp - 0.1f*temp + 7.0212f;
					break;
				case 1:
					value = -0.0001f*temp*temp + 0.0083f*temp + 0.9077f;
					break;
				case 2:
					value = -0.00004f*temp*temp + 0.0015f*temp - 0.5488f;
					break;
			}
			break;
		case 3:
			switch(channel){
				case 0:
					value = 0.0003f*temp*temp + 0.0101f*temp + 2.1163f;
					break;
				case 1:
					value = 0.0002f*temp*temp - 0.0083f*temp - 1.0702f;
					break;
				case 2:
					value = -0.000003f*temp*temp - 0.0131f*temp + 0.9225f;
					break;
			}
			break;
		case 4:
			switch(channel){
				case 0:
					value = 0.0003f*temp*temp - 0.0176f*temp + 3.6351f;
					break;
				case 1:
					value = -0.00005f*temp*temp + 0.0242f*temp - 0.9812f;
					break;
				case 2:
					value = -0.00005f*temp*temp + 0.0199f*temp - 1.1762f;
					break;
			}
			break;
		case 5:
			switch(channel){
				case 0:
					value = 0.0009f*temp*temp - 0.1001f*temp + 4.3651f;
					break;
				case 1:
					value = -0.0001f*temp*temp + 0.0385f*temp - 2.0141f;
					break;
				case 2:
					value = -0.00004f*temp*temp + 0.0036f*temp + 1.1112f;
					break;
			}
			break;
	}
	return value;
}

float MPU6500::getAccMax(int index, int channel){
	float value = 0;
	switch(index){
		case 0:
			switch(channel){
				case 0:
					value = 9.7f;
					break;
				case 1:
					value = 10.1f;
					break;
				case 2:
					value = 10.2f;
					break;
			}
			break;
		case 1:
			switch(channel){
				case 0:
					value = 9.7f;
					break;
				case 1:
					value = 10.1f;
					break;
				case 2:
					value = 10.0f;
					break;
			}
			break;
		case 2:
			switch(channel){
				case 0:
					value = 9.7f;
					break;
				case 1:
					value = 10.1f;
					break;
				case 2:
					value = 10.0f;
					break;
			}
			break;
		case 3:
			switch(channel){
				case 0:
					value = 9.8f;
					break;
				case 1:
					value = 10.2f;
					break;
				case 2:
					value = 10.0f;
					break;
			}
			break;
		case 4:
			switch(channel){
				case 0:
					value = 9.7f;
					break;
				case 1:
					value = 10.1f;
					break;
				case 2:
					value = 9.8f;
					break;
			}
			break;
		case 5:
			switch(channel){
				case 0:
					value = 10.0f;
					break;
				case 1:
					value = 10.1f;
					break;
				case 2:
					value = 9.7f;
					break;
			}
			break;
	}
	return value;
}

float MPU6500::getAccMin(int index, int channel){
	float value = 0;
	switch(index){
		case 0:
			switch(channel){
				case 0:
					value = -10.0f;
					break;
				case 1:
					value = -9.4f;
					break;
				case 2:
					value = -9.8f;
					break;
			}
			break;
		case 1:
			switch(channel){
				case 0:
					value = -10.0f;
					break;
				case 1:
					value = -9.5f;
					break;
				case 2:
					value = -9.9f;
					break;
			}
			break;
		case 2:
			switch(channel){
				case 0:
					value = -9.9f;
					break;
				case 1:
					value = -9.6f;
					break;
				case 2:
					value = -10.0f;
					break;
			}
			break;
		case 3:
			switch(channel){
				case 0:
					value = -9.9f;
					break;
				case 1:
					value = -9.5f;
					break;
				case 2:
					value = -10.0f;
					break;
			}
			break;
		case 4:
			switch(channel){
				case 0:
					value = -9.8f;
					break;
				case 1:
					value = -9.5f;
					break;
				case 2:
					value = -10.0f;
					break;
			}
			break;
		case 5:
			switch(channel){
				case 0:
					value = -9.8f;
					break;
				case 1:
					value = -9.6f;
					break;
				case 2:
					value = -10.1f;
					break;
			}
			break;
	}
	return value;
}

bool MPU6500::getIsValided(){
	return isValided;
}

void MPU6500::setTemperature(float value){
	temperature = value;
}

float MPU6500::getTemperature(){
	return temperature;
}

void MPU6500::setRawOmegaOffset(int index, float value){
	RawOmegaOffset[index] = value;
}

float MPU6500::getRawOmegaOffset(int index){
	return RawOmegaOffset[index];
}

void MPU6500::setRawOmega(int index, float value){
	RawOmega[index] = value;
}

float MPU6500::getRawOmega(int index){
	return RawOmega[index];
}

void MPU6500::setRawAcc(int index, float value){
	RawAcc[index] = value;
}

float MPU6500::getRawAcc(int index){
	return RawAcc[index];
}
