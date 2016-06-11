/*
 * Compass.cpp
 *
 *  Created on: 2015¦~12¤ë19¤é
 *      Author: wongy
 */

#include <Compass.h>

using namespace Inertia;
using namespace Utility;

Compass::Compass(Sensors::HMC5883L* mHMC5883L, Acceleration* mAcceleration) : _mHMC5883L(mHMC5883L), InitMagHead(0), _mAcceleration(mAcceleration), isValided(false){
	mMagMovingWindowAverageFilter[0] = new MovingWindowAverageFilter(10);
	mMagMovingWindowAverageFilter[1] = new MovingWindowAverageFilter(10);
	mMagMovingWindowAverageFilter[2] = new MovingWindowAverageFilter(10);
	Update();
	InitMagHead = getAngle()[2];
	InitMagHead = InitMagHead < 0 ? InitMagHead + 2 * Math::MathTools::PI : InitMagHead;
}

void Compass::Update(){
	if(_mHMC5883L->getIsValided()){
		Mag = _mHMC5883L->getRawMag();
		for(int i = 0; i < 3; i++){
			mMagMovingWindowAverageFilter[i]->Update(Mag[i]);
		}
		isValided = true;
	}
	else{
		isValided = false;
	}
}

bool Compass::getIsValided(){
	return isValided;
}

Vector3f Compass::getMag(){
	return Mag;
}

Vector3f Compass::getFilteredMag(){
	Vector3f acc;
	acc << mMagMovingWindowAverageFilter[0]->getAverage(),
			mMagMovingWindowAverageFilter[1]->getAverage(),
			mMagMovingWindowAverageFilter[2]->getAverage();
	return acc;
}

Vector3f Compass::getFilteredAngle(){
	Vector3f mag = getFilteredMag();
	Vector3f angle;
	Vector3f AccAngle = _mAcceleration->getFilteredAngle();
	float y = mag[2]*sinf(AccAngle[1]) - mag[1]*cosf(AccAngle[1]);
	float x = mag[0]*cosf(AccAngle[0]) + mag[1]*sinf(AccAngle[0])*sinf(AccAngle[1]) + mag[2]*sinf(AccAngle[0])*cosf(AccAngle[1]);
	angle[0] = 0.0f;
	angle[1] = 0.0f;
	angle[2] = -atan2(-y, x) - InitMagHead;
	angle[2] = angle[2] > MathTools::PI ? angle[2] -  2 * MathTools::PI:
			   angle[2] < -MathTools::PI ? 2 * MathTools::PI + angle[2] : angle[2];
	return angle;
}

void Compass::setMag(Vector3f value){
	Mag = value;
}

Vector3f Compass::getAngle(){
	Vector3f mag = getMag();
	Vector3f angle;
	Vector3f AccAngle = _mAcceleration->getFilteredAngle();
	float y = mag[2]*sinf(AccAngle[0]) - mag[1]*cosf(AccAngle[0]);
	float x = mag[0]*cosf(AccAngle[1]) + mag[1]*sinf(AccAngle[1])*sinf(AccAngle[0]) + mag[2]*sinf(AccAngle[1])*cosf(AccAngle[0]);
	angle[0] = 0.0f;
	angle[1] = 0.0f;
	angle[2] = -atan2(-y, x) - InitMagHead;
	angle[2] = angle[2] > MathTools::PI ? angle[2] -  2 * MathTools::PI:
			   angle[2] < -MathTools::PI ? 2 * MathTools::PI + angle[2] : angle[2];
	return angle;
}

void Compass::Reset(){
	mMagMovingWindowAverageFilter[0]->Clear();
	mMagMovingWindowAverageFilter[1]->Clear();
	mMagMovingWindowAverageFilter[2]->Clear();
	InitMagHead = 0;
	Update();
	InitMagHead = getAngle()[2];
	InitMagHead = InitMagHead < 0 ? InitMagHead + 2 * Math::MathTools::PI : InitMagHead;
}

