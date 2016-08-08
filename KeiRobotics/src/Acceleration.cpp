/*
 * Acceleration.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <Acceleration.h>

using namespace Inertia;

float Acceleration::Gravity = 9.80665f;

Acceleration::Acceleration(Sensors::MPU6050* mMPU6050) : isMPU6500(false), _mMPU6050(mMPU6050), isValided(false){
	mAccMovingWindowAverageFilter[0] = new MovingWindowAverageFilter(20);
	mAccMovingWindowAverageFilter[1] = new MovingWindowAverageFilter(20);
	mAccMovingWindowAverageFilter[2] = new MovingWindowAverageFilter(20);

	FilteredAcc.setZero();
	Matrix3f Q;
	Q.setIdentity();
	Q *= 1e-6f;
	Matrix3f R;
	R.setIdentity();
	R *= 1e-5f;
	AccKalman = new Kalman(FilteredAcc, Q, R);
	Update();
}

Acceleration::Acceleration(Sensors::MPU6500* mMPU6500) : isMPU6500(true), _mMPU6500(mMPU6500), isValided(false){
	mAccMovingWindowAverageFilter[0] = new MovingWindowAverageFilter(20);
	mAccMovingWindowAverageFilter[1] = new MovingWindowAverageFilter(20);
	mAccMovingWindowAverageFilter[2] = new MovingWindowAverageFilter(20);
	FilteredAcc.setZero();
	Matrix3f Q;
	Q.setIdentity();
	Q *= 1e-6f;
	Matrix3f R;
	R.setIdentity();
	R *= 1e-5f;
	AccKalman = new Kalman(FilteredAcc, Q, R);
	Update();
}

void Acceleration::Update(){
	if(isMPU6500){
		if(_mMPU6500->getIsValided()){

			Acc = _mMPU6500->getRawAcc();
			for(int i = 0; i < 3; i++){
				mAccMovingWindowAverageFilter[i]->Update(Acc[i]);
			}

//			Matrix3f A;
//			A.setIdentity();
//			Matrix3f H;
//			H.setIdentity();
//			if(AccKalman->Filtering(A, FilteredAcc, H, Acc)){
//				FilteredAcc = AccKalman->getCorrectedData();
////				for(int i = 0; i < 3; i++){
////					mAccMovingWindowAverageFilter[i]->Update(FilteredAcc[i]);
////				}
//			}
//			else{
//				isValided = false;
//			}
			isValided = true;
		}
		else{
			isValided = false;
		}
	}
	else{
		if(_mMPU6050->getIsValided()){
			Acc = _mMPU6050->getRawAcc();
			for(int i = 0; i < 3; i++){
				mAccMovingWindowAverageFilter[i]->Update(Acc[i]);
			}
//			Matrix3f A;
//			A.setIdentity();
//			Matrix3f H;
//			H.setIdentity();
//			if(AccKalman->Filtering(A, FilteredAcc, H, Acc)){
//				FilteredAcc = AccKalman->getCorrectedData();
////				for(int i = 0; i < 3; i++){
////					mAccMovingWindowAverageFilter[i]->Update(FilteredAcc[i]);
////				}
//			}
//			else{
//				isValided = false;
//			}
			isValided = true;
		}
		else{
			isValided = false;
		}
	}
}

bool Acceleration::getIsValided(){
	return isValided;
}

Vector3f Acceleration::getAcc(){
	return Acc;
}

Vector3f Acceleration::getFilteredAcc(){
	Vector3f acc;
	acc << mAccMovingWindowAverageFilter[0]->getAverage(),
		   mAccMovingWindowAverageFilter[1]->getAverage(),
		   mAccMovingWindowAverageFilter[2]->getAverage();
//	acc << mAccMovingWindowAverageFilter[0]->getMedian(),
//		   mAccMovingWindowAverageFilter[1]->getMedian(),
//		   mAccMovingWindowAverageFilter[2]->getMedian();
	return acc;
//	return FilteredAcc;
}

Vector3f Acceleration::getFilteredAngle(){
	Vector3f acc = getFilteredAcc();
	Vector3f angle;
	angle[0] = atan2(acc[1], sqrtf(acc[0] * acc[0] + acc[2] * acc[2]));
	angle[1] = atan2(-acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2]));
	angle[2] = 0.0f;
	return angle;
}

void Acceleration::setAcc(Vector3f value){
	Acc = value;
}

Vector3f Acceleration::getAngle(){
	Vector3f angle;

	angle[0] = atan2(Acc[1], sqrtf(Acc[0] * Acc[0] + Acc[2] * Acc[2]));
	angle[1] = atan2(-Acc[0], sqrtf(Acc[1] * Acc[1] + Acc[2] * Acc[2]));
	angle[2] = 0.0f;
	return angle;
}
