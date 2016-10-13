/*
 * Omega.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <Omega.h>

Omega::Omega(MPU6050* mMPU6050) : isMPU6500(false), _mMPU6050(mMPU6050), isValided(false){
	mOmegaMovingWindowAverageFilter[0] = new MovingWindowAverageFilter(100);
	mOmegaMovingWindowAverageFilter[1] = new MovingWindowAverageFilter(100);
	mOmegaMovingWindowAverageFilter[2] = new MovingWindowAverageFilter(100);
	_FilteredOmega.setZero();
	Matrix3f Q;
	Q.setIdentity();
	Q *= 1e-6f;
	Matrix3f R;
	R.setIdentity();
	R *= 1e-5f;
	OmegaKalman = new Kalman(_FilteredOmega, Q, R);
	Update();
}

Omega::Omega(MPU6500* mMPU6500) : isMPU6500(true), _mMPU6500(mMPU6500), isValided(false){
	mOmegaMovingWindowAverageFilter[0] = new MovingWindowAverageFilter(100);
	mOmegaMovingWindowAverageFilter[1] = new MovingWindowAverageFilter(100);
	mOmegaMovingWindowAverageFilter[2] = new MovingWindowAverageFilter(100);
	_FilteredOmega.setZero();
	Matrix3f Q;
	Q.setIdentity();
	Q *= 1e-6f;
	Matrix3f R;
	R.setIdentity();
	R *= 1e-5f;
	OmegaKalman = new Kalman(_FilteredOmega, Q, R);
	Update();
}

void Omega::Update(){
	if(isMPU6500){
		if(_mMPU6500->getIsValided()){
			_Omega = _mMPU6500->getRawOmega();
//			for(int i = 0; i < 3; i++){
//				mOmegaMovingWindowAverageFilter[i]->Update(_mMPU6500->getRawOmega()[i]);
//				//_FilteredOmega[i] = mOmegaMovingWindowAverageFilter[i]->getAverage();
//			}
//
//			Matrix3f A;
//			A.setIdentity();
//			Matrix3f H;
//			H.setIdentity();
//			if(OmegaKalman->Filtering(A, _FilteredOmega, H, _Omega)){
//				_FilteredOmega = OmegaKalman->getCorrectedData();
//				for(int i = 0; i < 3; i++){
//					_FilteredOmega[i] = MathTools::CutOff(_FilteredOmega[i], 0.0f, 1.0f);
//				}
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
			_Omega = _mMPU6050->getRawOmega();
			for(int i = 0; i < 3; i++){
				mOmegaMovingWindowAverageFilter[i]->Update(_mMPU6050->getRawOmega()[i]);
				//_FilteredOmega[i] = mOmegaMovingWindowAverageFilter[i]->getAverage();
			}

			Matrix3f A;
			A.setIdentity();
			Matrix3f H;
			H.setIdentity();
			if(OmegaKalman->Filtering(A, _FilteredOmega, H, _Omega)){
				_FilteredOmega = OmegaKalman->getCorrectedData();
				for(int i = 0; i < 3; i++){
					_FilteredOmega[i] = MathTools::CutOff(_FilteredOmega[i], 0.0f, 2.0f);
				}
			}
			else{
				isValided = false;
			}

			isValided = true;
		}
		else{
			isValided = false;
		}
	}
}

bool Omega::getIsValided(){
	return isValided;
}


Vector3f Omega::getFilteredOmega(){
	return _FilteredOmega;
}

Vector3f Omega::getOmega(){
	return _Omega;
}

void Omega::setOmega(Vector3f value){
	_Omega = value;
}
