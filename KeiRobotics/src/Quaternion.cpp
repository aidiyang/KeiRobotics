/*
 * Quaternion.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <App.h>
#include <Quaternion.h>
#include <MathTools.h>
#include <math.h>
#include <Kalman.h>
#include <MPU6050.h>
#include <stdio.h>
#include <Pid.h>
#include <Task.h>
#include <HMC5883L.h>
#include <Controlling.h>
#include <Delay.h>
#include <AdditionalTools.h>
#include <Communicating.h>
#include <Led.h>
#include <UART.h>
#include <EncoderYaw.h>

using namespace Utility;

Quaternion::Quaternion(Acceleration* mAcceleration, Omega* mOmega) : _mAcceleration(mAcceleration), _mOmega(mOmega), IsUseCompass(false), IsUseEncoderYaw(false), _mCompass(0), Interval(0), Valid(false), _mEncoderYaw((EncoderYaw*)0){
	PrevT.setZero();
	Matrix3f Q;
	Q.setIdentity();
	Q *= 1e-6f;
	Q(2,2) = 1e-12f;
	Matrix3f R;
	R.setIdentity();
	R *= 1e-2f;
	R(2,2) *= 1e-6f;
	_QuaternionKalman = new Kalman(mAcceleration->getAngle(), Q, R);
	PrevTick = App::mApp->mTicks->getTicks();
}

Quaternion::Quaternion(Acceleration* mAcceleration, Omega* mOmega, Compass* mCompass) : _mAcceleration(mAcceleration), _mOmega(mOmega), IsUseCompass(true), IsUseEncoderYaw(false), Interval(0), Valid(false), _mCompass(mCompass), _mEncoderYaw((EncoderYaw*)0){
	PrevT.setZero();
	Matrix3f Q;
	Q.setIdentity();
	Q *= 1e-6f;
	Matrix3f R;
	R.setIdentity();
	R *= 1e-1f;
	_QuaternionKalman = new Kalman(mAcceleration->getAngle(), Q, R);
	PrevTick = App::mApp->mTicks->getTicks();
}


Quaternion::Quaternion(Acceleration* mAcceleration, Omega* mOmega, EncoderYaw* mEncoderYaw) : _mAcceleration(mAcceleration), _mOmega(mOmega), IsUseCompass(false), IsUseEncoderYaw(true), _mCompass(0), Interval(0), Valid(false), _mEncoderYaw(mEncoderYaw){
	PrevT.setZero();
	Matrix3f Q;
	Q.setIdentity();
	Q *= 1e-6f;
	Matrix3f R;
	R.setIdentity();
	R *= 1e-1f;
	_QuaternionKalman = new Kalman(mAcceleration->getAngle(), Q, R);
	PrevTick = App::mApp->mTicks->getTicks();
}

bool Quaternion::Update(){
	if(_mOmega->getIsValided()){
		Vector3f omega = _mOmega->getOmega() * MathTools::RADIAN_PER_DEGREE;
		Vector4f q;
		q << 0, omega[0], omega[1], omega[2];
		Vector4f t;
		t[0] = -q[1]*_Quaternion[1]-q[2]*_Quaternion[2]-q[3]*_Quaternion[3];
		t[1] = q[1]*_Quaternion[0]+q[2]*_Quaternion[3]-q[3]*_Quaternion[2];
		t[2] = -q[1]*_Quaternion[3]+q[2]*_Quaternion[0]+q[3]*_Quaternion[1];
		t[3] = q[1]*_Quaternion[2]-q[2]*_Quaternion[1]+q[3]*_Quaternion[0];
		Interval = App::mApp->mTicks->getTicks() - PrevTick;
		PrevTick = App::mApp->mTicks->getTicks();
		Interval /= 1000.0f;
		if(Interval <= 0){
			Valid = false;
			return false;
		}
		t *= 0.5f * Interval;
		q = _Quaternion + 0.5f * (t + PrevT);
		PrevT = t;
		q.normalize();
		Vector3f e = QuaternionToEuler(q);
		bool valid = true;
		bool AccValid = _mAcceleration->getIsValided() && (_mAcceleration->getAcc().norm() > Acceleration::Gravity * 0.95f) && (_mAcceleration->getAcc().norm() < Acceleration::Gravity * 1.05f);
		Vector3f angle;
		bool MagValid;

		if(IsUseCompass){
			MagValid = _mCompass->getIsValided() && _mCompass->getMag().norm() > 0.9f && _mCompass->getMag().norm() < 1.1f;
		}
		if(AccValid){
			angle = _mAcceleration->getAngle();
			if(IsUseCompass && MagValid){
				angle[2] = _mCompass->getAngle()[2];
			}
			else if(IsUseEncoderYaw){
				angle[2] = _mEncoderYaw->getYaw();
			}
			else{
				angle[2] = e[2];
			}
		}
		else{
			angle[0] = e[0];
			angle[1] = e[1];
			if(IsUseCompass && MagValid){
				angle[2] = _mCompass->getAngle()[2];
			}
			else if(IsUseEncoderYaw){
				angle[2] = _mEncoderYaw->getYaw();
			}
			else{
				angle[2] = e[2];
			}
		}

		Matrix3f A;
		A.setIdentity();
		Matrix3f H;
		H.setIdentity();
		if(valid && AccValid && _QuaternionKalman->Filtering(A, e, H, angle)){
			angle = _QuaternionKalman->getCorrectedData();
			for(int i = 0; i < 3; i++){
				if(angle[i] != angle[i]){
					Valid = false;
					return false;
				}
			}
			if(IsUseEncoderYaw && fabs(fabs(_mEncoderYaw->getYaw()) - MathTools::PI) < 0.01f){
				angle[2] = _mEncoderYaw->getYaw();
			}
			_Euler = angle;
		}
		else{
			_Euler = e;
		}
		_Quaternion = EulerToQuaternion(_Euler);
		return true;
	}
	else{
		Valid = false;
		return false;
	}
}

Matrix4f Quaternion::calcStateTransMatrix(Vector3f w, float t){
	Matrix4f A;
	A << 0, -w[0], -w[1], -w[2],
		w[0], 0, w[2], -w[1],
		w[1], -w[2], 0, w[0],
		w[2], w[1], -w[0], 0;
	A *= 0.5f;
	Matrix4f T;
	T.setIdentity();
	T += A * t;
	return T;
}

Eigen::Matrix<float, 3, 4> Quaternion::calcQuatToEulerMeasMatrix(Vector4f q){
	Eigen::Matrix<float, 3, 4> C;
	Matrix3f R = QuaternionToMatrix(q);
	Eigen::Matrix<float, 3, 4> n;
	n << 2*R(2,2)*q[1],
		 2*(R(2,2)*q[0]+2*R(1,2)*q[1]),
		 2*(R(2,2)*q[3]+2*R(1,2)*q[2]),
		 2*R(2,2)*q[2],
		 2*q[2],
		 -2*q[3],
		 2*q[0],
		 -2*q[1],
		 2*R(0,0)*q[3],
		 2*R(0,0)*q[2],
		 2*(R(0,0)*q[0]+2*R(0,1)*q[2]),
		 2*(R(0,0)*q[0]+2*R(0,1)*q[3]);
	Eigen::Matrix<float, 3, 1> d;
	d << R(2,2)*R(2,2)+R(1,2)*R(1,2),
		 sqrtf(1-R(0,2)*R(0,2)),
		 R(0,0)*R(0,0)+R(0,1)*R(0,1);
	C << -n(0,0)/d(0,0),-n(0,1)/d(0,0),-n(0,2)/d(0,0),-n(0,3)/d(0,0),
		 n(1,0)/d(1,0),n(1,1)/d(1,0),n(1,2)/d(1,0),n(1,3)/d(1,0),
		 -n(2,0)/d(2,0),-n(2,1)/d(2,0),-n(2,2)/d(2,0),-n(2,3)/d(2,0);
	return C;
}

Vector3f Quaternion::QuaternionToEuler(Vector4f q){
	Vector3f euler;
	euler[0] = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
	float a = 2 * (q[0] * q[1] + q[2] * q[3]), b = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
	float r32 = 2 * (q[0] * q[1] + q[2] * q[3]);
	float r33 = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
	euler[1] = atan2f(-2 * (q[1] * q[3] - q[0] * q[2]), sqrtf(r32 * r32 + r33 * r33));
	euler[2] = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
	return euler;
}

Vector4f Quaternion::EulerToQuaternion(Vector3f euler){

	float CosHalfRoll = cosf(euler[0] / 2.0f), SinHalfRoll = sinf(euler[0] / 2.0f);
	float CosHalfPitch = cosf(euler[1] / 2.0f), SinHalfPitch = sinf(euler[1] / 2.0f);
	float CosHalfYaw = cosf(euler[2] / 2.0f), SinHalfYaw = sinf(euler[2] / 2.0f);
	Vector4f q;
	q[0] = CosHalfRoll * CosHalfPitch * CosHalfYaw + SinHalfRoll * SinHalfPitch * SinHalfYaw;
	q[1] = SinHalfRoll * CosHalfPitch * CosHalfYaw - CosHalfRoll * SinHalfPitch * SinHalfYaw;
	q[2] = CosHalfRoll * SinHalfPitch * CosHalfYaw + SinHalfRoll * CosHalfPitch * SinHalfYaw;
	q[3] = CosHalfRoll * CosHalfPitch * SinHalfYaw - SinHalfRoll * SinHalfPitch * CosHalfYaw;
	return q;
}

Vector4f Quaternion::FixedAnglesToQuaternion(Vector3f fixedAngles){
	float CosHalfRoll = cosf(fixedAngles[0] / 2.0f), SinHalfRoll = sinf(fixedAngles[0] / 2.0f);
	float CosHalfPitch = cosf(fixedAngles[1] / 2.0f), SinHalfPitch = sinf(fixedAngles[1] / 2.0f);
	float CosHalfYaw = cosf(fixedAngles[2] / 2.0f), SinHalfYaw = sinf(fixedAngles[2] / 2.0f);
	Vector4f q;
	q[0] = CosHalfRoll * CosHalfPitch * CosHalfYaw - SinHalfRoll * SinHalfPitch * SinHalfYaw;
	q[1] = CosHalfRoll * SinHalfPitch * SinHalfYaw + SinHalfRoll * CosHalfPitch * CosHalfYaw;
	q[2] = CosHalfRoll * SinHalfPitch * CosHalfYaw - SinHalfRoll * CosHalfPitch * SinHalfYaw;
	q[3] = CosHalfRoll * CosHalfPitch * SinHalfYaw + SinHalfRoll * SinHalfPitch * CosHalfYaw;
	return q;
}

Matrix3f Quaternion::EulerToMatrix(Vector3f euler){
	Matrix3f R;
	float CosRoll = cosf(euler[0]), SinRoll = sinf(euler[0]);
	float CosPitch = cosf(euler[1]), SinPitch = sinf(euler[1]);
	float CosYaw = cosf(euler[2]), SinYaw = sinf(euler[2]);
	R(0,0) = CosYaw * CosPitch;
	R(0,1) = -SinYaw * CosRoll + CosYaw * SinPitch * SinRoll;
	R(0,2) = SinYaw * SinRoll + CosYaw * SinPitch * CosRoll;
	R(1,0) = SinYaw * CosPitch;
	R(1,1) = CosYaw * CosRoll + SinYaw * SinPitch * SinRoll;
	R(1,2) = -CosYaw * SinRoll + SinYaw * SinPitch * CosRoll;
	R(2,0) = -SinPitch;
	R(2,1) = CosPitch * SinRoll;
	R(2,2) = CosPitch * CosRoll;
	return R;
}

Vector3f Quaternion::MatrixToFixedAngles(Matrix3f R){
	Vector3f fixedAngles;
	fixedAngles[0] = atan2f(-R(1,2), R(2,2));
	fixedAngles[1] = atan2f(R(0,2), sqrtf(R(1,2)*R(1,2) + R(2,2)*R(2,2)));
	fixedAngles[2] = atan2f(-R(0,1), R(0,0));
	return fixedAngles;
}

Matrix3f Quaternion::QuaternionToMatrix(Vector4f q){
	Matrix3f M;
	M << 1-2*(q[2]*q[2]+q[3]*q[3]),
		 2*(q[1]*q[2]-q[0]*q[3]),
		 2*(q[1]*q[3]+q[0]*q[2]),
		 2*(q[1]*q[2]+q[0]*q[3]),
		 1-2*(q[1]*q[1]+q[3]*q[3]),
		 2*(q[2]*q[3]-q[0]*q[1]),
		 2*(q[1]*q[3]-q[0]*q[2]),
		 2*(q[2]*q[3]+q[0]*q[1]),
		 1-2*(q[1]*q[1]+q[2]*q[2]);
	return M;
}

Vector3f Quaternion::getEuler(){
	return _Euler;
}

void Quaternion::setEuler(Vector3f angle){
	_QuaternionKalman->Clear(angle);
	_Quaternion = EulerToQuaternion(angle);
	PrevT.setZero();
}

void Quaternion::Reset(){
	Vector3f angle = _mAcceleration->getFilteredAngle();
	if(IsUseCompass){
		angle[2] = _mCompass->getFilteredAngle()[2];
	}
	else{
		angle[2] = 0;
	}
	_QuaternionKalman->Clear(angle);
	_Quaternion = EulerToQuaternion(angle);
	PrevT.setZero();
}
