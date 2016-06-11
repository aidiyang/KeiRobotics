/*
 * PX4FLOW.cpp
 *
 *  Created on: 2015¦~5¤ë13¤é
 *      Author: YunKei
 */

#include <PX4FLOW.h>

PX4FLOW* _mPX4FLOW;

PX4FLOW::PX4FLOW(I2C_TypeDef* I2Cx, float interval) : PX4FLOW_I2C(I2Cx), Interval(interval), prevZ(0), isValided(false){
	_mPX4FLOW = this;
	Translation.setZero();
	prevZ = 0;
	float r[2] = {0.001, -1};
	flowKalman[0] = new Kalman(0.0001f, r, 0, 1);
	flowKalman[1] = new Kalman(0.0001f, r, 0, 1);
	flowKalman[2] = new Kalman(0.0001f, r, 0, 1);
	offset[0] = 0.0f;
	offset[1] = 0.0;
	offset[2] = 0.0;
	Update();
//	Vector3f x;
//	x.setZero();
//	Matrix3d p;
//	p.setIdentity();
//	Matrix3d q;
//	q.setIdentity();
//	q*=0.0001;
//	Matrix3d r;
//	r.setIdentity();
//	r*=0.001;
//	flowUKF = new UKF(x,p,q,r);
}

PX4FLOW* PX4FLOW::getInstance(){
	return _mPX4FLOW;
}

void PX4FLOW::Update(){
	uint8_t data[6];
	int16_t temp;

	if(!I2C::getInstance(PX4FLOW_I2C)->BurstRead(0x42, 0x06, 4, data)){
		isValided = false;
		return;
	}
	if(!I2C::getInstance(PX4FLOW_I2C)->BurstRead(0x42, 0x14, 2, &data[4])){
		isValided = false;
		return;
	}
	float velocityX = 0;
	float velocityY = 0;
	float posZ = 0;
//	float cosRcosP = cosf(Quaternion::getInstance()->getEuler(0) - Quaternion::getInstance()->getInitAngles(0)) * cosf(Quaternion::getInstance()->getEuler(1) - Quaternion::getInstance()->getInitAngles(1));
//	temp = ((uint16_t)data[0] | ((uint16_t)data[1] << 8));
//	velocityY = -((float)temp) / 1000.0;
//	velocityY -= offset[1];
//	velocityY *= cosRcosP;
//	temp = ((uint16_t)data[2] | ((uint16_t)data[3] << 8));
//	velocityX = -((float)temp) / 1000.0;
//	velocityX -= offset[0];
//	velocityX *= cosRcosP;
//	temp = ((uint16_t)data[4] | ((uint16_t)data[5] << 8));
//	posZ = ((float)temp) / 1000.0;
//	Translation(0) = velocityX * Interval;
//	Translation(1) = velocityY * Interval;
//	Translation(2) = (posZ - prevZ) - offset[2];
////	flowKalman[0]->Filtering(&Translation(0), Translation(0), 0);
////	flowKalman[1]->Filtering(&Translation(1), Translation(1), 0);
////	flowKalman[2]->Filtering(&Translation(2), Translation(2), 0);
////	Vector3f x[7];
////	flowUKF->getSigmaPoints(x);
////	Translation = flowUKF->Filtering(x,x,Translation);
//	prevZ = posZ;
//	isValided = true;
}

bool PX4FLOW::getIsValided(){
	return isValided;
}

Vector3f PX4FLOW::getTranslation(){
	return Translation;
}

void PX4FLOW::reset(){
	Translation.setZero();
	prevZ = 0;
	float r[2] = {0.001, -1};
	delete flowKalman[0];
	delete flowKalman[1];
	delete flowKalman[2];
	flowKalman[0] = new Kalman(0.0001f, r, 0, 1);
	flowKalman[1] = new Kalman(0.0001f, r, 0, 1);
	flowKalman[2] = new Kalman(0.0001f, r, 0, 1);
//	Vector3f x;
//	x.setZero();
//	Matrix3d p;
//	p.setIdentity();
//	Matrix3d q;
//	q.setIdentity();
//	q*=0.0001;
//	Matrix3d r;
//	r.setIdentity();
//	r*=0.001;
//	delete flowUKF;
//	flowUKF = new UKF(x,p,q,r);
}

