/*
 * Localization.cpp
 *
 *  Created on: 2016¦~2¤ë26¤é
 *      Author: wongy
 */

#include <Localization.h>

Localization::Localization(Quaternion* quaternion,
		Encoder* encoderX,
		Encoder* encoderY,
		float  encoderYTranslation,
		float  encoderXTranslation) : mQuaternion(quaternion),
		mEncoderX(encoderX),
		mEncoderY(encoderY),
		EncoderXTranslation(encoderXTranslation),
		EncoderYTranslation(encoderYTranslation),
		Interval(0), EncoderXFramePosX(encoderXTranslation), PrevProjX(0), EncoderXFramePosY(encoderYTranslation), PrevProjY(0){
	Pos.setZero();
	Vel.setZero();
	PrevTick = App::mApp->mTicks->getTicks();

}
void Localization::setPos(Vector3f value){
	EncoderXFramePosX = value[0];
	EncoderXFramePosY = value[1];
	PrevProjX = 0;
	PrevProjY = 0;
	Pos = value;
}

Vector3f Localization::getPos(){
	return Pos;
}

Vector3f Localization::getVel(){
	return Vel;
}

void Localization::LocalizationCalc(){
	float yaw = mQuaternion->getEuler()[2];
	float sinyaw = sinf(yaw);
	float cosyaw = cosf(yaw);
	float x = mEncoderX->getVel();
	float y = mEncoderY->getVel();
	Interval = App::mApp->mTicks->getTicks() - PrevTick;
	PrevTick = App::mApp->mTicks->getTicks();
	Interval /= 1000.0f;
	if(Interval <= 0){
		return;
	}
	float ProjX = (cosyaw * x - sinyaw * y);
	float ProjY = (sinyaw * x + cosyaw * y);
	if(ProjX == ProjX){
		EncoderXFramePosX += 0.5f * (PrevProjX + ProjX) * Interval;
		PrevProjX = ProjX;
	}
	if(ProjY == ProjY){
		EncoderXFramePosY += 0.5f * (PrevProjY + ProjY) * Interval;
		PrevProjY = ProjY;
	}
	Vector3f t;
	t << EncoderXFramePosX - cosyaw * EncoderXTranslation + sinyaw * EncoderYTranslation , EncoderXFramePosY - sinyaw * EncoderXTranslation - cosyaw * EncoderYTranslation, 0;
	Vel = (t - Pos) / Interval;
	Pos = t;
}

void Localization::setEncoderXTranslation(float value){
	EncoderXTranslation = value;
}

void Localization::setEncoderYTranslation(float value){
	EncoderYTranslation = value;
}

void Localization::Reset(){
	Pos.setZero();
	Vel.setZero();
	EncoderXFramePosX = EncoderXTranslation;
	EncoderXFramePosY = EncoderYTranslation;
	PrevProjX = 0;
	PrevProjY = 0;
}
