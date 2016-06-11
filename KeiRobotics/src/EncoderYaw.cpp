/*
 * EncoderYaw.cpp
 *
 *  Created on: 2016¦~3¤ë8¤é
 *      Author: wongy
 */

#include <EncoderYaw.h>
#include <App.h>

using namespace Inertia;

EncoderYaw::EncoderYaw(Encoder* encoder1, Encoder* encoder2, float radius) : Encoder1(encoder1), Encoder2(encoder2), Radius(radius), Interval(0), Yaw(0), PrevOmega(0){
	PrevTick = App::mApp->mTicks->getTicks();
}

void EncoderYaw::Update(){
	Interval = App::mApp->mTicks->getTicks() - PrevTick;
	PrevTick = App::mApp->mTicks->getTicks();
	Interval /= 1000.0f;
	if(Interval <= 0){
		return;
	}
	float omega = (Encoder2->getRawVel() - Encoder1->getRawVel()) / Radius;
	Yaw += 0.5f * (PrevOmega + omega) * Interval;
	PrevOmega = omega;
	Yaw = Yaw >= MathTools::PI ? Yaw - 2.0f * MathTools::PI :
		  Yaw <= -MathTools::PI ? Yaw + 2.0f * MathTools::PI : Yaw;
}

float EncoderYaw::getYaw(){
	return Yaw;
}

void EncoderYaw::Reset(){
	PrevOmega = 0;
	Yaw = 0;
}
