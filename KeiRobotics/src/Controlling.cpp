/*
 * Controlling.cpp
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#include <Controlling.h>
#include <App.h>
#include <stdio.h>
#include <AdditionalTools.h>

using Eigen::Vector3f;

using namespace Math;
using namespace Utility;

void StartingTask(Bundle* bundle){

	if(App::mApp->mControlling->getStarting()){

//		if(App::mApp->mControlling->startCount == 0){
//			App::mApp->mControlling->_mPWM->Control1(App::mApp->mControlling->initLift);
//			App::mApp->mControlling->_mPWM->Control2(App::mApp->mControlling->initLift);
//			App::mApp->mControlling->_mPWM->Control3(App::mApp->mControlling->initLift);
//			App::mApp->mControlling->_mPWM->Control4(App::mApp->mControlling->initLift);
//		}
//		else if(App::mApp->mControlling->startCount >= 15){
			App::mApp->mControlling->setStart(true);
			App::mApp->mCommunicating1->Acknowledgement();
//		}
//		else if(App::mApp->mControlling->startCount >= 40){
			App::mApp->mControlling->setStarting(false);
//		}
//		App::mApp->mControlling->startCount++;
	}
}

void StoppingTask(Bundle* bundle){
	if(App::mApp->mControlling->getStopping()){
		if(App::mApp->mControlling->Lift > App::mApp->mControlling->landingLift){
			App::mApp->mControlling->Lift = App::mApp->mControlling->Lift - 500;
		}
		else if(App::mApp->mControlling->StoppingDelayCount < 25){
			App::mApp->mControlling->StoppingDelayCount++;
		}
		else{
			App::mApp->mControlling->StoppingDelayCount = 0;
			App::mApp->mControlling->StopAllMotors();
			App::mApp->mControlling->setStart(false);

			App::mApp->mControlling->Motor1PWM = 0;
			App::mApp->mControlling->Motor2PWM = 0;
			App::mApp->mControlling->Motor3PWM = 0;
			App::mApp->mControlling->Motor4PWM = 0;
//			App::mApp->mControlling->Motor1SpeedTarget = 0;
//			App::mApp->mControlling->Motor2SpeedTarget = 0;
//			App::mApp->mControlling->Motor3SpeedTarget = 0;
//			App::mApp->mControlling->Motor4SpeedTarget = 0;
//			App::mApp->mControlling->Motor1->clear();
//			App::mApp->mControlling->Motor2->clear();
//			App::mApp->mControlling->Motor3->clear();
//			App::mApp->mControlling->Motor4->clear();
			App::mApp->mControlling->RollPid->clear();
			App::mApp->mControlling->PitchPid->clear();
			App::mApp->mControlling->YawPid->clear();
			App::mApp->mControlling->KdRollPid->clear();
			App::mApp->mControlling->KdPitchPid->clear();
			App::mApp->mControlling->KdYawPid->clear();
			App::mApp->mControlling->Lift = 0;
			App::mApp->mControlling->setStopping(false);
		}
		App::mApp->mCommunicating1->Acknowledgement();
	}
}
Controlling::Controlling(PWM* mPWM, Encoder* encoder1, Encoder* encoder2, Encoder* encoder3) : _mPWM(mPWM), WatchDogLimit(800), minLift(7000), maxLift(9500),
		Encoder1(encoder1), Encoder2(encoder2),Encoder3(encoder3),
//Controlling::Controlling(PWM* mPWM) : _mPWM(mPWM), WatchDogLimit(800), landingLift(5000), minLift(4000), maxLift(10000),
		Motor1PWM(0), Motor2PWM(0), Motor3PWM(0), Motor4PWM(0),
		started(false), starting(false), stopping(false),
		watchDogCount(0), Lift(0.0f), RollTarget(0),
		PitchTarget(0), RollOffset(0),
		PitchOffset(0), YawOffset(0), initLift(5000), startCount(0), StoppingDelayCount(0),
		Speed(1.0f), Motor1SpeedTarget(0), Motor2SpeedTarget(0), Motor3SpeedTarget(0), Motor4SpeedTarget(0),
		XPosTarget(0), YPosTarget(0), YawTarget(0), ManualMode(false), IsSonicDriveYaw(false){

	RollPid = new Pid(0.0f,0.0f,0.0,10000.0f);
	PitchPid = new Pid(10000.0f,0.0f,0.0,10000.0f);
	YawPid = new Pid(0.0f,0.0f,0.0,10000.0f);

	KdRollPid = new Pid(0.0f,0.0f,0.0,10000.0f);
	KdPitchPid = new Pid(50.0f,0.0f,0.0,10000.0f);
	KdYawPid = new Pid(0.0f,0.0f,0.0,10000.0f);

	Motor1 = new Pid(600.0f,0.0f,0.5,10000.0f);
	Motor2 = new Pid(600.0f,0.0f,0.5,10000.0f);
	Motor3 = new Pid(600.0f,0.0f,0.5,10000.0f);
	Motor4 = new Pid(600.0f,0.0f,0.5,10000.0f);

	App::mApp->mTask->Attach(40, StartingTask, "StartingTask", true);
	App::mApp->mTask->Attach(40, StoppingTask, "StoppingTask", true);
}

void Controlling::setStarting(bool value){
	starting = value;
}

bool Controlling::getStarting(){
	return starting;
}
void Controlling::setStopping(bool value){
	stopping = value;
}

void Controlling::ControllingPoll(){

//	if(started){
//
//		if(watchDogCount < WatchDogLimit){
//			//watchDogCount++;
//		}
//		float errRoll = 0;
//		float errPitch = 0;
//		float errYaw = 0;
//		float cosRollcosPitch = 0;
//		bool failed = false;
//		Vector3f euler = App::mApp->mQuaternion->getEuler();
//		Vector3f omega = App::mApp->mOmega->getOmega();
//		if(euler[0] == euler[0] &&
//			euler[1] == euler[1] &&
//			euler[2] == euler[2] &&
//			omega[0] == omega[0] &&
//			omega[1] == omega[1] &&
//			omega[2] == omega[2]){
//			errRoll = RollPid->pid(MathTools::DegreeToRadian(RollTarget), App::mApp->mQuaternion->getEuler()[0] - MathTools::DegreeToRadian(RollOffset)) + KdRollPid->pid(0, App::mApp->mOmega->getOmega()[0]);
//			errPitch = PitchPid->pid(MathTools::DegreeToRadian(PitchTarget), App::mApp->mQuaternion->getEuler()[1] - MathTools::DegreeToRadian(PitchOffset)) + KdPitchPid->pid(0, App::mApp->mOmega->getOmega()[1]);
//			errYaw = YawPid->pid(MathTools::DegreeToRadian(YawTarget), App::mApp->mQuaternion->getEuler()[2] - MathTools::DegreeToRadian(YawOffset)) + KdYawPid->pid(0, App::mApp->mOmega->getOmega()[2]);
//
//			cosRollcosPitch = cosf(App::mApp->mQuaternion->getEuler()[0] - MathTools::DegreeToRadian(RollOffset)) * cosf(App::mApp->mQuaternion->getEuler()[1] - MathTools::DegreeToRadian(PitchOffset));
//		}else{
//			failed = true;
//		}
//		if(!failed && errRoll == errRoll &&
//			errPitch == errPitch &&
//			errYaw == errYaw &&
//			cosRollcosPitch == cosRollcosPitch){
//			Motor1PWM = Lift / cosRollcosPitch + errRoll - errPitch + errYaw;
//			Motor2PWM = Lift / cosRollcosPitch - errRoll - errPitch - errYaw;
//			Motor3PWM = Lift / cosRollcosPitch - errRoll + errPitch + errYaw;
//			Motor4PWM = Lift / cosRollcosPitch + errRoll + errPitch - errYaw;
//			if(stopping){
//				Motor1PWM = MathTools::Trim(landingLift, Motor1PWM, maxLift);
//				Motor2PWM = MathTools::Trim(landingLift, Motor2PWM, maxLift);
//				Motor3PWM = MathTools::Trim(landingLift, Motor3PWM, maxLift);
//				Motor4PWM = MathTools::Trim(landingLift, Motor4PWM, maxLift);
//			}
//			else{
//				Motor1PWM = MathTools::Trim(minLift, Motor1PWM, maxLift);
//				Motor2PWM = MathTools::Trim(minLift, Motor2PWM, maxLift);
//				Motor3PWM = MathTools::Trim(minLift, Motor3PWM, maxLift);
//				Motor4PWM = MathTools::Trim(minLift, Motor4PWM, maxLift);
//			}
//			_mPWM->Control1(Motor1PWM);
//			_mPWM->Control2(Motor2PWM);
//			_mPWM->Control3(Motor3PWM);
//			_mPWM->Control4(Motor4PWM);
//		}else{
//			failed = true;
//		}
//		if(failed){
//			return;
//		}
//	}
//	if(watchDogCount >= WatchDogLimit){ //||
////	   fabsf(RollTarget + RollOffset - MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[0])) > 25.0f ||
////	   fabsf(PitchTarget + PitchOffset - MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[1])) > 25.0f){
//		if(started){
//			Stopping();
//			App::mApp->mCommunicating1->Acknowledgement();
//		}
//	}

	if(started){


//			if(watchDogCount < WatchDogLimit){
//				//watchDogCount++;
//			}
//
//	//		if(!ManualMode){
//			if(!ManualMode && App::mApp->trigger){																	//Run in Auto mode
//				if(XPosTarget == XPosTarget && YPosTarget == YPosTarget &&											// Check NAN
//						App::mApp->mLocalization->getPos()[0] == App::mApp->mLocalization->getPos()[0] &&			// Check NAN
//						App::mApp->mLocalization->getPos()[1] == App::mApp->mLocalization->getPos()[1])				// Check NAN
//					{
//						float errX = XPosPid->pid(XPosTarget, App::mApp->mLocalization->getPos()[0]);				// get X PID
//						float errY = YPosPid->pid(YPosTarget, App::mApp->mLocalization->getPos()[1]);				// get Y PID
//						if(fabs(YawTarget - App::mApp->mQuaternion->getEuler()[2]) > 3 * MathTools::PI / 2)			// get Yaw data
//						{																							// by use MPU6050 data in normal running
//							if(YawTarget < 0){																		// convert to 0-360 degree
//									YawTarget += 2*MathTools::PI;
//							}else{
//									YawTarget -= 2*MathTools::PI;
//							}
//						}
//						float errYaw = YawPid->pid(YawTarget, App::mApp->mQuaternion->getEuler()[2]);				// get Yaw PID
//						float dirAngle;
//						dirAngle = atan2f(1000*errY,1000*errX) - App::mApp->mQuaternion->getEuler()[2];
//						if(fabs(dirAngle) > MathTools::PI)
//						{
//							if(dirAngle < 0){
//								dirAngle += 2*MathTools::PI;
//							}else{
//								dirAngle -= 2*MathTools::PI;
//							}
//						}
//
//						// get turn direction of Yaw
//						if(IsSonicDriveYaw){																		// get Yaw by Sonic Data
//							errYaw = SonicPid->pid(0,App::mApp->mSonic1->Distance - App::mApp->mSonic2->Distance);
//							errYaw = errYaw > MathTools::PI / 6 ? MathTools::PI / 6 :
//							errYaw < -MathTools::PI / 6 ? -MathTools::PI / 6 : errYaw;
//							// dirAngle ????
//						}
//						if(dirAngle == dirAngle && errYaw == errYaw){
//							Move(Speed, dirAngle, errYaw);
//						}
//						else{
//							XPosPid->clear();
//							YPosPid->clear();
//							YawPid->clear();
//							return;
//						}
//					}
//			}
//	//		float errRoll = RollPid->pid(MathTools::DegreeToRadian(RollTarget), App::mApp->mQuaternion->getEuler()[0] - MathTools::DegreeToRadian(RollOffset)) + KdRollPid->pid(0, App::mApp->mOmega->getOmega()[0]);
//	//		float errPitch = PitchPid->pid(MathTools::DegreeToRadian(PitchTarget), App::mApp->mQuaternion->getEuler()[1] - MathTools::DegreeToRadian(PitchOffset)) +  + KdPitchPid->pid(0, App::mApp->mOmega->getOmega()[1]);
//	//		float errYaw = YawPid->pid(MathTools::DegreeToRadian(YawTarget), App::mApp->mQuaternion->getEuler()[2] - MathTools::DegreeToRadian(YawOffset)) +  + KdYawPid->pid(0, App::mApp->mOmega->getOmega()[2]);
//	//
//	//		float cosRollcosPitch = cosf(App::mApp->mQuaternion->getEuler()[0] - MathTools::DegreeToRadian(RollOffset)) * cosf(App::mApp->mQuaternion->getEuler()[1] - MathTools::DegreeToRadian(PitchOffset));
//	//
//	//		float Motor1PWM = Lift / cosRollcosPitch + errRoll - errPitch + errYaw;
//	//		float Motor2PWM = Lift / cosRollcosPitch - errRoll - errPitch - errYaw;
//	//		float Motor3PWM = Lift / cosRollcosPitch - errRoll + errPitch + errYaw;
//	//		float Motor4PWM = Lift / cosRollcosPitch + errRoll + errPitch - errYaw;
//	//		if(stopping){
//	//			Motor1PWM = MathTools::Trim(leadingLift, Motor1PWM, maxLift);
//	//			Motor2PWM = MathTools::Trim(leadingLift, Motor2PWM, maxLift);
//	//			Motor3PWM = MathTools::Trim(leadingLift, Motor3PWM, maxLift);
//	//			Motor4PWM = MathTools::Trim(leadingLift, Motor4PWM, maxLift);
//	//		}
//	//		else{
//	//			Motor1PWM = MathTools::Trim(minLift, Motor1PWM, maxLift);
//	//			Motor2PWM = MathTools::Trim(minLift, Motor2PWM, maxLift);
//	//			Motor3PWM = MathTools::Trim(minLift, Motor3PWM, maxLift);
//	//			Motor4PWM = MathTools::Trim(minLift, Motor4PWM, maxLift);
//	//		}
//			if(!ManualMode && !App::mApp->trigger){											//Auto Mode without trigger --> Stop
//				Pause();
//			}
			//###########################################################################	//Run this ever Cycle
			float pwm1 = Motor1->pid(Motor1SpeedTarget, Encoder1->getVel());
			float pwm2 = Motor2->pid(Motor2SpeedTarget, Encoder2->getVel());
			float pwm3 = Motor3->pid(Motor3SpeedTarget, Encoder3->getVel());
			if(pwm1 == pwm1 && pwm2 == pwm2 && pwm3 == pwm3){
	//
					Motor1PWM += pwm1;
					Motor2PWM += pwm2;
					Motor3PWM += pwm3;
	//
	//				Motor1PWM = pwm1;
	//				Motor2PWM = pwm2;
	//				Motor3PWM = pwm3;
	//				Motor4PWM = pwm4;
	//			}
				Motor1PWM = Motor1PWM <= -10000.0f ? -10000.0f : Motor1PWM  >= 10000.0f ? 10000.0f : Motor1PWM;		//Limit PWM less then 10000.0f
				Motor2PWM = Motor2PWM <= -10000.0f ? -10000.0f : Motor2PWM  >= 10000.0f ? 10000.0f : Motor2PWM;
				Motor3PWM = Motor3PWM <= -10000.0f ? -10000.0f : Motor3PWM  >= 10000.0f ? 10000.0f : Motor3PWM;
				if(Motor1PWM < 0.0f){
					App::mApp->mGPIO1->GPIOControl(false);
				}
				else{
					App::mApp->mGPIO1->GPIOControl(true);
				}
				if(Motor2PWM < 0.0f){
					App::mApp->mGPIO3->GPIOControl(false);
				}
				else{
					App::mApp->mGPIO3->GPIOControl(true);
				}
				if(Motor3PWM < 0.0f){
					App::mApp->mGPIO4->GPIOControl(false);
				}
				else{
					App::mApp->mGPIO4->GPIOControl(true);
				}

				_mPWM->Control1(fabs(Motor1PWM));
				_mPWM->Control3(fabs(Motor2PWM));
				_mPWM->Control4(fabs(Motor3PWM));
			}

	//		float integral1 = RollPid->getIntegral();
	//		float integral2 = PitchPid->getIntegral();
	//		float integral3 = YawPid->getIntegral();
	//
	//		if(App::mApp->mCommunicating1->PrintType == 1){
	//			AdditionalTools::setBuffer(0, &integral1, 1);
	//			AdditionalTools::setBuffer(1, &integral2, 1);
	//			AdditionalTools::setBuffer(2, &integral3, 1);
	//		}
		}

		if(watchDogCount >= WatchDogLimit){// ||
	//	   fabsf(RollTarget + RollOffset - MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[0])) > 25.0f ||
	//	   fabsf(PitchTarget + PitchOffset - MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[1])) > 25.0f){
			if(started){
				Stopping();
				App::mApp->mCommunicating1->Acknowledgement();
			}
		}

}

void Controlling::Starting(){

//	if(4096 * 0.52 / App::mApp->mADC->getReading() > 3.8f){
//		if(!started){
//			startCount = 0;
			stopping = false;
			starting = true;
//		}
//		else{
//			StopAllMotors();
//			startCount = 0;
//			started = false;
//			stopping = false;
//			starting = false;
//		}
//	}
//	else{
		App::mApp->mCommunicating1->Acknowledgement();
//	}
}

bool Controlling::getStopping(){
	return stopping;
}

void Controlling::setStart(bool value){
	started = value;
}
bool Controlling::getStart(){
	return started;
}

void Controlling::Stopping(){
	if(!stopping){
		stopping = true;
		starting = false;
	}
}

float Controlling::getRollTarget(){
	return RollTarget;
}

float Controlling::getPitchTarget(){
	return PitchTarget;
}

float Controlling::getYawTarget(){
	return YawTarget;
}

void Controlling::setRollTarget(float roll){
	RollTarget = roll;
}

void Controlling::setPitchTarget(float pitch){
	PitchTarget = pitch;
}

void Controlling::setYawTarget(float yaw){
	YawTarget = yaw;
}

float Controlling::getRollOffset(){
	return RollOffset;
}

float Controlling::getPitchOffset(){
	return PitchOffset;
}

float Controlling::getYawOffset(){
	return YawOffset;
}

void Controlling::setRollOffset(float roll){
	RollOffset = roll;
}

void Controlling::setPitchOffset(float pitch){
	PitchOffset = pitch;
}

void Controlling::setYawOffset(float yaw){
	YawOffset = yaw;
}

void Controlling::clearWatchDogCount(){
	watchDogCount = 0;
}

void Controlling::Pause(){
	Motor1SpeedTarget = 0;
	Motor2SpeedTarget = 0;
	Motor3SpeedTarget = 0;
	Motor4SpeedTarget = 0;
}

void Controlling::StopAllMotors(){
	_mPWM->Control1(0);
	_mPWM->Control2(0);
	_mPWM->Control3(0);
	_mPWM->Control4(0);
}

void Controlling::Forward(){
	Move(Speed, MathTools::PI / 2, 0);
//	Motor1SpeedTarget = Speed;
//	Motor2SpeedTarget = Speed;
//	Motor3SpeedTarget = Speed;
//	Motor4SpeedTarget = Speed;
}

void Controlling::Backward(){
	Move(Speed, 3 * MathTools::PI / 2, 0);
//	Motor1SpeedTarget = -Speed;
//	Motor2SpeedTarget = -Speed;
//	Motor3SpeedTarget = -Speed;
//	Motor4SpeedTarget = -Speed;
}

void Controlling::Left(){
	Move(Speed, MathTools::PI, 0);
//	Motor1SpeedTarget = -Speed;
//	Motor2SpeedTarget = Speed;
//	Motor3SpeedTarget = -Speed;
//	Motor4SpeedTarget = Speed;
}

void Controlling::Right(){
	Move(Speed, 0, 0);
//	Motor1SpeedTarget = Speed;
//	Motor2SpeedTarget = -Speed;
//	Motor3SpeedTarget = Speed;
//	Motor4SpeedTarget = -Speed;
}

void Controlling::CW(){
	Motor1SpeedTarget = Speed;
	Motor2SpeedTarget = Speed;
	Motor3SpeedTarget = Speed;
}

void Controlling::CCW(){
	Motor1SpeedTarget = -Speed;
	Motor2SpeedTarget = -Speed;
	Motor3SpeedTarget = -Speed;
}

void Controlling::Move(float vel, float dirAngle, float orientationAngle){
	float y = vel*sinf(dirAngle);
	float x = vel*cosf(dirAngle);
	Matrix3f A;
	A <<  1.000,  0.268, -0.268,
			-1.000,  0.268, -0.268,
			 0.000, -0.536, -0.464;
	Vector3f u;
	u << y,x,orientationAngle;
	Vector3f v = A * u;
	Motor1SpeedTarget = v[0];
	Motor2SpeedTarget = v[1];
	Motor3SpeedTarget = v[2];
}

void Controlling::MoveToTargetWithSonicDriveYaw(float speed, float x, float y){
	IsSonicDriveYaw = true;
	Speed = speed;
	YPosTarget = y;
	XPosTarget = x;
}

void Controlling::MoveToTarget(float speed, float x, float y, float yaw){
	IsSonicDriveYaw = false;
	Speed = speed;
	YPosTarget = y;
	XPosTarget = x;
	YawTarget = yaw;
}
