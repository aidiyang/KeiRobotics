/*
 * Controlling.h
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#ifndef CONTROLLING_H_
#define CONTROLLING_H_

#include <PWM.h>
#include <App.h>
#include <Communicating.h>
#include <Quaternion.h>
#include <math.h>
#include <MathTools.h>
#include <stdio.h>
#include <Pid.h>
#include <PWM.h>
#include <Omega.h>
#include <Task.h>
#include <Acceleration.h>
#include <AdditionalTools.h>
#include <Delay.h>
#include <Led.h>
#include <Encoder.h>
#include <Bundle.h>
#include <Eigen/Eigen>
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::Matrix4f;

using namespace Math;
using namespace Sensors;

namespace Control{

	class Controlling{
		public:
			Controlling(PWM* mPWM, Encoder* encoder1, Encoder* encoder2, Encoder* encoder3);
			void ControllingPoll();
			void Starting();
			void Stopping();
			float getRollTarget();
			float getPitchTarget();
			float getYawTarget();
			void setRollTarget(float roll);
			void setPitchTarget(float pitch);
			void setYawTarget(float yaw);
			float getRollOffset();
			float getPitchOffset();
			float getYawOffset();
			void setRollOffset(float roll);
			void setPitchOffset(float pitch);
			void setYawOffset(float yaw);
			void clearWatchDogCount();
			void setStart(bool);
			bool getStart();
			void setStarting(bool);
			bool getStarting();
			void setStopping(bool);
			bool getStopping();
			void StopAllMotors();
			void Forward();
			void Backward();
			void Left();
			void Right();
			void Pause();
			void CW();
			void CCW();
			void Move(float vel, float dirAngle, float orientationAngle);
			void MoveToTarget(float speed, float x, float y, float yaw);
			void MoveToTargetWithSonicDriveYaw(float speed, float x, float y);
			PWM* _mPWM;
			Encoder* Encoder1;
			Encoder* Encoder2;
			Encoder* Encoder3;
			Pid* RollPid;
			Pid* PitchPid;
			Pid* KdRollPid;
			Pid* KdPitchPid;
			Pid* KdYawPid;
			Pid* YPosPid;
			Pid* XPosPid;
			Pid* YawPid;
			Pid* SonicPid;
			Pid* Motor1;
			Pid* Motor2;
			Pid* Motor3;
			Pid* Motor4;
			float maxLift;
			float minLift;
			float initLift;
			float landingLift;
			float Lift;
			int watchDogCount;
			float RollOffset;
			float PitchOffset;
			float YawOffset;
			int startCount;
			int StoppingDelayCount;
			float Motor1PWM;
			float Motor2PWM;
			float Motor3PWM;
			float Motor4PWM;
			float Speed;
			float Motor1SpeedTarget;
			float Motor2SpeedTarget;
			float Motor3SpeedTarget;
			float Motor4SpeedTarget;
			float XPosTarget;
			float YPosTarget;
			float YawTarget;
			bool ManualMode;
			bool IsSonicDriveYaw;

		private:
			int WatchDogLimit;
			float RollTarget;
			float PitchTarget;
			bool started;
			bool starting;
			bool stopping;
	};
};

#endif /* CONTROLLING_H_ */
