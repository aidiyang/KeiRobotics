/*
 * Localization.h
 *
 *  Created on: 2016¦~2¤ë26¤é
 *      Author: wongy
 */

#ifndef LOCALIZATION_H_
#define LOCALIZATION_H_

#include <Quaternion.h>
#include <Encoder.h>
#include <math.h>
#include <MathTools.h>
#include <Eigen/Eigen>

using Eigen::Matrix3f;
using Eigen::Vector3f;
using namespace Math;
using namespace Sensors;

namespace Sensors{
	class Encoder;
};

namespace Math{
	class Localization{
		public:
			Localization(Quaternion* quaternion, Encoder* encoderX, Encoder* encoderY, float encoderXTranslation, float encoderYTranslation);
			Vector3f getPos();
			void setPos(Vector3f value);
			Vector3f getVel();
			void setEncoderXTranslation(float value);
			void setEncoderYTranslation(float value);
			void LocalizationCalc();
			void Reset();
		private:
			Quaternion* mQuaternion;
			Encoder* mEncoderX;
			Encoder* mEncoderY;
			float Interval;
			float EncoderXFramePosX;
			float EncoderXFramePosY;
			float PrevProjX;
			float PrevProjY;
			Vector3f Pos;
			Vector3f Vel;
			float EncoderXTranslation;
			float EncoderYTranslation;
			float PrevTick;
	};
}

#endif /* LOCALIZATION_H_ */
