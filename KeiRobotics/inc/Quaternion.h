/*
 * Quaternion.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include <Kalman.h>
#include <Omega.h>
#include <Acceleration.h>
#include <Compass.h>
#include <AdditionalTools.h>
#include <EncoderYaw.h>
#include <Eigen/Eigen>
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Vector3f;
using Eigen::Vector4f;

using namespace Inertia;

namespace Inertia{
	class Acceleration;
	class Omega;
	class Compass;
	class EncoderYaw;
};

namespace Inertia{
};

namespace Math{

	class Quaternion{

		public:
			Quaternion(Acceleration* mAcceleration, Omega* mOmega);
			Quaternion(Acceleration* mAcceleration, Omega* mOmega, Compass* mCompass);
			Quaternion(Acceleration* mAcceleration, Omega* mOmega, EncoderYaw* mEncoderYaw);
			bool Update();
			Vector3f getEuler();
			void setEuler(Vector3f angle);
			void Reset();
			Vector4f _Quaternion;

		private:
			Acceleration* _mAcceleration;
			Omega* _mOmega;
			Compass* _mCompass;
			EncoderYaw* _mEncoderYaw;
			float Interval;
			Vector3f _Euler;
			Kalman* _QuaternionKalman;

			bool Valid;
			bool IsUseCompass;
			bool IsUseEncoderYaw;
			Vector4f PrevT;
			Vector3f QuaternionToEuler(Vector4f q);
			Matrix3f QuaternionToMatrix(Vector4f q);
			Vector4f EulerToQuaternion(Vector3f euler);
			Matrix3f EulerToMatrix(Vector3f euler);
			Vector4f FixedAnglesToQuaternion(Vector3f fixedAngles);
			Vector3f MatrixToFixedAngles(Matrix3f R);
			Matrix4f calcStateTransMatrix(Vector3f w, float t);
			Eigen::Matrix<float, 3, 4> calcQuatToEulerMeasMatrix(Vector4f q);
			float PrevTick;
	};
};

#endif /* QUATERNION_H_ */
