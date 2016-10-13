/*
 * Acceleration.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef ACCELERATION_H_
#define ACCELERATION_H_

#include <MPU6050.h>
#include <MPU6500.h>
#include <Kalman.h>
#include <math.h>
#include <MathTools.h>
#include <MovingWindowAverageFilter.h>
#include <Eigen/Eigen>
using Eigen::Vector3f;
using Eigen::Matrix3f;
using namespace Math;

namespace Math{
	class Kalman;
};

namespace Sensors{
	class MPU6050;
	class MPU6500;
};

namespace Inertia{

	class Acceleration{

		public:
			static float Gravity;
			Acceleration(Sensors::MPU6050* mMPU6050);
			Acceleration(Sensors::MPU6500* mMPU6500);
			void Update();
			Vector3f getAcc();
			Vector3f getAngle();
			Vector3f getFilteredAcc();
			Vector3f getFilteredAngle();
			void setAcc(Vector3f value);
			bool getIsValided();

		private:
			Sensors::MPU6050* _mMPU6050;
			Sensors::MPU6500* _mMPU6500;
			bool isValided;
			Vector3f Acc;
			Vector3f FilteredAcc;
			Kalman* AccKalman;
			bool isMPU6500;
	};
};

#endif /* ACCELERATION_H_ */
