/*
 * Acceleration.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef ACCELERATION_H_
#define ACCELERATION_H_

#include <MPU6050.h>
#include <math.h>
#include <MathTools.h>
#include <MovingWindowAverageFilter.h>
#include <Eigen/Eigen>
using Eigen::Vector3f;
using namespace Math;

namespace Sensors{
	class MPU6050;
};

namespace Inertia{

	class Acceleration{

		public:
			static float Gravity;
			Acceleration(Sensors::MPU6050* mMPU6050);
			void Update();
			Vector3f getAcc();
			Vector3f getFilteredAcc();
			void setAcc(Vector3f value);
			Vector3f getAngle();
			Vector3f getFilteredAngle();
			bool getIsValided();

		private:
			Sensors::MPU6050* _mMPU6050;
			bool isValided;
			Vector3f Acc;
			MovingWindowAverageFilter* mAccMovingWindowAverageFilter[3];
	};
};

#endif /* ACCELERATION_H_ */
