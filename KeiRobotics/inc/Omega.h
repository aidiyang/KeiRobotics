/*
 * Omega.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef OMEGA_H_
#define OMEGA_H_

#include <MPU6050.h>
#include <MPU6500.h>
#include <Kalman.h>
#include <MovingWindowAverageFilter.h>
#include <Eigen/Eigen>
using Eigen::Vector3f;
using Eigen::Matrix3f;
using namespace Math;

namespace Sensors{
	class MPU6050;
	class MPU6500;
};

namespace Math{
	class MovingWindowAverageFilter;
	class Kalman;
};

using namespace Math;

namespace Inertia{

	class Omega{

		public:
			Omega(Sensors::MPU6050* mMPU6050);
			Omega(Sensors::MPU6500* mMPU6500);
			void Update();
			Vector3f getOmega();
			Vector3f getFilteredOmega();
			void setOmega(Vector3f value);
			bool getIsValided();
			bool isMPU6500;
			MovingWindowAverageFilter* mOmegaMovingWindowAverageFilter[3];
			Kalman* OmegaKalman;

		private:
			Sensors::MPU6050* _mMPU6050;
			Sensors::MPU6500* _mMPU6500;
			bool isValided;
			Vector3f _Omega;
			Vector3f _FilteredOmega;
	};
};

#endif /* OMEGA_H_ */
