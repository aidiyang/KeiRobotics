/*
 * Compass.h
 *
 *  Created on: 2015¦~12¤ë19¤é
 *      Author: wongy
 */

#ifndef COMPASS_H_
#define COMPASS_H_

#include <HMC5883L.h>
#include <Acceleration.h>
#include <math.h>
#include <MathTools.h>
#include <AdditionalTools.h>
#include <MovingWindowAverageFilter.h>
#include <Eigen/Eigen>
using Eigen::Vector3f;
using namespace Math;

namespace Sensors{
	class HMC5883L;
};

namespace Inertia{
	class Acceleration;
};

namespace Inertia{

	class Compass{

		public:
			Compass(Sensors::HMC5883L* mHMC5883L, Acceleration* mAcceleration);
			void Update();
			Vector3f getMag();
			Vector3f getFilteredMag();
			void setMag(Vector3f value);
			Vector3f getAngle();
			Vector3f getFilteredAngle();
			void Reset();
			bool getIsValided();

		private:
			Sensors::HMC5883L* _mHMC5883L;
			Acceleration* _mAcceleration;
			bool isValided;
			Vector3f Mag;
			float InitMagHead;
			MovingWindowAverageFilter* mMagMovingWindowAverageFilter[3];
	};
};

#endif /* COMPASS_H_ */
