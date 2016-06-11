/*
 * Omega.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef OMEGA_H_
#define OMEGA_H_

#include <MPU6050.h>
#include <Eigen/Eigen>
using Eigen::Vector3f;

namespace Sensors{
	class MPU6050;
};

namespace Inertia{

	class Omega{

		public:
			Omega(Sensors::MPU6050* mMPU6050);
			void Update();
			Vector3f getOmega();
			void setOmega(Vector3f value);
			bool getIsValided();

		private:
			Sensors::MPU6050* _mMPU6050;
			bool isValided;
			Vector3f _Omega;
	};
};

#endif /* OMEGA_H_ */
