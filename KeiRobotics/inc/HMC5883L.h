/*
 * MPU6050.h
 *
 *  Created on: 2014¦~8¤ë23¤é
 *      Author: YunKei
 */

#ifndef  HMC5883L_H_
#define  HMC5883L_H_

#include <I2C.h>
#include <Task.h>
#include <stdio.h>
#include <math.h>
#include <MathTools.h>
#include <MPU6050.h>
#include <App.h>
#include <Eigen/Eigen>
using Eigen::Vector3f;

using namespace Math;

namespace Sensors{
	class MPU6050;
};

namespace Sensors{

	class HMC5883L{
		public:
			enum ADDR{ADDRESS = 0x1e};
			enum REG{CFG_REG_A = 0x00,
					 CFG_REG_B = 0x01,
					 MODE_REG = 0x02,
					 DATA_OUT_X_MSB = 0x03,
					 DATA_OUT_X_LSB = 0x04,
					 DATA_OUT_Z_MSB = 0x05,
					 DATA_OUT_Z_LSB = 0x06,
					 DATA_OUT_Y_MSB = 0x07,
					 DATA_OUT_Y_LSB = 0x08,
					 STATUS_REG = 0x09,
					 ID_REGA = 0x0A,
					 ID_REGB = 0x0B,
					 ID_REGC = 0x0c};

			HMC5883L(MPU6050* mMPU6050);
			bool Update();
			void setRawMag(Vector3f value);
			Vector3f getRawMag();
			bool getIsValided();
			void CalibrationPrint();
			void CalibrationResultPrint();

		private:
			MPU6050* _mMPU6050;
			Communication::I2C* i2cx;
			bool isValided;
			Vector3f RawMag;
			Vector3f RawMagScale;
			Vector3f RawMagOffset;
			static float RawMagPosX;
			static float RawMagNegX;
			static float RawMagPosY;
			static float RawMagNegY;
			static float RawMagPosZ;
			static float RawMagNegZ;
			Vector3f MaxMag;
			Vector3f MinMag;
			bool CompassCal();
			void FastInitialization();
	};
};

#endif /*  MPU6050_H_ */
