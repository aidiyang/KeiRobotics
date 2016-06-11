/*
 * MPU6050.h
 *
 *  Created on: 2014¦~8¤ë23¤é
 *      Author: YunKei
 */

#ifndef  MPU6050_H_
#define  MPU6050_H_

#include <App.h>
#include <I2C.h>
#include <Acceleration.h>
#include <math.h>
#include <Eigen/Eigen>
using Eigen::Vector3f;

namespace Sensors{

	class MPU6050{
		public:
			enum ADDR{ADDRESS = 0x68};
			enum REG{RA_XG_OFFS_TC = 0x00,
					 RA_YG_OFFS_TC = 0x01,
					 RA_ZG_OFFS_TC = 0x02,
					 RA_X_FINE_GAIN = 0x03,
					 RA_Y_FINE_GAIN = 0x04,
					 RA_Z_FINE_GAIN = 0x05,
					 RA_XA_OFFS_H = 0x06,
					 RA_XA_OFFS_L_TC = 0x07,
					 RA_YA_OFFS_H = 0x08,
					 RA_YA_OFFS_L_TC = 0x09,
					 RA_ZA_OFFS_H = 0x0A,
					 RA_ZA_OFFS_L_TC = 0x0B,
					 RA_XG_OFFS_USRH = 0x13,
					 RA_XG_OFFS_USRL = 0x14,
					 RA_YG_OFFS_USRH = 0x15,
					 RA_YG_OFFS_USRL = 0x16,
					 RA_ZG_OFFS_USRH = 0x17,
					 RA_ZG_OFFS_USRL = 0x18,
					 RA_SMPLRT_DIV = 0x19,
					 RA_CONFIG = 0x1A,
					 RA_GYRO_CONFIG = 0x1B,
					 RA_ACCEL_CONFIG = 0x1C,
					 RA_FF_THR = 0x1D,
					 RA_FF_DUR = 0x1E,
					 RA_MOT_THR = 0x1F,
					 RA_MOT_DUR = 0x20,
					 RA_ZRMOT_THR = 0x21,
					 RA_ZRMOT_DUR = 0x22,
					 RA_FIFO_EN = 0x23,
					 RA_I2C_MST_CTRL = 0x24,
					 RA_I2C_SLV0_ADDR = 0x25,
					 RA_I2C_SLV0_REG = 0x26,
					 RA_I2C_SLV0_CTRL = 0x27,
					 RA_I2C_SLV1_ADDR = 0x28,
					 RA_I2C_SLV1_REG = 0x29,
					 RA_I2C_SLV1_CTRL = 0x2A,
					 RA_I2C_SLV2_ADDR = 0x2B,
					 RA_I2C_SLV2_REG = 0x2C,
					 RA_I2C_SLV2_CTRL = 0x2D,
					 RA_I2C_SLV3_ADDR = 0x2E,
					 RA_I2C_SLV3_REG = 0x2F,
					 RA_I2C_SLV3_CTRL = 0x30,
					 RA_I2C_SLV4_ADDR = 0x31,
					 RA_I2C_SLV4_REG = 0x32,
					 RA_I2C_SLV4_DO = 0x33,
					 RA_I2C_SLV4_CTRL = 0x34,
					 RA_I2C_SLV4_DI = 0x35,
					 RA_I2C_MST_STATUS = 0x36,
					 RA_INT_PIN_CFG = 0x37,
					 RA_INT_ENABLE = 0x38,
					 RA_DMP_INT_STATUS = 0x39,
					 RA_INT_STATUS = 0x3A,
					 RA_ACCEL_XOUT_H = 0x3B,
					 RA_ACCEL_XOUT_L = 0x3C,
					 RA_ACCEL_YOUT_H = 0x3D,
					 RA_ACCEL_YOUT_L = 0x3E,
					 RA_ACCEL_ZOUT_H = 0x3F,
					 RA_ACCEL_ZOUT_L = 0x40,
					 RA_TEMP_OUT_H = 0x41,
					 RA_TEMP_OUT_L = 0x42,
					 RA_GYRO_XOUT_H = 0x43,
					 RA_GYRO_XOUT_L = 0x44,
					 RA_GYRO_YOUT_H = 0x45,
					 RA_GYRO_YOUT_L = 0x46,
					 RA_GYRO_ZOUT_H = 0x47,
					 RA_GYRO_ZOUT_L = 0x48,
					 RA_EXT_SENS_DATA_00 = 0x49,
					 RA_EXT_SENS_DATA_01 = 0x4A,
					 RA_EXT_SENS_DATA_02 = 0x4B,
					 RA_EXT_SENS_DATA_03 = 0x4C,
					 RA_EXT_SENS_DATA_04 = 0x4D,
					 RA_EXT_SENS_DATA_05 = 0x4E,
					 RA_EXT_SENS_DATA_06 = 0x4F,
					 RA_EXT_SENS_DATA_07 = 0x50,
					 RA_EXT_SENS_DATA_08 = 0x51,
					 RA_EXT_SENS_DATA_09 = 0x52,
				  	 RA_EXT_SENS_DATA_10 = 0x53,
				  	 RA_EXT_SENS_DATA_11 = 0x54,
				  	 RA_EXT_SENS_DATA_12 = 0x55,
				  	 RA_EXT_SENS_DATA_13 = 0x56,
				  	 RA_EXT_SENS_DATA_14 = 0x57,
				  	 RA_EXT_SENS_DATA_15 = 0x58,
				  	 RA_EXT_SENS_DATA_16 = 0x59,
				  	 RA_EXT_SENS_DATA_17 = 0x5A,
				  	 RA_EXT_SENS_DATA_18 = 0x5B,
				  	 RA_EXT_SENS_DATA_19 = 0x5C,
				  	 RA_EXT_SENS_DATA_20 = 0x5D,
				  	 RA_EXT_SENS_DATA_21 = 0x5E,
				  	 RA_EXT_SENS_DATA_22 = 0x5F,
				  	 RA_EXT_SENS_DATA_23 = 0x60,
				  	 RA_MOT_DETECT_STATUS = 0x61,
				  	 RA_I2C_SLV0_DO = 0x63,
				  	 RA_I2C_SLV1_DO = 0x64,
				  	 RA_I2C_SLV2_DO = 0x65,
				  	 RA_I2C_SLV3_DO = 0x66,
				  	 RA_I2C_MST_DELAY_CTRL = 0x67,
				  	 RA_SIGNAL_PATH_RESET = 0x68,
				  	 RA_MOT_DETECT_CTRL = 0x69,
				  	 RA_USER_CTRL = 0x6A,
				  	 RA_PWR_MGMT_1 = 0x6B,
				  	 RA_PWR_MGMT_2 = 0x6C,
				  	 RA_BANK_SEL = 0x6D,
				  	 RA_MEM_START_ADDR = 0x6E,
				  	 RA_MEM_R_W = 0x6F,
				  	 RA_DMP_CFG_1 = 0x70,
				  	 RA_DMP_CFG_2 = 0x71,
				  	 RA_FIFO_COUNTH = 0x72,
				  	 RA_FIFO_COUNTL = 0x73,
				  	 RA_FIFO_R_W = 0x74,
				  	 RA_WHO_AM_I = 0x75};

			MPU6050(Communication::I2C* i2c);
			void setI2CBypass(bool onState);
			bool Update();
			void setTemperature(float value);
			float getTemperature();
			void setRawAcc(Vector3f value);
			Vector3f getRawAcc();
			void setRawAccOffset(Vector3f value);
			Vector3f getRawAccOffset();
			void setRawAccScale(Vector3f value);
			Vector3f getRawAccScale();
			void setRawOmega(Vector3f value);
			Vector3f getRawOmega();
			void setRawOmegaOffset(Vector3f value);
			Vector3f getRawOmegaOffset();
			bool getIsValided();
			Communication::I2C* i2cx;

		private:

			static float RawAccPosX;
			static float RawAccNegX;
			static float RawAccPosY;
			static float RawAccNegY;
			static float RawAccPosZ;
			static float RawAccNegZ;

			bool isValided;
			Vector3f RawAcc;
			Vector3f RawAccOffset;
			Vector3f RawAccScale;
			Vector3f RawOmega;
			Vector3f RawOmegaScale;
			Vector3f RawOmegaOffset;
			float temperature;
			void FastInitialization();
	};
};

#endif /*  MPU6050_H_ */
