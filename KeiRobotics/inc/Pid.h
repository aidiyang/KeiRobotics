/*
 * Pid.h
 *
 *  Created on: 2014¦~11¤ë8¤é
 *      Author: YunKei
 */

#ifndef PID_H_
#define PID_H_

#include <Eigen/Eigen>

namespace Math{

	class Pid{

		public:

			Pid(float kp, float ki, float kd, float integralLimit);
			void setKp(float kp);
			void setKi(float ki);
			void setKd(float kd);
			float getKp();
			float getKi();
			float getKd();

			float pid(float target, float current);
			void clear();
			float getIntegral();

		private:

			float Kp, Ki, Kd;
			float Interval;
			float Integral;
			float IntegralLimit;
			float PreErr;
			float PrevTick;

	};
};

#endif /* PID_H_ */
