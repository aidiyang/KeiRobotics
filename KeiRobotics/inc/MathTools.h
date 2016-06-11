/*
 * MathTools.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef MATHTOOLS_H_
#define MATHTOOLS_H_

#include <inttypes.h>

namespace Math{

	class MathTools{

		public:

			static float PI;
			static float DEGREE_PER_RADIAN;
			static float RADIAN_PER_DEGREE;
			static float Sqrt(float);
			static float Sign(float);
			static float DegreeToRadian(float);
			static float RadianToDegree(float);
			static float Trim(float lowerBound, float value, float upperBound);
			static float CutOff(float, float, float);
			static float QuadRoot(float);
			static float OctRoot(float);
			static float TrimResolution(float value);
			static float CalcLength(float* x, int length);
			static int FloatToHalfInt(float _float);
			static float HalfIntToFloat(int hbits);
			static bool CheckWithInInterval(float value, float mean, float var);
		private:


	};
};

#endif /* MATHTOOLS_H_ */
