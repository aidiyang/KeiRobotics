/*
 * AdaptiveKalman.h
 *
 *  Created on: 2015¦~10¤ë22¤é
 *      Author: wongy
 */

#ifndef ADAPTIVEKALMAN_H_
#define ADAPTIVEKALMAN_H_

#include <Kalman.h>
#include <MovingWindowAverageFilter.h>

namespace Math{

	class AdaptiveKalman : public Kalman{

		public:
			AdaptiveKalman(float x, float minQ, float minR1, float minR2, bool isOneDim = false);
			void Filtering(float data1, float data2);

		private:
			MovingWindowAverageFilter* correlationProcess;
			MovingWindowAverageFilter* meanProcess;
			MovingWindowAverageFilter* correlationData[2];
			MovingWindowAverageFilter* meanData[2];
			float MinQ;
			float MinR1;
			float MinR2;
	};
};

using namespace Math;



#endif /* ADAPTIVEKALMAN_H_ */
