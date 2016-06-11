/*
 * MovingWindowAverageFilter.h
 *
 *  Created on: 2015¦~3¤ë24¤é
 *      Author: YunKei
 */

#ifndef MOVINGWINDOWAVERAGEFILTER_H_
#define MOVINGWINDOWAVERAGEFILTER_H_

namespace Math{

	class MovingWindowAverageFilter{
		public:
			MovingWindowAverageFilter(int windowSize);
			void Update(float value);
			float getAverage();
			void Clear();
		private:
			float* buffer;
			int WindowSize;
			int bufferCount;
	};
};

#endif /* MOVINGWINDOWAVERAGEFILTER_H_ */
