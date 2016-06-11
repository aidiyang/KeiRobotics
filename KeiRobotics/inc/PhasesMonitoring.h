/*
 * PhasesMonitoring.h
 *
 *  Created on: 2015¦~3¤ë20¤é
 *      Author: YunKei
 */

#ifndef PHASESMONITORING_H_
#define PHASESMONITORING_H_

#include <Kalman.h>
#include <MovingWindowAverageFilter.h>

namespace Sensors{

	class PhasesMonitoring{

		#define PERIOD_LOWER_LIMIT 		10500.0f//5250.0f
		#define PERIOD_UPPER_LIMIT 		10500000.0f//5250000.0f
		#define CONSISTANT_PERIOD_CHANGE 		1000000.0f//5250000.0f
		#define FREQ_ALLOW_CHANGE_RANGE	100.0f
		#define PERIOD_UPDATE_COUNT		1
		#define MIN_FREQ 			100.0f
		#define MAX_FREQ 			1200.0f
		#define SCHMITT_RANGE		200.0f
		#define BANDWIDTH 			(MAX_FREQ - MIN_FREQ)
		#define STARTUP_COUNT	100

		public:
			PhasesMonitoring();
			static PhasesMonitoring* getInstance();
			float getPeriod(int index);
			void setPeriod(int index, float value);
			float getPrePeriod(int index);
			void setPrePeriod(int index, float value);
			int getOVFCount(int index);
			void incrementOVFCount(int index);
			void resetOVFCount(int index);
			int getExpaired(int index);
			void incrementExpaired(int index);
			void resetExpaired(int index);
			int getStartCount(int index);
			void setStartCount(int index, int value);
			float getTimestamp(int index);
			void setTimestamp(int index, float value);
			float getPreTimestamp(int index);
			void setPreTimestamp(int index, float value);
			float getRPM(int index);
			Kalman* getKalman(int index);
			void setToggleCount(int value);
			int getToggleCount();
			void setInterruptCount(int value);
			int getInterruptCount();
			void setInterruptIndex(int value);
			int getInterruptIndex();
			void setInterruptEnabled(int index, bool value);
			MovingWindowAverageFilter* getMovingWindowAverageFilter(int index);

		private:
			float period[4];
			int OVFCount[4];
			int expaired[4];
			float preTimestamp[4];
			float prePeriod[4];
			int startCount[4];
			Kalman* phaseKalman[4];
			int toggleCount;
			int interruptCount;
			int interruptIndex;
//			MovingWindowAverageFilter* phaseAverage[4];
	};
};

#endif /* PHASESMONITORING_H_ */
