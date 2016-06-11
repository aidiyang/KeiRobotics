/*
 * Battery.h
 *
 *  Created on: 2014¦~11¤ë30¤é
 *      Author: YunKei
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include <inttypes.h>

namespace Debug{
	class Battery{
		public:
			Battery();
			static Battery* getInstance();
			double getBatteryLevel();
		private:
			uint16_t BatteryLevelBuffer;
			double BatteryLevel;
	};
};

#endif /* BATTERY_H_ */
