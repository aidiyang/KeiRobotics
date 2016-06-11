/*
 * Sonic.h
 *
 *  Created on: 2016¦~4¤ë13¤é
 *      Author: wongy
 */

#ifndef SONIC_H_
#define SONIC_H_

#include <Led.h>
#include <ExternalInterrupt.h>
#include <MovingWindowAverageFilter.h>

using namespace Debug;
using namespace Math;

namespace Sensors{
	class Sonic{
		public:
			class SonicConfiguration{
				public:
					SonicConfiguration(Led::LedConfiguration* trigger, Configuration* echo);
					Led::LedConfiguration* Trigger;
					Configuration* Echo;
			};
			Sonic(SonicConfiguration* conf);
			void Update();
			SonicConfiguration* Conf;
			Led* Trigger;
			ExternalInterrupt* Echo;
			float Distance;
			static int SonicNum;
			static int OverFlowCount;
			MovingWindowAverageFilter* SonicFilter;
	};
};

#endif /* SONIC_H_ */
