/*
 * Sonic.h
 *
 *  Created on: 2014¦~12¤ë18¤é
 *      Author: YunKei
 */

#ifndef SONIC_H_
#define SONIC_H_

#include <Configuration.h>

using namespace System;

namespace Sensors{
	class Sonic{
		public:
			class SonicConfiguration{
				public:
					SonicConfiguration(Configuration* trigger, Configuration* echo, uint8_t echoSource);
					Configuration* _trigger;
					Configuration* _echo;
					uint8_t _echoSource;
				private:
			};

			Sonic(SonicConfiguration* conf);
			static void Reset();
			static int OverFlowCount;
			void Update();
			void TriggerSet();
			void TriggerReset();
			float getDeltaUS();
			void setDeltaUS(float);
		private:
			float DeltaUS;
			SonicConfiguration* Conf;
	};
};

#endif /* SONIC_H_ */
