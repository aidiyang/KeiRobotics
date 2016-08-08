/*
 * InputCapture.h
 *
 *  Created on: 2016¦~7¤ë29¤é
 *      Author: wongy
 */

#ifndef INPUTCAPTURE_H_
#define INPUTCAPTURE_H_

#include <stm32f4xx.h>
#include <stm32f4xx_tim.h>
#include <Configuration.h>
#include <AdditionalTools.h>

using namespace System;

namespace Sensors{

	class InputCaptureConfiguration{
		public:
			enum TimerConf{
				TimerConf1,
				TimerConf2,
				TimerConf3,
				TimerConf4,
				TimerConf5,
				TimerConf6
			};
			InputCaptureConfiguration(Configuration* cap1, Configuration* cap2, Configuration* cap3, Configuration* cap4, TimerConf timerConf, float timerClk);
			Configuration* _cap1;
			Configuration* _cap2;
			Configuration* _cap3;
			Configuration* _cap4;
			TimerConf _timerConf;
			float TimerClk;
	};

	class InputCapture{
		public:
			InputCapture(InputCaptureConfiguration* conf);
			InputCaptureConfiguration* Conf;
			TIM_TypeDef* TIMx;
			float PrevTimestamp1;
			float PrevTimestamp2;
			float PrevTimestamp3;
			float PrevTimestamp4;
			float PrevOVFCount1;
			float PrevOVFCount2;
			float PrevOVFCount3;
			float PrevOVFCount4;
			float Period1;
			float Period2;
			float Period3;
			float Period4;
			uint32_t OVFCount;
	};
};

#endif /* INPUTCAPTURE_H_ */
