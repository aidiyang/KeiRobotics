/*
 * PWM.h
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#ifndef PWM_H_
#define PWM_H_

#include <stm32f4xx.h>
#include <stm32f4xx_tim.h>
#include <Configuration.h>

using namespace System;

namespace Control{
	class PWM{
		public:
			class PWMConfiguration{
				public:
					PWMConfiguration(Configuration* pwm1, Configuration* pwm2, Configuration* pwm3, Configuration* pwm4, float freq);
					Configuration* _pwm1;
					Configuration* _pwm2;
					Configuration* _pwm3;
					Configuration* _pwm4;
					float _freq;
				private:
			};
			PWM(PWMConfiguration* conf);
			void Control1(float dutyCycle);
			void Control2(float dutyCycle);
			void Control3(float dutyCycle);
			void Control4(float dutyCycle);
		private:
			PWMConfiguration* Conf;
			float MaxPWM;
			float MinPWM;
			float UpperLimit1;
			float UpperLimit2;
	};
};

#endif /* PWM_H_ */
