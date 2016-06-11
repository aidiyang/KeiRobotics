/*
 * Encoder.h
 *
 *  Created on: 2016¦~2¤ë23¤é
 *      Author: wongy
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <stm32f4xx.h>
#include <stm32f4xx_tim.h>
#include <Configuration.h>
#include <AdditionalTools.h>

using namespace System;

namespace Sensors{
	class Encoder{
		public:
			class EncoderConfiguration{
				public:
					enum TimerSelections{
						TimerConf1,
						TimerConf2,
						TimerConf3,
						TimerConf4,
						TimerConf5,
						TimerConf6};
					EncoderConfiguration(Configuration* signalA, Configuration* signalB, TimerSelections timerConf, float timerClk);
					Configuration* _signalA;
					Configuration* _signalB;
					float TimerClk;
					TimerSelections _timerConf;
			};
			Encoder(EncoderConfiguration* conf, float scale, float angle);
			void Update(float angle);
			float getVel();
			float getRawVel();
			float getPos();
			void setPos(float value);
			void Reset();
			float calcEncoderErrorCompensationFactor(float angle);
		private:
			EncoderConfiguration* Conf;
			TIM_TypeDef* TIMx;
			float Interval;
			float RawVel;
			float Vel;
			float Pos;
			float Scale;
			float PrevTick;
			float PrevRawVel;
			float PrevVel;
			float Angle;
	};
}

#endif /* ENCODER_H_ */
