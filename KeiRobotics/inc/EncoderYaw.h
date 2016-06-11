/*
 * EncoderYaw.h
 *
 *  Created on: 2016¦~3¤ë8¤é
 *      Author: wongy
 */

#ifndef ENCODERYAW_H_
#define ENCODERYAW_H_

#include <Encoder.h>

using namespace Sensors;

namespace Inertia{
	class EncoderYaw{
		public:
			EncoderYaw(Encoder* encoder1, Encoder* encoder2, float radius);
			void Update();
			float getYaw();
			void Reset();
			Encoder* Encoder1;
			Encoder* Encoder2;
			float Yaw;
			float Radius;
			float Interval;
			float PrevTick;
			float PrevOmega;

	};
};

#endif /* ENCODERYAW_H_ */
