/*
 * Buzzer.h
 *
 *  Created on: 2014¦~8¤ë3¤é
 *      Author: YunKei
 */

#ifndef BUZZER_H_
#define BUZZER_H_

#include <inttypes.h>

namespace Debug{

	class Buzzer{

		public:
			#define BUZZER_PIN	GPIO_Pin_9
			#define BUZZER_PORT	GPIOB
			#define BUZZER_RCC	RCC_AHB1Periph_GPIOB
			Buzzer();
			static Buzzer* getInstance();
			void BuzzerControl(bool);
			void Toggle();
			void Frequency(float, int, bool);
			int getDuration();
			void setDuration(int value);

		private:
			int duration;
	};
};

#endif /* BUZZER_H_ */
