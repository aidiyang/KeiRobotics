/*
  * Led.cpp
 *
 *  Created on: 2014¦~8¤ë3¤é
 *      Author: YunKei
 */

#include <Led.h>
#include <App.h>
#include <Config.h>
#include <Task.h>
#include <stdio.h>
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

Led* LedAddr = 0;

Led::LedConfiguration::LedConfiguration(Configuration* led, BitAction onState) : _led(led), _onState(onState), _offState(onState ? Bit_RESET : Bit_SET){
};

Led::Led(LedConfiguration* conf) : Conf(conf){
	GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(conf->_led->_rcc, ENABLE);
	GPIO_InitStructure.GPIO_Pin = conf->_led->_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(conf->_led->_port, &GPIO_InitStructure);

	LedControl(false);
}

void Led::LedControl(bool onState){
	GPIO_WriteBit(Conf->_led->_port, Conf->_led->_pin, onState ? Conf->_onState : Conf->_offState);
}

void BlinkTask1(){
	App::mApp->mLed1->Toggle();
}

void BlinkTask2(){
	App::mApp->mLed2->Toggle();
}

void BlinkTask3(){
	App::mApp->mLed3->Toggle();
}

void BlinkTask4(){
	App::mApp->mLed4->Toggle();
}

void BlinkTask5(){
	App::mApp->mSonicTrigger[0]->Toggle();
}

void BlinkTask6(){
	App::mApp->mSonicTrigger[1]->Toggle();
}

void BlinkTask7(){
	App::mApp->mSonicTrigger[2]->Toggle();
}

void BlinkTask8(){
	App::mApp->mSonicTrigger[3]->Toggle();
}

void BlinkTask9(){
	App::mApp->mSonicTrigger[4]->Toggle();
}

void BlinkTask10(){
	App::mApp->mSonicTrigger[5]->Toggle();
}

void BlinkTask11(){
	App::mApp->mSonicTrigger[6]->Toggle();
}

void BlinkTask12(){
	App::mApp->mSonicTrigger[7]->Toggle();
}

void BlinkTask13(){
	App::mApp->mSonicTrigger[8]->Toggle();
}

void BlinkTask14(){
	App::mApp->mSonicTrigger[9]->Toggle();
}

void BlinkTask15(){
	App::mApp->mSonicTrigger[10]->Toggle();
}

void BlinkTask16(){
	App::mApp->mSonicTrigger[11]->Toggle();
}

void BlinkTask17(){
	App::mApp->mSonicTrigger[12]->Toggle();
}

void BlinkTask18(){
	App::mApp->mSonicTrigger[13]->Toggle();
}

void BlinkTask19(){
	App::mApp->mSonicTrigger[14]->Toggle();
}

void BlinkTask20(){
	App::mApp->mSonicTrigger[15]->Toggle();
}

void Led::Toggle(){
	GPIO_ToggleBits(Conf->_led->_port, Conf->_led->_pin);
}

//void Led::Blink(bool onState, uint16_t period, int count){
//	Led* LedAddr = this;
//	if(LedAddr == App::mApp->mLed1){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask1, "BlinkTask1", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask1, "BlinkTask1", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask1);
//		}
//	}
//	else if(LedAddr == App::mApp->mLed2){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask2, "BlinkTask2", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask2, "BlinkTask2", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask2);
//		}
//	}
//	else if(LedAddr == App::mApp->mLed3){
//			if(onState){
//				if(count >= 0){
//					App::mApp->mTask->Attach(period / 2, 0, BlinkTask3, "BlinkTask3", false, 2 * count);
//				}
//				else{
//					App::mApp->mTask->Attach(period / 2, 0, BlinkTask3, "BlinkTask3", true);
//				}
//			}
//			else{
//				App::mApp->mTask->DeAttach(BlinkTask3);
//			}
//	}
//	else if(LedAddr == App::mApp->mLed4){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask4, "BlinkTask4", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask4, "BlinkTask4", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask4);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[0]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask5, "BlinkTask5", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask5, "BlinkTask5", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask5);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[1]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask6, "BlinkTask6", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask6, "BlinkTask6", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask6);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[2]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask7, "BlinkTask7", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask7, "BlinkTask7", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask7);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[3]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask8, "BlinkTask8", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask8, "BlinkTask8", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask8);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[4]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask9, "BlinkTask9", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask9, "BlinkTask9", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask9);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[5]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask10, "BlinkTask10", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask10, "BlinkTask10", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask10);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[6]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask11, "BlinkTask11", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask11, "BlinkTask11", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask11);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[7]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask12, "BlinkTask12", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask12, "BlinkTask12", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask12);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[8]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask13, "BlinkTask13", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask13, "BlinkTask13", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask13);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[9]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask14, "BlinkTask14", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask14, "BlinkTask14", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask14);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[10]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask15, "BlinkTask15", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask15, "BlinkTask15", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask15);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[11]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask16, "BlinkTask16", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask16, "BlinkTask16", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask16);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[12]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask17, "BlinkTask17", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask17, "BlinkTask17", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask17);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[13]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask18, "BlinkTask18", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask18, "BlinkTask18", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask18);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[14]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask19, "BlinkTask19", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask19, "BlinkTask19", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask19);
//		}
//	}
//	else if(LedAddr == App::mApp->mSonicTrigger[15]){
//		if(onState){
//			if(count >= 0){
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask20, "BlinkTask20", false, 2 * count);
//			}
//			else{
//				App::mApp->mTask->Attach(period / 2, 0, BlinkTask20, "BlinkTask20", true);
//			}
//		}
//		else{
//			App::mApp->mTask->DeAttach(BlinkTask20);
//		}
//	}
//}
