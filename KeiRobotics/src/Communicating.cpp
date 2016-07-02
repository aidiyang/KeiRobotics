/*
 * Communicating.cpp
 *
 *  Created on: 2014¦~11¤ë11¤é
 *      Author: YunKei
 */

#include <Communicating.h>

using namespace Communication;
using namespace Math;
using namespace Control;

Com::Com(Interface interface, uint32_t addr, int index) : _interface(interface), Index(index){
	switch(interface){
		case __UART:
			_UART = (UART*)addr;
			break;
		case __SPI:
			_Spi = (Spi*)addr;
			break;
		case __I2C:
			break;
	}
}

Communicating::Communicating(Com* com) : readyToSend(false), count(0), _com(com), WatchDog(0), Cmd(0), Data(0),isToken(false), BufferCount(0), PrintType(0), CmdData(0), txBufferCount(0){
}

int Communicating::getTxBufferCount(){
	return txBufferCount;
}

void Communicating::ReceivePoll(){
	switch(_com->_interface){
		case Com::__UART:
			Length = _com->_UART->AvailableLength;
			_com->_UART->Read(Buffer + BufferCount, Length);
			break;
		case Com::__SPI:
			Length = _com->_Spi->AvailableLength;
			_com->_Spi->Read(Buffer + BufferCount, Length);
			break;
		case Com::__I2C:
			break;
	}
	BufferCount += Length;

	if(BufferCount > 0){
		int cutLenght = 0;
		int startPos = -1;
		for(int i = 0; i < BufferCount; i++){
			if(Buffer[i] == '$' && Buffer[i + 4] == '#'){
				int tokenPos = i;
				if(startPos == -1){
					startPos = tokenPos;
				}
				char ch[3] = {Buffer[i + 1], Buffer[i + 2], Buffer[i + 3]};
				cutLenght += 5;
				int d[3];
				d[0] = (int)ch[0] - 14;
				for(int l = 1; l < 3; l++){
					if(ch[l] < 0){
						d[l] = (int)ch[l] + 255;
					}
					else{
						d[l] = (int)ch[l] - 1;
					}
				}
				int halfInt = (((int)d[1]) << 8) | ((int)d[2]);
				Data = MathTools::HalfIntToFloat(halfInt);
				Cmd = d[0];
				Execute(Cmd, Data);
			}
		}
		if(startPos > -1){
			for(int k = 0; k < BufferCount - startPos - cutLenght; k++){
				Buffer[k] = Buffer[k + startPos + cutLenght];
			}
			BufferCount -= startPos + cutLenght;
		}
	}
}

void Communicating::SendPoll(){
	char D[txBufferCount + 1];
	for(int i = 0; i < txBufferCount; i++){
		D[i] = txBuffer[i];
	}
	D[txBufferCount] = '\0';
	switch(_com->_interface){
		case Com::__UART:
			if((!_com->_UART->Conf->_UseDMA || !_com->_UART->isDmaBusy) && txBufferCount >= 5){
				_com->_UART->Print("%s\n", D);
			}
			break;
		case Com::__SPI:
			if(txBufferCount >= 5){
				_com->_Spi->Print(_com->Index, "%s\n", D);
			}
			break;
		case Com::__I2C:
			break;
	}
	txBufferCount = 0;
}

void Communicating::Execute(int cmd, float data){
//	if(App::mApp->mCommunicating3 == this){
//		printf("CMD:%d  DATA:%g\r\n", cmd, data);
//	}
	switch(cmd){

		case CMD::WATCHDOG:
//			App::mCommunicating1->readyToSend = true;
//			App::mCommunicating1->PeriodicCmd = cmd;
//			App::mCommunicating1->PeriodicData = data;
//			App::mApp->mControlling->clearWatchDogCount();
			App::mApp->mCommunicating1->Send(cmd, data);
//			printf("CMD:%d,DATA:%g\r\n", cmd, data);
//			Acknowledgement();
			break;
		case CMD::PRINT_MODE:
			PrintType = data;
			Acknowledgement();
			break;
		case CMD::STOP:
			App::mApp->mControlling->Stopping();
			Acknowledgement();
			break;
		case CMD::POWER:
			App::mApp->mPWM->Control1(data);
			App::mApp->mPWM->Control2(data);
			App::mApp->mPWM->Control3(data);
			App::mApp->mPWM->Control4(data);
			Acknowledgement();
			printf("Power:%g\r\n", data);
			break;
		case CMD::START:
			App::mApp->mControlling->Starting();
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::START;
			Acknowledgement();
			break;
		case CMD::ROLL_KP:
			App::mApp->Motor1PID->setKp(data);
//			App::mApp->mControlling->RollPid->setKp(data);
			Acknowledgement();
			break;
		case CMD::ROLL_KI:
			App::mApp->Motor1PID->setKi(data);
//			App::mApp->mControlling->RollPid->setKi(data);
			Acknowledgement();
			break;
		case CMD::ROLL_KD:
			App::mApp->Motor1PID->setKd(data);
//			App::mApp->mControlling->KdRollPid->setKd(data);
			Acknowledgement();
			break;
		case CMD::PITCH_KP:
			App::mApp->mControlling->PitchPid->setKp(data);
			Acknowledgement();
			break;
		case CMD::PITCH_KI:
			App::mApp->mControlling->PitchPid->setKi(data);
			Acknowledgement();
			break;
		case CMD::PITCH_KD:
			App::mApp->mControlling->KdPitchPid->setKd(data);
			Acknowledgement();
			break;
		case CMD::YAW_KP:
			App::mApp->mControlling->YawPid->setKp(data);
			Acknowledgement();
			break;
		case CMD::YAW_KI:
			App::mApp->mControlling->YawPid->setKi(data);
			Acknowledgement();
			break;
		case CMD::YAW_KD:
			App::mApp->mControlling->KdYawPid->setKd(data);
			Acknowledgement();
			break;
		case CMD::RESET_ALL:
//			App::mApp->mControlling->setStart(false);
//			App::mApp->mControlling->setStarting(false);
//			App::mApp->mControlling->setStopping(false);
//			App::mApp->mControlling->StopAllMotors();
			if(App::mApp->mCompass != 0){
				App::mApp->mCompass->Reset();
			}
			App::mApp->mQuaternion->Reset();

//			for(int i = 0; i < 500; i++){
//				App::mApp->mMPU6050->Update();
//				App::mApp->mHMC5883L->Update();
//				App::mApp->mAcceleration->Update();
//				App::mApp->mOmega->Update();
//				App::mApp->mCompass->Update();
//				App::mApp->mQuaternion->Update();
		//		Delay::DelayMS(2);
	//		}

//			App::mApp->mControlling->RollOffset = MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[0]);
//			App::mApp->mControlling->PitchOffset = MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[1]);
//			App::mApp->mControlling->YawOffset = MathTools::RadianToDegree(App::mApp->mQuaternion->getEuler()[2]);
//			App::mApp->mControlling->Lift = 0;
			Acknowledgement();
			break;
		case CMD::ROLL_OFFSET:
			App::mApp->mControlling->setRollOffset(App::mApp->mControlling->getRollOffset() + data);
			Acknowledgement();
			break;
		case CMD::PITCH_OFFSET:
			App::mApp->mControlling->setPitchOffset(App::mApp->mControlling->getPitchOffset() + data);
			Acknowledgement();
			break;
		case CMD::YAW_OFFSET:
			App::mApp->mControlling->setYawOffset(App::mApp->mControlling->getYawOffset() + data);
			Acknowledgement();
			break;
		case CMD::HIGH:
			for(int i = 0; i < 4; i++){
				switch(i){
					case 0:
						TIM_SetCompare1(TIM8, 2400);
						break;
					case 1:
						TIM_SetCompare2(TIM8, 2400);
						break;
					case 2:
						TIM_SetCompare3(TIM8, 2400);
						break;
					case 3:
						TIM_SetCompare4(TIM8, 2400);
						break;
				}
			}
			Acknowledgement();
			break;
		case CMD::LOW:
			for(int i = 0; i < 4; i++){
				switch(i){
					case 0:
						TIM_SetCompare1(TIM8, 700);
						break;
					case 1:
						TIM_SetCompare2(TIM8, 700);
						break;
					case 2:
						TIM_SetCompare3(TIM8, 700);
						break;
					case 3:
						TIM_SetCompare4(TIM8, 700);
						break;
				}
			}
			Acknowledgement();
			break;
		case CMD::MAX_LIFT_VALUE:
			App::mApp->mControlling->maxLift = data;
			Acknowledgement();
			break;
		case CMD::MIN_LIFT_VALUE:
			App::mApp->mControlling->minLift = data;
			Acknowledgement();
			break;
		case CMD::LIFT:
			App::mApp->mControlling->Lift = data;
			Acknowledgement();
			break;
		case CMD::TARGET_ROLL:
			App::mApp->mControlling->setRollTarget(data);
			Acknowledgement();
			break;
		case CMD::TARGET_PITCH:
			App::mApp->mControlling->setPitchTarget(data);
			Acknowledgement();
			break;
		case CMD::TARGET_YAW:
			App::mApp->mControlling->setYawTarget(data);
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::TARGET_YAW;
			Acknowledgement();
			break;
		case CMD::SET_X_TRANSLATION:
			App::mApp->mLocalization->setEncoderXTranslation(data);
			Acknowledgement();
			break;
		case CMD::SET_Y_TRANSLATION:
			App::mApp->mLocalization->setEncoderYTranslation(data);
			Acknowledgement();
			break;
		case CMD::MOTOR_KP:
			App::mApp->mControlling->Motor1->setKp(data);
			App::mApp->mControlling->Motor2->setKp(data);
			App::mApp->mControlling->Motor3->setKp(data);
			App::mApp->mControlling->Motor4->setKp(data);
			Acknowledgement();
			break;
		case CMD::MOTOR_KI:
			App::mApp->mControlling->Motor1->setKi(data);
			App::mApp->mControlling->Motor2->setKi(data);
			App::mApp->mControlling->Motor3->setKi(data);
			App::mApp->mControlling->Motor4->setKi(data);
			Acknowledgement();
			break;
		case CMD::FORWARD:
			App::mApp->mControlling->Forward();
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::FORWARD;
			Acknowledgement();
			break;
		case CMD::BACKWARD:
			App::mApp->mControlling->Backward();
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::BACKWARD;
			Acknowledgement();
			break;
		case CMD::LEFT:
			App::mApp->mControlling->Left();
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::LEFT;
			Acknowledgement();
			break;
		case CMD::RIGHT:
			App::mApp->mControlling->Right();
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::RIGHT;
			Acknowledgement();
			break;
		case CMD::PAUSE:
			App::mApp->mControlling->Pause();
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::PAUSE;
			Acknowledgement();
			break;
		case CMD::CW:
			App::mApp->mControlling->CW();
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::CW;
			Acknowledgement();
			break;
		case CMD::CCW:
			App::mApp->mControlling->CCW();
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::CCW;
			Acknowledgement();
			break;
		case CMD::MOVE:
			App::mApp->mControlling->Move(App::mApp->mControlling->Speed, data, 0);
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::MOVE;
			Acknowledgement();
			break;
		case CMD::MANUAL_MODE:
			App::mApp->mControlling->ManualMode = true;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::MANUAL_MODE;
			Acknowledgement();
			break;
		case CMD::RETURN_HOME:
			App::mApp->PathState = 999;
			Acknowledgement();
			break;
		case CMD::TEST:
			App::mApp->mCommunicating2->Send(9, 689);
			Acknowledgement();
			break;
		case CMD::CLAMPER_STOP_ALL:
			App::mApp->PeriodicCmd = CMD::CLAMPER_STOP_ALL_RUN;
			App::mApp->PeriodicData = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::CLAMPER_STOP_ALL;
//					App::mApp->mCommunicating3->Send(0,0);
			Acknowledgement();
			break;
		case CMD::CLAMPER_RESET:
			App::mApp->PeriodicCmd = CMD::CLAMPER_RESET_RUN;
			App::mApp->PeriodicData = 0;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::CLAMPER_RESET;
//					App::mApp->mCommunicating3->Send(1,0);
			Acknowledgement();
			break;
		case CMD::CLAMPER_START:
			App::mApp->PeriodicCmd = CMD::CLAMPER_START_RUN;
			App::mApp->PeriodicData = 0;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::CLAMPER_START;
//					App::mApp->mCommunicating3->Send(2,0);
			Acknowledgement();
			break;
		case CMD::CLAMPER_SET_MOTOR1_TARGET:
			App::mApp->PeriodicCmd = CMD::CLAMPER_SET_MOTOR1_TARGET_RUN;
			App::mApp->PeriodicData = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::CLAMPER_SET_MOTOR1_TARGET;
//					App::mApp->mCommunicating3->Send(3,data);
			Acknowledgement();
			break;
		case CMD::CLAMPER_SET_MOTOR2_TARGET:
			App::mApp->PeriodicCmd = CMD::CLAMPER_SET_MOTOR2_TARGET_RUN;
			App::mApp->PeriodicData = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::CLAMPER_SET_MOTOR2_TARGET;
//					App::mApp->mCommunicating3->Send(4,data);
			Acknowledgement();
			break;
		case CMD::CLAMPER_SET_MOTOR3_TARGET:
			App::mApp->PeriodicCmd = CMD::CLAMPER_SET_MOTOR3_TARGET_RUN;
			App::mApp->PeriodicData = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::CLAMPER_SET_MOTOR3_TARGET;
//					App::mApp->mCommunicating3->Send(5,data);
			Acknowledgement();
			break;
		case CMD::CLAMPER_WATCHDOG:
			App::mApp->PeriodicCmd = CMD::CLAMPER_WATCHDOG_RUN;
			App::mApp->PeriodicData = data;
//					App::mApp->mCommunicating3->Send(6,0);
			break;
		case CMD::SUCCESS:
//			App::mApp->PeriodicCmd2 = CMD::SUCCESS;
//			App::mApp->PeriodicData2 = data;
//			App::mApp->mCommunicating2->Send(CMD::SUCCESS, data);
//			printf("SUCCESS:%g\r\n", data);
			Acknowledgement();
			break;
		case CMD::CLAMPER_SET_HORIZONTAL:
			App::mApp->PeriodicCmd = CMD::CLAMPER_SET_HORIZONTAL_RUN;
			App::mApp->PeriodicData = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::CLAMPER_SET_HORIZONTAL;
			Acknowledgement();
			break;
		case CMD::SET_SPEED:
			App::mApp->nextPT.speed = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_SPEED;
			Acknowledgement();
			break;
		case CMD::SET_X_POS:
			App::mApp->nextPT.x = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_X_POS;
			Acknowledgement();
			break;
		case CMD::SET_Y_POS:
			App::mApp->nextPT.y = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_Y_POS;
			Acknowledgement();
			break;
		case CMD::SET_YAW:
			App::mApp->nextPT.yaw = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_YAW;
			Acknowledgement();
			break;
		case CMD::SET_SONIC_CAL_FL:
			if(data > 10){
				App::mApp->nextPT.SonicCalFL = true;
			}
			else{
				App::mApp->nextPT.SonicCalFL = false;
			}
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_SONIC_CAL_FL;
			Acknowledgement();
			break;
		case CMD::SET_SONIC_CAL_FR:
			if(data > 10){
				App::mApp->nextPT.SonicCalFR = true;
			}
			else{
				App::mApp->nextPT.SonicCalFR = false;
			}
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_SONIC_CAL_FR;
			Acknowledgement();
			break;
		case CMD::SET_SONIC_CAL_L:
			if(data > 10){
				App::mApp->nextPT.SonicCalL = true;
			}
			else{
				App::mApp->nextPT.SonicCalL = false;
			}
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_SONIC_CAL_L;
			Acknowledgement();
			break;
		case CMD::SET_SONIC_CAL_R:
			if(data > 10){
				App::mApp->nextPT.SonicCalR = true;
			}
			else{
				App::mApp->nextPT.SonicCalR = false;
			}
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_SONIC_CAL_R;
			Acknowledgement();
			break;
		case CMD::SET_SONIC_CAL_FL_VALUE:
			App::mApp->nextPT.FL = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_SONIC_CAL_FL_VALUE;
			Acknowledgement();
			break;
		case CMD::SET_SONIC_CAL_FR_VALUE:
			App::mApp->nextPT.FR = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_SONIC_CAL_FR_VALUE;
			Acknowledgement();
			break;
		case CMD::SET_SONIC_CAL_L_VALUE:
			App::mApp->nextPT.L = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_SONIC_CAL_L_VALUE;
			Acknowledgement();
			break;
		case CMD::SET_SONIC_CAL_R_VALUE:
			App::mApp->nextPT.R = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_SONIC_CAL_R_VALUE;
			Acknowledgement();
			break;
		case CMD::SET_X_CAL:
			if(data > 10){
				App::mApp->nextPT.CalX = true;
			}
			else{
				App::mApp->nextPT.CalX = false;
			}
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_X_CAL;
			Acknowledgement();
			break;
		case CMD::SET_Y_CAL:
			if(data > 10){
				App::mApp->nextPT.CalY = true;
			}
			else{
				App::mApp->nextPT.CalY = false;
			}
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_Y_CAL;
			Acknowledgement();
			break;
		case CMD::SET_X_CAL_VALUE:
			App::mApp->nextPT.CalXValue = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_X_CAL_VALUE;
			Acknowledgement();
			break;
		case CMD::SET_Y_CAL_VALUE:
			App::mApp->nextPT.CalYValue = data;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::SET_Y_CAL_VALUE;
			Acknowledgement();
			break;
		case CMD::TRIGGER:
			App::mApp->currentPT.speed = App::mApp->nextPT.speed;
			App::mApp->currentPT.x = App::mApp->nextPT.x;
			App::mApp->currentPT.y = App::mApp->nextPT.y;
			App::mApp->currentPT.yaw = App::mApp->nextPT.yaw;
			App::mApp->currentPT.SonicCalFL = App::mApp->nextPT.SonicCalFL;
			App::mApp->currentPT.SonicCalFR = App::mApp->nextPT.SonicCalFR;
			App::mApp->currentPT.SonicCalL = App::mApp->nextPT.SonicCalL;
			App::mApp->currentPT.SonicCalR = App::mApp->nextPT.SonicCalR;
			App::mApp->currentPT.FL = App::mApp->nextPT.FL;
			App::mApp->currentPT.FR = App::mApp->nextPT.FR;
			App::mApp->currentPT.L = App::mApp->nextPT.L;
			App::mApp->currentPT.R = App::mApp->nextPT.R;
			App::mApp->currentPT.CalX = App::mApp->nextPT.CalX;
			App::mApp->currentPT.CalY = App::mApp->nextPT.CalY;
			App::mApp->currentPT.CalXValue = App::mApp->nextPT.CalXValue;
			App::mApp->currentPT.CalYValue = App::mApp->nextPT.CalYValue;
			App::mApp->trigger = true;
			App::mApp->arrived = false;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::TRIGGER;
			Acknowledgement();
			break;
		case CMD::CLAMPER_STOP_ALL_RUN:
			App::mApp->ControlStart = false;
//			printf("STOP\r\n");
			break;
		case CMD::CLAMPER_RESET_RUN:
			//if(App::mApp->IsCal1 == -100){
				App::mApp->Motor1Target = -5.0;
			//}
			//if(App::mApp->IsCal2 == -100){
				App::mApp->Motor2Target = 1.0;
			//}
			//if(App::mApp->IsCal3 == -100){
				App::mApp->Motor3Target = 1.0;
			//}
			App::mApp->IsCal1 = 0;
			App::mApp->IsCal2 = 0;
			App::mApp->IsCal3 = 0;
			App::mApp->ControlStart = true;
//			printf("RESET\r\n");
			break;
		case CMD::CLAMPER_START_RUN:
			App::mApp->ControlStart = true;
//			printf("START\r\n");
			break;
		case CMD::CLAMPER_SET_MOTOR1_TARGET_RUN:
			App::mApp->Motor1Target = data;
//			printf("MOTOR1\r\n");
			break;
		case CMD::CLAMPER_SET_MOTOR2_TARGET_RUN:
			App::mApp->Motor2Target = data;
//			printf("MOTOR2\r\n");
			break;
		case CMD::CLAMPER_SET_MOTOR3_TARGET_RUN:
			App::mApp->Motor3Target = data;
//			printf("MOTOR3\r\n");
			break;
		case CMD::CLAMPER_WATCHDOG_RUN:
			break;
		case CMD::CLAMPER_SET_HORIZONTAL_RUN:
			App::mApp->Motor2Target = data;
			App::mApp->Motor3Target = data;
//			printf("HORIZONTAL\r\n");
			break;
		case CMD::AUTO_MODE:
			App::mApp->mControlling->ManualMode = false;
			App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
			App::mApp->PeriodicData2 = Communicating::AUTO_MODE;
			Acknowledgement();
			break;
		case CMD::NEXT:
			if(App::mApp->arrived){
				App::mApp->PeriodicCmd2 = Communicating::SUCCESS;
				App::mApp->PeriodicData2 = Communicating::NEXT;
			}
			Acknowledgement();
			break;
		default:
//			App::mApp->mUART4->Print("CMD:%d  DATA:%g\r\n", cmd, data);
//			Acknowledgement();
			break;
	}
//	if(App::mApp->mCommunicating2 == this){
//		App::mApp->mUART4->Print("CMD:%d  DATA:%g\r\n", cmd, data);
//		Acknowledgement();
//	}
}

void Communicating::Acknowledgement(){
//	printf("OK\r\n");
//	App::mApp->mLed1->Blink(App::mApp->mLed1, true, 100, 2);
//	App::mApp->mLed2->Blink(App::mApp->mLed2, true, 100, 2);
//	App::mApp->mLed3->Blink(App::mApp->mLed3, true, 100, 2);
//	App::mApp->mLed4->Blink(App::mApp->mLed4, true, 100, 2);
}

void Communicating::Send(int cmd, float data){
	char bytes[5];
	int halfInt = MathTools::FloatToHalfInt(data);
	bytes[0] = 0x24;
	bytes[1] = (char)(cmd + 14);
	bytes[2] = (char)(((halfInt & 0xff00) >> 8) + 1);
	bytes[3] = (char)((halfInt & 0x00ff) + 1);
	bytes[4] = 0x23;
	for(int i = 0; i < 5; i++){
		txBuffer[txBufferCount++] = bytes[i];
	}
}
