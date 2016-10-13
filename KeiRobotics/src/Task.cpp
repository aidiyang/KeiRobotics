/*
 * Task.cpp
 *
 *  Created on: 2014¦~8¤ë6¤é
 *      Author: YunKei
 */

#include <App.h>
#include <Task.h>
#include <Ticks.h>
#include <stdio.h>
#include <stdio.h>
#include <stm32f4xx_iwdg.h>
#include <Communicating.h>
#include <UART.h>
#include <string>
#include <Bundle.h>

using namespace std;

Bundle* Task::mBundle = 0;

TaskObj::TaskObj(uint16_t period, pTask fn, string fnName, bool isPeriodic, int BreakCout){
	mTask = fn;
	TaskPeriod = period;
	TaskName = fnName;
	IsPeriodic = isPeriodic;
	_BreakCout = BreakCout;
	hangCount = 0;
//	printf("mTask[TasksNum]:%lx\r\n", mTask);
//	printf("TaskPeriod[TasksNum]:%d\r\n", TaskPeriod);
//	printf("TaskName[TasksNum]:%s\r\n", TaskName.c_str());
//	printf("IsPeriodic[TasksNum]:%d\r\n", IsPeriodic);
//	printf("_BreakCout[TasksNum]:%d\r\n", _BreakCout);
}

Task::Task() : mTicks(App::mApp->mTicks), OnWatchDog(App::mApp->mTicks->OnWatchDog), TasksNum(0), currentTaskNum(0), IsPrintTaskNum(false), KeepLoopping(true), Count(-1){
	mBundle = new Bundle();
}

void Task::Attach(uint16_t period, pTask fn, string fnName, bool isPeriodic, int BreakCout, bool keepLoopping){
	if(TasksNum >= 1023){
		printf("Cannot attach task any more!\r\n");
		return;
	}

	mTaskObj[TasksNum] = new TaskObj(period, fn, fnName, isPeriodic, BreakCout);
	TasksNum++;
	KeepLoopping = keepLoopping;
//	printf("TasksNum:%d\r\n\n", TasksNum);
}

void Task::DeAttach(string fnName){

	for(int i = 0; i < TasksNum; i++){
		if(mTaskObj[i]->TaskName.compare(fnName) == 0){
			delete mTaskObj[i];
			for(int j = i; j < TasksNum - 1; j++){
				mTaskObj[j] = mTaskObj[j + 1];
			}
			TasksNum--;
			return;
		}
	}
}

void Task::printDeration(){
	for(int i = 0; i < TasksNum; i++){
		printf("%s:t:%d\r\n", mTaskObj[i]->TaskName.c_str(), mTaskObj[i]->duration[1] - mTaskObj[i]->duration[0]);
	}
}

void Task::Run(bool isPrintTaskNum){

	IsPrintTaskNum = isPrintTaskNum;

	uint16_t ticksImg = 0;
	bool isBreak = false;
	do{
		if(mTicks->getTicks() != ticksImg){
			ticksImg = mTicks->getTicks();
			if(OnWatchDog){
				IWDG_ReloadCounter();
			}
			for(int i = 0; i < TasksNum; i++){
				if(mTicks->TicksComp(mTaskObj[i]->TaskPeriod, ticksImg)){
					mTaskObj[i]->hangCount = 0;
					currentTaskNum = i;
					if(mTaskObj[i]->_BreakCout != 0){
						mBundle->mTaskObj = mTaskObj[i];
						mTaskObj[i]->duration[0] = mTicks->getTicks();
						mTaskObj[i]->mTask(mBundle);
						mTaskObj[i]->duration[1] = mTicks->getTicks();
						if(!(mTaskObj[i]->IsPeriodic)){
							if(mTaskObj[i]->_BreakCout > 0){
								mTaskObj[i]->_BreakCout--;
							}
						}
					}
				}
			}
			for(int i = 0; i < TasksNum; i++){
				if(!mTaskObj[i]->IsPeriodic){
					if(mTaskObj[i]->_BreakCout == 0){
						DeAttach(mTaskObj[i]->TaskName);
						if(!KeepLoopping){
							isBreak = true;
						}
					}
				}
			}
		}

	}while(KeepLoopping || !isBreak);
}
