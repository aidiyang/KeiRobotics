/*
 * Task.h
 *
 *  Created on: 2014¦~8¤ë6¤é
 *      Author: YunKei
 */

#ifndef TASK_H_
#define TASK_H_

#include <inttypes.h>
#include <Ticks.h>
#include <string>
#include <Bundle.h>

using namespace System;
using namespace std;

namespace System{
	class Bundle;
};

namespace Time{

	class TaskObj{
		public:
			typedef void (*pTask)(Bundle* bundle);
			TaskObj(uint16_t period, pTask fn, string fnName, bool isPeriodic, int BreakCout = -1);
			int duration[2];
			pTask mTask;
			uint16_t TaskPeriod;
			string TaskName;
			bool IsPeriodic;
			int _BreakCout;
			int hangCount;
	};
	class Task{

		public:
			typedef void (*pTask)(Bundle* bundle);
			Task();
			void Attach(uint16_t period, pTask fn, string fnName, bool isPeriodic, int BreakCout = -1, bool keepLoopping = true);
			void DeAttach(string fnName);
			void Run(bool isPrintTaskNum = false);
			void printDeration();
			bool IsPrintTaskNum;
			int currentTaskNum;
			int Count;
			uint16_t TasksNum;
			static Bundle* mBundle;
			TaskObj* mTaskObj[1024];

		private:
			bool KeepLoopping;
			bool OnWatchDog;
			Ticks* mTicks;
	};
};

#endif /* TASK_H_ */
