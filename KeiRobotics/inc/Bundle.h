/*
 * Bundle.h
 *
 *  Created on: 2016¦~6¤ë10¤é
 *      Author: wongy
 */

#ifndef BUNDLE_H_
#define BUNDLE_H_

#include <GPIO.h>
#include <Task.h>

using namespace Time;

namespace System{
	class BlinkObj;
};

namespace Time{
	class Task;
	class TaskObj;
};

namespace System{

	class Bundle{
		public:
			Bundle();
			BlinkObj** mBlinkObj;
			int BlinkObjNum;
			TaskObj* mTaskObj;
	};

};

#endif /* BUNDLE_H_ */
