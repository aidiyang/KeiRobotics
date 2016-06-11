/*
 * MovingWindowAverageFilter.cpp
 *
 *  Created on: 2015¦~3¤ë24¤é
 *      Author: YunKei
 */

#include <MovingWindowAverageFilter.h>
using namespace Math;

MovingWindowAverageFilter::MovingWindowAverageFilter(int windowSize) : WindowSize(windowSize), bufferCount(0){
	buffer = new float[windowSize]{};
}

void MovingWindowAverageFilter::Update(float value){
	if(bufferCount == WindowSize){
		for(int i = 0; i < WindowSize - 1; i++){
			buffer[i] = buffer[i + 1];
		}
		buffer[WindowSize - 1] = value;
	}
	else{
		buffer[bufferCount] = value;
	}
	bufferCount = (bufferCount == WindowSize) ? WindowSize : bufferCount + 1;
}

float MovingWindowAverageFilter::getAverage(){
	float sum = 0;
	for(int i = 0; i < bufferCount; i++){
		sum += buffer[i];
	}
	if(bufferCount > 0){
		return sum / bufferCount;
	}
	else{
		return 0;
	}
}

void MovingWindowAverageFilter::Clear(){
	delete buffer;
	buffer = new float[WindowSize]{};
	bufferCount = 0;
}

