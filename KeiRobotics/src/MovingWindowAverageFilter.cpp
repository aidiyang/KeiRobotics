/*
 * MovingWindowAverageFilter.cpp
 *
 *  Created on: 2015¦~3¤ë24¤é
 *      Author: YunKei
 */

#include <MovingWindowAverageFilter.h>
#include <stdio.h>
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

float MovingWindowAverageFilter::getMedian(){
	float array[WindowSize];
	bool index[WindowSize];
	for(int i = 0; i < WindowSize; i++){
		array[i] = 99999.9f;
		index[i] = true;
	}
	for(int i = 0; i < WindowSize; i++){
		int count = 0;
		for(int j = 0; j < WindowSize; j++){
			if(index[j]){
				if(array[i] > buffer[j]){
					array[i] = buffer[j];
					count++;
				}
			}
		}
		index[count] = false;
	}


//	printf("%d,%d,%d,%d,%d,%d,%d\r\n", index[0], index[1], index[2], index[3], index[4], index[5], index[6]);
//	printf("%g,%g,%g,%g,%g,%g,%g\r\n", array[0], array[1], array[2], array[3], array[4], array[5], array[6]);
	return array[WindowSize/2+1];
}

void MovingWindowAverageFilter::Clear(){
	delete buffer;
	buffer = new float[WindowSize]{};
	bufferCount = 0;
}

