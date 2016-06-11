/*
 * AdaptiveKalman.cpp
 *
 *  Created on: 2015¦~10¤ë22¤é
 *      Author: wongy
 */

#include <AdaptiveKalman.h>
#include <stdio.h>

AdaptiveKalman::AdaptiveKalman(float x,float minQ, float minR1, float minR2, bool isOneDim) : Kalman(x, minQ, minR1, minR2, isOneDim), MinQ(minQ), MinR1(minR1), MinR2(minR2){
	correlationProcess = new MovingWindowAverageFilter(100);
	meanProcess = new MovingWindowAverageFilter(100);
	correlationData[0] = new MovingWindowAverageFilter(100);
	meanData[0] = new MovingWindowAverageFilter(100);
	correlationData[1] = new MovingWindowAverageFilter(100);
	meanData[1] = new MovingWindowAverageFilter(100);
}

void AdaptiveKalman::Filtering(float data1, float data2){
	float x = getCorrectedData();
	correlationProcess->Update(x * x);
	meanProcess->Update(x);
	correlationData[0]->Update(data1 * data1);
	meanData[0]->Update(data1);
	correlationData[1]->Update(data2 * data2);
	meanData[1]->Update(data2);
	float value = getQ() + 0.1f * (correlationProcess->getAverage() - meanProcess->getAverage() * meanProcess->getAverage() - getQ());
	value = value < MinQ ? MinQ : value;
	setQ(value);
	value = getR1() + 0.1f * (correlationData[0]->getAverage() - meanData[0]->getAverage() * meanData[0]->getAverage() - getR1());
	value = value < MinR1 ? MinR1 : value;
	setR1(value);
	value = getR2() + 0.1f * (correlationData[1]->getAverage() - meanData[1]->getAverage() * meanData[1]->getAverage() - getR2());
	value = value < MinR2 ? MinR2 : value;
	setR2(value);
	Kalman::Filtering(data1, data2);
}



