/*
 * Fuzzy.cpp
 *
 *  Created on: 2015¦~4¤ë28¤é
 *      Author: YunKei
 */

#include <Fuzzy.h>
#include <Math.h>
#include <stdlib.h>
#include <stdio.h>
#include <Communicating.h>
#include <UART.h>

Fuzzy* _mFuzzy[10];

Fuzzy::Fuzzy(int index, int setLength,
			float maxE, float maxEC, float maxU,
			float minE, float minEC, float minU,
			float maxKp, float maxKi, float maxKd, float** fuzzyTable) : SetLength(setLength),
																		MaxE(maxE),
																		MaxEC(maxEC),
																		MaxU(maxU),
																		MinE(minE),
																		MinEC(minE),
																		MinU(minU),
																		MaxKp(maxKp),
																		MaxKi(maxKi),
																		MaxKd(maxKd),
																		E(0),
																		EC(0),
																		U(0){
	Ke = (float)(2.0 * SetLength) / (MaxE - MinE);
	Kec = (float)(2.0 * SetLength) / (MaxEC - MinEC);
	Ku = (MaxU - MinU) / (2.0 * SetLength);
	FuzzyTable = new float*[2 * SetLength + 1];
	for(int i = 0; i < 2 * SetLength + 1; i++){
		FuzzyTable[i] = new float[2 * SetLength + 1];
	}
	for(int i = 0; i < 2 * SetLength + 1; i++){
		for(int j = 0; j < 2 * SetLength + 1; j++){
			FuzzyTable[i][j] = fuzzyTable[i][j];
		}
	}
	_mFuzzy[index] = this;
}

Fuzzy* Fuzzy::getInstance(int index){
	return _mFuzzy[index];
}

bool Fuzzy::FuzzyAlgorithm(float e, float ec, float* Kp, float* Ki, float* Kd){

	Fuzzification(e, ec);
	if(!FuzzyDerivation()){
		return false;
	}
	Defuzzification(Kp, Ki, Kd);
	return true;
}

void Fuzzy::Fuzzification(float e, float ec){
	E = Ke * (e - (MaxE + MinE) / 2.0);
	EC = Kec * (ec - (MaxEC + MinEC) / 2.0);
}

bool Fuzzy::FuzzyDerivation(){
	if(abs(E) <= SetLength && abs(EC) <= SetLength){
		U = FuzzyTable[EC + 7][E + 7];
		return true;
	}
	else{
		return false;
	}
}

void Fuzzy::Defuzzification(float* Kp, float* Ki, float* Kd){
	float u = (float)(Ku * U) + (MaxU + MinU) / 2.0;
	*Kp = u * MaxKp;
	//*Ki = MaxKi * (1 - u);
	*Kd =  MaxKd * (1 - u);
}

