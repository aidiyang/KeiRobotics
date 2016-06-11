/*
 * Fuzzy.h
 *
 *  Created on: 2015¦~4¤ë28¤é
 *      Author: YunKei
 */

#ifndef FUZZY_H_
#define FUZZY_H_

namespace Math{

	class Fuzzy{

		public:

			Fuzzy(int index, int setLength, float maxE, float maxEC, float maxU, float minE, float minEC, float minU, float maxKp, float maxKi, float maxKd, float** fuzzyTable);
			static Fuzzy* getInstance(int index);
			bool FuzzyAlgorithm(float, float, float*, float*, float*);

		private:
			float MaxE, MaxEC, MaxU;
			float MinE, MinEC, MinU;
			int SetLength;
			float MaxKp, MaxKi, MaxKd;
			float Ke, Kec, Ku;
			int E, EC, U;
			float** FuzzyTable;
			void Fuzzification(float, float);
			bool FuzzyDerivation();
			void Defuzzification(float*, float*, float*);

	};
};

#endif /* FUZZY_H_ */
