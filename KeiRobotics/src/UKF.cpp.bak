/*
 * UKF.cpp
 *
 *  Created on: 2015¦~4¤ë17¤é
 *      Author: YunKei
 */

#include <UKF.h>
#include <math.h>
#include <MathAlgorithm.h>
#include <Ticks.h>
#include <MathTools.h>

extern uint32_t heap_loc;

UKF* _mUKF;

void UKF::printMat(char* ch, int index,  Matrix3f x){
	printf("\n%s:%d\n", ch, index);
	printf("%g  %g  %g\n", x(0,0), x(0,1), x(0,2));
	printf("%g  %g  %g\n", x(1,0), x(1,1), x(1,2));
	printf("%g  %g  %g\n", x(2,0), x(2,1), x(2,2));

}

void UKF::printVect(char* ch, int index,  Vector3f x){
	printf("\n%s:%d\n", ch, index);
	printf("%g  %g  %g\n", x(0), x(1), x(2));

}

UKF::UKF(Vector3f x, Matrix3f p, Matrix3f q, Matrix3f r, float k, float a){
	_mUKF = this;
	K = k;
	_a = a;
	lambda = a*a*(3+k) - 3;
	predictX.setZero();
	correctX.setZero();
	predictP.setZero();
	correctP.setZero();
	predictZ.setZero();
	KalmanGain.setZero();
	Q.setZero();
	R.setZero();
	correctX = x;
	correctP = p;
	Q = q;
	R = r;
	for (int i = 0; i < 3; i++){
		S[i].setZero();
	}
	S[3] = correctX;
	float sumWm = Wm[3] = lambda / (3.0 + lambda);
	float sumWc = Wc[3] = lambda / (3.0 + lambda) + (1 - a*a + 2);
	for (int i = 0; i < 3; i++)
	{
		Wm[i] = Wc[i] = 1.0 / (2.0 * (3.0 + lambda));
		Wm[4 + i] = Wc[4 + i] = 1.0 / (2.0 * (3.0 + lambda));
		sumWm += Wm[i];
		sumWm += Wm[4 + i];
		sumWc += Wc[i];
		sumWc += Wc[4 + i];
	}
	for (int i = 0; i < 7; i++)
	{
		Wm[i] /= sumWm;
		Wc[i] /= sumWc;
	}
	CalcSigmaPoints();
}

UKF* UKF::getInstance(){
	return _mUKF;
}

void UKF::CalcSigmaPoints()
{
	S[3] = correctX;
//	Matrix3f P;
//	Matrix3f D;
//	Matrix3f Q;
//	Matrix3f invQ;
//	P = correctP;
	correctP = correctP.llt().matrixL();
//	MathAlgorithm::EigenDecomposition(P, D, Q);
//	for (int j = 0; j < 3; j++)
//	{
//		D(j,j) = sqrt(D(j,j));
//	}
//	bool isInvertable = false;
//	Q.computeInverseWithCheck(invQ, isInvertable);
//	correctP = Q * D * invQ;
	correctP *= sqrt(3.0 + K);
	for(int i = 0; i < 3; i++){
		Vector3f v;
		v.setZero();
		for(int j = 0; j < 3; j++){
			v(j,0)	= correctP(j,i);
		}
		S[i] = correctX + v;
		S[4 + i] = correctX - v;
	}
}

void UKF::CalcEstimation(Vector3f x[7], Vector3f z[7])
{
    predictX.setZero();
    predictP.setZero();
    predictZ.setZero();
    for (int i = 0; i < 7; i++)
    {
        predictX += x[i]*Wm[i];
        predictZ += z[i]*Wm[i];
    }
//    for(int i = 0; i < 3; i++){
//    	predictX(i) = MathTools::TrimResolution(predictX(i));
//    	predictZ(i) = MathTools::TrimResolution(predictZ(i));
//	}
    for (int i = 0; i < 7; i++)
    {
        predictP += (S[i] - predictX) * (S[i] - predictX).transpose()*Wc[i];
    }
    predictP += Q;
}

void UKF::CalcMeasurement(Vector3f x[7], Vector3f z[7], Vector3f Z)
{
    Matrix3d Pzz;
    Matrix3d Pxz;
    Matrix3d invPzz;
    Pzz.setZero();
    Pxz.setZero();
    for (int i = 0; i < 7; i++)
    {
        Pzz += (z[i] - predictZ).cast<double>() * (z[i] - predictZ).transpose().cast<double>()*Wc[i];
        Pxz += (x[i] - predictX).cast<double>() * (z[i] - predictZ).transpose().cast<double>()*Wc[i];
    }
    Pzz += R.cast<double>();
    invPzz = Pzz.inverse();
//    for(int i = 0; i < 3; i++){
//        for(int j = 0; j < 3; j++){
//        	Pzz(i,j) = MathTools::TrimResolution(Pzz(i,j));
//        	Pxz(i,j) = MathTools::TrimResolution(Pxz(i,j));
//        	invPzz(i,j) = MathTools::TrimResolution(invPzz(i,j));
//        }
//    }
    KalmanGain = (Pxz * invPzz).cast<float>();
    correctX = predictX + (KalmanGain * (Z - predictZ));
    correctP = predictP - (KalmanGain * Pzz.cast<float>() * KalmanGain.transpose());
//    for(int i = 0; i < 3; i++){
//    	correctX(i) = MathTools::TrimResolution(correctX(i));
//        for(int j = 0; j < 3; j++){
//        	correctP(i,j) = MathTools::TrimResolution(correctP(i,j));
//        }
//    }
//	printMat("Pzz",0, Pzz);
//	printMat("invPzz",0, invPzz);
//	printMat("Pxz",0, Pxz);
}

void UKF::getSigmaPoints(Vector3f x[7]){
	for (int i = 0; i < 7; i++)
	{
		x[i] = S[i];
	}
}

Vector3f UKF::Filtering(Vector3f x[7], Vector3f z[7], Vector3f Z)
{
//	for(int i = 0; i < 7; i++){
//		printVect("x",i, x[i]);
//		printVect("z",i, z[i]);
//	}
	CalcEstimation(x, z);
//		printVect("predictX",0, predictX);
//		printMat("predictP",0, predictP);
//		printVect("predictZ",0, predictZ);
	CalcMeasurement(x, z, Z);
//	printVect("correctX",0, correctX);
//	printMat("correctP",0, correctP);
//	printMat("KalmanGain",0, KalmanGain);
//	printf("N: %g\n", KalmanGain.norm());
	CalcSigmaPoints();
//	for(int i = 0; i < 7; i++){
//		printVect("S",i, S[i]);
//	}
	return correctX;
}
