/*
 * Kalman.cpp
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#include <Kalman.h>
#include <stdio.h>

using namespace Math;
using namespace Utility;

Kalman::Kalman(VectorXf X, MatrixXf Q, MatrixXf R) : predictX(X), correctX(X), _Q(Q), _R(R){
	correctP = Q;
	correctP.setIdentity();
	correctP *= 1e-12f;
	predictP = Q;
	predictP.setIdentity();
	predictP *= 1e-12f;
}

void Kalman::setCorrectedData(VectorXf data){
	correctX = data;
}

VectorXf Kalman::getCorrectedData(){
	return correctX;
}

bool Kalman::Filtering(MatrixXf A, VectorXf X, MatrixXf H, VectorXf Z){
	StatePredict(A, X);
	CovariancePredict(A);
	if(!Gain(H)){
		return false;
	}
	StateUpdate(Z, H);
	CovarianceUpdate(H);
	return true;
}

void Kalman::StatePredict(MatrixXf A, VectorXf X){
	predictX = A * X;
}

void Kalman::CovariancePredict(MatrixXf A){
	predictP = A * correctP * A.transpose() + _Q;
}

void Kalman::StateUpdate(VectorXf Z, MatrixXf H){
	correctX = predictX + _K * (Z - H * predictX);
}

void Kalman::CovarianceUpdate(MatrixXf H){
	correctP = predictP - _K * H * predictP;
}

bool Kalman::Gain(MatrixXf H){
	MatrixXf M = H * predictP * H.transpose() + _R;
	MatrixXf invM = M.inverse();
	float sum = invM.sum();
	if(sum != sum){
		return false;
	}
	for(int i = 0; i < invM.rows(); i++){
		for(int j = 0; j < invM.cols(); j++){
			if(fabs(invM(i,j)) > 999999999999.0f){
				return false;
			}
		}
	}
	_K = predictP * H.transpose() * invM;
	return true;
}

void Kalman::Clear(VectorXf X){
	predictP.setIdentity();
	predictP *= 1e-12f;
	correctP.setIdentity();
	correctP *= 1e-12f;
}

MatrixXf Kalman::getQ(){
	return _Q;
}

void Kalman::setQ(MatrixXf Q){
	_Q = Q;
}

MatrixXf Kalman::getR(){
	return _R;
}

void Kalman::setR(MatrixXf R){
	_R = R;
}
