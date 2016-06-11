/*
 * SE3.cpp
 *
 *  Created on: 2015¦~5¤ë14¤é
 *      Author: YunKei
 */

#include <SE3.h>

SE3* _mSE3;

void SE3::printMat(char* ch, int index,  Matrix4f x){
	printf("\n%s:%d\n", ch, index);
	printf("%g  %g  %g  %g\n", x(0,0), x(0,1), x(0,2), x(0,3));
	printf("%g  %g  %g  %g\n", x(1,0), x(1,1), x(1,2), x(1,3));
	printf("%g  %g  %g  %g\n", x(2,0), x(2,1), x(2,2), x(2,3));
	printf("%g  %g  %g  %g\n", x(3,0), x(3,1), x(3,2), x(3,3));

}

void SE3::printVect(char* ch, int index,  Vector4f x){
	printf("\n%s:%d\n", ch, index);
	printf("%g  %g  %g  %g\n", x(0), x(1), x(2), x(3));

}

SE3::SE3(){
	_mSE3 = this;
	prevTranslation.setZero();
	_SE3.setIdentity();
	_pos.setZero();
	_pos(3) = 1;
	pos.setZero();
	prevTranslation.setZero();
	Vector3f x;
	Matrix3f p;
	Matrix3f q;
	Matrix3f r;
	x.setZero();
	p.setIdentity();
	q.setIdentity();
	r.setIdentity();
	q*=0.0001f;
	r*=0.01f;
	SE3UKF = new UKF(x, p, q, r);
//	float r[2] = {0.1f, -1};
//	SE3Kalman[0] = new Kalman(0.0001f, r, 0, 1.0);
//	SE3Kalman[1] = new Kalman(0.0001f, r, 0, 1.0);
//	SE3Kalman[2] = new Kalman(0.0001f, r, 0, 1.0);
}

SE3* SE3::getInstance(){
	return _mSE3;
}

void SE3::Update(){
//	Vector3f z[7];
//	Vector4d temp;
//	for(int i = 0; i < 7; i++){
////		UKF::getInstance()->printVect("x[i]", i, x[i]);
//		for(int j = 0; j < 3; j++){
//			temp(j) = x[i](j);
//		}
//		temp(3) = 1.0;
//		temp = _SE3 * temp;
//		for(int j = 0; j < 3; j++){
//			z[i](j) = temp(j);
//		}
////		UKF::getInstance()->printVect("z[i]", i, z[i]);
//	}

	Matrix4f deltaSE3;
	deltaSE3.setIdentity();
//	Matrix3f deltaRotation = Quaternion::getInstance()->getDeltaRotationMatrix();
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
//			deltaSE3(i,j) = deltaRotation(i,j);
		}
	}

	if(PX4FLOW::getInstance()->getIsValided() == false){
		return;
	}
	Vector3f Translation = PX4FLOW::getInstance()->getTranslation();
	for(int i = 0; i < 3; i++){
		deltaSE3(i,3) = Translation(i);
	}
//	printMat("deltaSE3", 0, deltaSE3);
//	printf("N: %g\n", deltaSE3.norm());
//	printMat("SE3", 0, _SE3);
//	printf("N: %g\n", _SE3.norm());
	_SE3 = deltaSE3;

//	printMat("SE3", 1, _SE3);
//	printf("N: %g\n", _SE3.norm());
//	UKF::printMat("R", 0, Quaternion::getInstance()->getRotationMatrix());
//	printVect("_pos", 0, _pos);
	_pos = _SE3 * _pos;
//	printVect("_pos", 1, _pos);
//	printMat("deltaR", 0, deltaSE3);
//	printMat("SE3", 1, deltaSE3 * _SE3);

	for(int i = 0; i < 3; i++){
		pos(i) = _pos(i);
	}
//	UKF::printVect("pos", 0, pos);
	Vector3f x[7];
	SE3UKF->getSigmaPoints(x);

	pos = SE3UKF->Filtering(x,x,pos);
//	SE3Kalman[0]->Filtering(&pos(0), pos(0), 0);
//	SE3Kalman[1]->Filtering(&pos(1), pos(1), 0);
//	SE3Kalman[2]->Filtering(&pos(2), pos(2), 0);



	for(int i = 0; i < 3; i++){
		_pos(i) = pos(i);
	}
//	printVect("_pos", 0, _pos);
//	UKF::printVect("pos", 0, pos);

}

Vector3f SE3::getPos(){
	return pos;
}

Matrix4f SE3::getSE3(){
	return _SE3;
}

void SE3::reset(){

	prevTranslation.setZero();
	_SE3.setIdentity();
	_pos.setZero();
	_pos(3) = 1;
	pos.setZero();
	prevTranslation.setZero();
	Vector3f x;
	Matrix3f p;
	Matrix3f q;
	Matrix3f r;
	x.setZero();
	p.setIdentity();
	q.setIdentity();
	r.setIdentity();
	q*=0.0001f;
	r*=0.01f;
	delete SE3UKF;
	SE3UKF = new UKF(x, p, q, r);
//	float r[2] = {0.1f, -1};
//	SE3Kalman[0] = new Kalman(0.0001f, r, 0, 1.0);
//	SE3Kalman[1] = new Kalman(0.0001f, r, 0, 1.0);
//	SE3Kalman[2] = new Kalman(0.0001f, r, 0, 1.0);
}
