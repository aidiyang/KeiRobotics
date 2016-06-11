/*
 * Kalman.h
 *
 *  Created on: 2014¦~8¤ë24¤é
 *      Author: YunKei
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include <MathTools.h>
#include <AdditionalTools.h>
#include <Eigen/Eigen>
using Eigen::MatrixXf;
using Eigen::VectorXf;

namespace Math{

	class Kalman{

		public:
			Kalman(VectorXf X, MatrixXf Q, MatrixXf R);
			bool Filtering(MatrixXf A, VectorXf X, MatrixXf H, VectorXf Z);
			void Clear(VectorXf X);
			void setCorrectedData(VectorXf data);
			VectorXf getCorrectedData();
			MatrixXf getQ();
			void setQ(MatrixXf Q);
			MatrixXf getR();
			void setR(MatrixXf R);

		private:

			MatrixXf _Q;
			MatrixXf _R;
			VectorXf correctX;
			VectorXf  predictX;
			MatrixXf predictP;
			MatrixXf correctP;
			MatrixXf _K;
			void StatePredict(MatrixXf A, VectorXf X);
			void CovariancePredict(MatrixXf A);
			bool Gain(MatrixXf H);
			void StateUpdate(VectorXf Z, MatrixXf H);
			void CovarianceUpdate(MatrixXf H);
	};
};

#endif /* KALMAN_H_ */
