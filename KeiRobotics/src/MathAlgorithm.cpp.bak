/*
 * MathAlgorithm.cpp
 *
 *  Created on: 2015¦~4¤ë17¤é
 *      Author: YunKei
 */

#include <MathAlgorithm.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <Delay.h>
#include <MathTools.h>

//extern uint32_t heap_loc;
//
//// calculate the cofactor of element (row,col)
//int GetMinor(float **src, float **dest, int row, int col, int order)
//{
//    // indicate which col and row is being copied to dest
//    int colCount=0,rowCount=0;
//
//    for(int i = 0; i < order; i++ )
//    {
//        if( i != row )
//        {
//            colCount = 0;
//            for(int j = 0; j < order; j++ )
//            {
//                // when j is not the element
//                if( j != col )
//                {
//                    dest[rowCount][colCount] = src[i][j];
//                    colCount++;
//                }
//            }
//            rowCount++;
//        }
//    }
//
//    return 1;
//}
//
//// Calculate the determinant recursively.
//double CalcDeterminant( float **mat, int order)
//{
//    // order must be >= 0
//    // stop the recursion when matrix is a single element
//    if( order == 1 )
//        return mat[0][0];
//
//    // the determinant value
//    float det = 0;
//
//    // allocate the cofactor matrix
//    float **minor;
//    minor = new float*[order-1];
//    for(int i=0;i<order-1;i++)
//        minor[i] = new float[order-1];
//
//    for(int i = 0; i < order; i++ )
//    {
//        // get minor of element (0,i)
//        GetMinor( mat, minor, 0, i , order);
//        // the recusion is here!
//
//        det += (i%2==1?-1.0:1.0) * mat[0][i] * CalcDeterminant(minor,order-1);
//        //det += pow( -1.0, i ) * mat[0][i] * CalcDeterminant( minor,order-1 );
//    }
//
//    // release memory
//    for(int i=0;i<order-1;i++)
//        delete [] minor[i];
//    delete [] minor;
//
//    return det;
//}
//
//
//
//void MatrixInversion(Matrix<double>& _A, Matrix<double>& invA)
//{
//	float **A = new float*[_A.getRowSize()];
//	float **Y = new float*[_A.getRowSize()];
//	for(int i = 0; i < _A.getRowSize(); i++){
//		A[i] = new float[_A.getColSize()];
//		Y[i] = new float[_A.getColSize()];
//	}
//	for(int i = 0; i < _A.getRowSize(); i++){
//		for(int j = 0; j < _A.getColSize(); j++){
//			A[i][j] = _A[i][j];
//		}
//	}
//	int order = _A.getRowSize();
//    // get the determinant of a
//    double det = 1.0/CalcDeterminant(A,order);
//
//    // memory allocation
//    float *temp = new float[(order-1)*(order-1)];
//    float **minor = new float*[order-1];
//    for(int i=0;i<order-1;i++)
//        minor[i] = temp+(i*(order-1));
//
//    for(int j=0;j<order;j++)
//    {
//        for(int i=0;i<order;i++)
//        {
//            // get the co-factor (matrix) of A(j,i)
//            GetMinor(A,minor,j,i,order);
//            Y[i][j] = det*CalcDeterminant(minor,order-1);
//            if( (i+j)%2 == 1)
//                Y[i][j] = -Y[i][j];
//        }
//    }
//
//    // release memory
//    //delete [] minor[0];
//    delete [] temp;
//    delete [] minor;
//
//
//	for(int i = 0; i < _A.getRowSize(); i++){
//		for(int j = 0; j < _A.getColSize(); j++){
//			invA[i][j] = Y[i][j];
//		}
//	}
//	for(int i = 0; i < _A.getRowSize(); i++){
//		delete [] A[i];
//		delete [] Y[i];
//	}
//	delete [] A;
//	delete [] Y;
//}
//
//bool MathAlgorithm::getInverseMatrix(Matrix<double>& A, Matrix<double>& invA)
//{
//	MatrixInversion(A, invA);
//	for(int i = 0; i < invA.getRowSize(); i++){
//		for(int j = 0; j < invA.getColSize(); j++){
//			if(invA[i][j] != invA[i][j]){
//				return false;
//			}
//		}
//	}
//	return true;
//}

void MathAlgorithm::EigenDecomposition(Matrix3d A, Matrix3d& D, Matrix3d& V)
{
	Eigen::EigenSolver<Matrix3d> es;
	es.compute(A);
	D = es.eigenvalues().real().asDiagonal();
	V = es.eigenvectors().real();

//	Matrix<double> a(A.getRowSize(), A.getColSize());
//	Matrix<double> q(A.getRowSize(), A.getColSize());
//	Matrix<double> Q_(A.getRowSize(), A.getColSize());
//	Matrix<double> R(A.getRowSize(), A.getColSize());
//	Q_.SetIdentity();
//	a = A;
//	q.SetIdentity();
//	bool isBreak = true;
//	for(int i = 0; i < A.getRowSize(); i++){
//		for(int j = 0; j < A.getColSize(); j++){
//			if(i != j){
//				if(A[i][j] != 0.0){
//					isBreak = false;
//				}
//			}
//		}
//	}
//	if(!isBreak){
//		for(int i = 0; i < 32; i++){
//			if (!QRDecompose(a, q, R))
//			{
//				return false;
//			}
//			a = R * q;
//			Q_ = Q_ * q;
//		}
//	}
//	Q = Q_;
//	D = a;
//	return true;
}

//typedef struct {
//	int m, n;
//	double ** v;
//} mat_t, mat;
//
//int count = 0;
//
//mat* matrix_new(int m, int n)
//{
//	mat* x = (mat*)malloc(sizeof(mat_t));
//	x->v = (double**)malloc(sizeof(double) * m);
//	x->v[0] = (double*)calloc(sizeof(double), m * n);
//	for (int i = 0; i < m; i++)
//		x->v[i] = x->v[0] + n * i;
//	x->m = m;
//	x->n = n;
//	return x;
//}
//
//void matrix_delete(mat* m)
//{
//	free(m->v[0]);
//	free(m->v);
//	free(m);
//}
//
//void matrix_transpose(mat* m)
//{
//	for (int i = 0; i < m->m; i++) {
//		for (int j = 0; j < i; j++) {
//			double t = m->v[i][j];
//			m->v[i][j] = m->v[j][i];
//			m->v[j][i] = t;
//		}
//	}
//}
//
//mat* matrix_copy(double** a, int m, int n)
//{
//	mat* x = matrix_new(m, n);
//	for (int i = 0; i < m; i++)
//		for (int j = 0; j < n; j++)
//			x->v[i][j] = a[i][j];
//	return x;
//}
//
//mat* matrix_mul(mat* x, mat* y)
//{
//	if (x->n != y->m) return 0;
//	mat* r = matrix_new(x->m, y->n);
//	for (int i = 0; i < x->m; i++)
//		for (int j = 0; j < y->n; j++)
//			for (int k = 0; k < x->n; k++)
//				r->v[i][j] += x->v[i][k] * y->v[k][j];
//	return r;
//}
//
//mat* matrix_minor(mat* x, int d)
//{
//	mat* m = matrix_new(x->m, x->n);
//	for (int i = 0; i < d; i++)
//		m->v[i][i] = 1;
//	for (int i = d; i < x->m; i++)
//		for (int j = d; j < x->n; j++)
//			m->v[i][j] = x->v[i][j];
//	return m;
//}
//
///* c = a + b * s */
//double *vmadd(double a[], double b[], double s, double c[], int n)
//{
//	for (int i = 0; i < n; i++)
//		c[i] = a[i] + s * b[i];
//	return c;
//}
//
///* m = I - v v^T */
//mat* vmul(double v[], int n)
//{
//	mat* x = matrix_new(n, n);
//	for (int i = 0; i < n; i++)
//		for (int j = 0; j < n; j++)
//			x->v[i][j] = -2 *  v[i] * v[j];
//	for (int i = 0; i < n; i++)
//		x->v[i][i] += 1;
//
//	return x;
//}
//
///* ||x|| */
//double vnorm(double x[], int n)
//{
//	double sum = 0;
//	for (int i = 0; i < n; i++) sum += x[i] * x[i];
//	return sqrt(sum);
//}
//
///* y = x / d */
//double* vdiv(double x[], double d, double y[], int n)
//{
//	for (int i = 0; i < n; i++) y[i] = x[i] / d;
//	return y;
//}
//
///* take c-th column of m, put in v */
//double* mcol(mat* m, double *v, int c)
//{
//	for (int i = 0; i < m->m; i++)
//		v[i] = m->v[i][c];
//	return v;
//}
//
//void householder(mat* m, Matrix<double>& Q, Matrix<double>& R)
//{
//	mat* _r, *_q;
//	mat* q[m->m];
//	mat* z = m, *z1;
//	for (int k = 0; k < m->n && k < m->m - 1; k++) {
//		double e[m->m], x[m->m], a;
//		z1 = matrix_minor(z, k);
//		if (z != m) matrix_delete(z);
//		z = z1;
//		mcol(z, x, k);
//		a = vnorm(x, m->m);
//		if (m->v[k][k] > 0) a = -a;
//		for (int i = 0; i < m->m; i++)
//			e[i] = (i == k) ? 1 : 0;
//
//		vmadd(x, e, a, e, m->m);
//		vdiv(e, vnorm(e, m->m), e, m->m);
//		q[k] = vmul(e, m->m);
//		z1 = matrix_mul(q[k], z);
//		if (z != m) matrix_delete(z);
//		z = z1;
//	}
//	matrix_delete(z);
//	_q = matrix_copy(q[0]->v, q[0]->m, q[0]->n);
//	for (int i = 1; i < m->n && i < m->m - 1; i++) {
//		z1 = matrix_mul(q[i], _q);
//		matrix_delete(_q);
//		_q = matrix_copy(z1->v, z1->m, z1->n);
//		matrix_delete(z1);
//		matrix_delete(q[i]);
//	}
//	matrix_delete(q[0]);
//	z = matrix_mul(_q, m);
//	_r = matrix_copy(z->v, z->m, z->n);
//	matrix_delete(z);
//	matrix_transpose(_q);
//	for(int i = 0; i < _q->m; i++){
//		for(int j = 0; j < _q->n; j++){
//			Q[i][j] = _q->v[i][j];
//		}
//	}
//	for(int i = 0; i < _r->m; i++){
//		for(int j = 0; j < _r->n; j++){
//			R[i][j] = _r->v[i][j];
//		}
//	}
//	matrix_delete(_q);
//	matrix_delete(_r);
//}
//
//bool MathAlgorithm::QRDecompose(Matrix<double>& A, Matrix<double>& Q, Matrix<double>& R){
//	mat* m = matrix_new(A.getRowSize(), A.getColSize());
//	for(int i = 0; i < m->m; i++){
//		for(int j = 0; j < m->n; j++){
//			m->v[i][j] = A[i][j];
//		}
//	}
//	householder(m, Q, R);
//	matrix_delete(m);
//}
