#ifndef PSEUDO_INVERSION_H
#define PSEUDO_INVERSION_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
using namespace Eigen;

inline void pseudo_inverse(const Eigen::MatrixXd &M_, Eigen::MatrixXd &M_pinv_,bool damped = true)
{	
	double lambda_ = damped?0.0001:0.0;

	JacobiSVD<MatrixXd> svd(M_, ComputeFullU | ComputeFullV);
	JacobiSVD<MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
	MatrixXd S_ = M_;	// copying the dimensions of M_, its content is not needed.
	S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i,i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_*lambda_);

    M_pinv_ = MatrixXd(svd.matrixV()*S_.transpose()*svd.matrixU().transpose());
}

#endif