/*
 * HomogMatrix.hpp
 *
 *  Created on: 26.05.2015
 *  Author: tkornuta
 */

#ifndef HOMOGMATRIX_HPP_
#define HOMOGMATRIX_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <Eigen/LU>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <limits>

namespace Types {

/// Base HomogMatrix type.
typedef Eigen::Transform< double, 3, Eigen::Affine > HomogMatrixBaseType;
typedef Eigen::Transform< double, 3, Eigen::AffineCompact > CompactHomogMatrixBaseType;


/// Class representing homogenous matrix.
struct HomogMatrix : public HomogMatrixBaseType
{
	/// Base constructor - creates an identity matrix.
	HomogMatrix() : HomogMatrixBaseType ( HomogMatrixBaseType::Identity ())
	{
	}


	/// Constructor casting the OpenCv cv::Mat to HomogMatrix Eigen::Transform (with doubles).
	HomogMatrix(cv::Mat mat_) {
		cv::Matx44d m = mat_;
		*this = HomogMatrix(m);

	}

	/// Constructor casting the 4x4 OpenCv matrix Matx44d to HomogMatrix Eigen::Transform (with doubles).
	HomogMatrix(cv::Matx44d mat_) {
	
		// Check proper sizes of cv::Mat  -- not required as strong typing is used.
		//assert ( (mat_.rows() == 4) && (mat_.cols() == 4)) || 16x1 || 1x16

		// Copy values from cv::mat.
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j)
				matrix()(i,j) = mat_(i,j);
	}

	/// Constructor casting Eigen::Transform with floats to HomogMatrix Eigen::Transform (with doubles). The conversion is done with the use of Rodrigues (forward and inverse) transform.
	HomogMatrix(Eigen::Matrix4f mat_)
    {
		cv::Mat_<double> rotationMatrix_in = cv::Mat_<double>::zeros(3,3);
        cv::Mat_<double> rotation;

		// Copy input values to temporary matrix.
		for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
				rotationMatrix_in(i,j)=static_cast<double>(mat_(i, j));
            }
            matrix()(i,3) = static_cast<double>(mat_(i, 3));
        }

		// Compute rotation vector.
		Rodrigues(rotationMatrix_in, rotation);
        cv::Mat_<double> rotationMatrixd;
		// Transform resulting rotation vector into matrix<double>.
		Rodrigues(rotation, rotationMatrixd);

		// Copy values to output matrix.
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                matrix()(i,j)=rotationMatrixd(i, j);
            }
        }
    }

	HomogMatrix(const CompactHomogMatrixBaseType & mat_) :  HomogMatrixBaseType ( HomogMatrixBaseType::Identity ())
	{
		// Copy (overwrite) values from Eigen::Transform< double, 3, Eigen::AffineCompact >.
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 4; ++j) 
				matrix()(i,j) = mat_(i,j);
	}


	/// Constructor casting the Eigen 4x4 matrix with float to HomogMatrix (Eigen::Transform with doubles).
//	HomogMatrix(Eigen::Matrix<float, 4, 4> mat_) {
//		// Copy values from matrix.
//		for (int i = 0; i < 4; ++i)
//			for (int j = 0; j < 4; ++j)
//				matrix()(i,j) = mat_(i,j);
//	}
	

	/// Method casts the HomogMatrix (Eigen matrix Matrix4d) to Eigen::Affine3f.
	operator Eigen::Affine3f const()
	{
		Eigen::Affine3f mat;
		// Copy values.
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j) 
				mat(i,j) = matrix()(i,j);
//		std::cout<<"CAST! Input hm = \n" << matrix() << "\n Output aff3f =\n" << mat.matrix();
		return mat;
	}


	/// Method casts Eigen::Affine3f to HomogMatrix.
	HomogMatrix & operator = (const Eigen::Affine3f & aff3f_)
	{
		// Copy values from HM.
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j) 
				matrix()(i,j) = aff3f_(i,j);
//		std::cout<<"CAST! Input aff3f =\n" << aff3f_.matrix() <<"\n Output hm = \n" << matrix();
		return *this;
	}


	/// Method casts the HomogMatrix (Eigen matrix Matrix4d) to 4x4 OpenCv matrix Matx44d.
	operator cv::Matx44d const()
	{
		cv::Matx44d mat;
		// Copy values to cv::mat.
		for (int i = 0; i < 4; ++i)
			for (int j = 0; j < 4; ++j) 
				mat(i,j) = matrix()(i,j);
		return mat;
	}


	/// Method casts the HomogMatrix (tHomogMatrixBaseType) to CompactHomogMatrixBaseType.
	HomogMatrix & operator = (const CompactHomogMatrixBaseType & hm_)
	{
		*this = HomogMatrixBaseType::Identity ();
		// Copy values from HM.
		for (int i = 0; i < 3; ++i)
			for (int j = 0; j < 4; ++j) 
				matrix()(i,j) = hm_(i,j);
		return *this;
	}


	/// Set transform on the basis of XYZ and RPY angles - arguments passed in a single vector.
	void setFromXYZRPY(cv::Vec6d vec_)
	{
		setFromXYZRPY(vec_[0], vec_[1], vec_[2], vec_[3], vec_[4], vec_[5]);
	}

	/// Set transform on the basis of XYZ and RPY angles.
	void setFromXYZRPY(double x, double y, double z, double roll, double pitch, double yaw)
	{
		// Set translation.
		Eigen::Vector3d t;
		t << x,y,z;
		this->translation() = t;

		// Set rotation (called linear part in Eigen:]).
		Eigen::Matrix3d m;
		m = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
		this->linear() = m;
	}



	/// Redirect the output stream.
	inline friend ostream & operator<< (ostream &out_, HomogMatrix &hm_) {
		cv::Matx44d tmp = hm_;
		return out_ << tmp;
	}

	/// Checks whether matrices are similar - returns true if distance is smaller than eps (set to 1e-5 as default).
	bool isSimilar(const HomogMatrixBaseType & hm_, double eps = 1e-5) {
		return (this->matrix()-hm_.matrix()).isMuchSmallerThan(eps);
	}


	/// Checks whether matrix is identity matrix - returns true if distance is smaller than eps (set to 1e-5 as default).
	bool isIdentity(double eps = 1e-5) {
		return isSimilar(HomogMatrixBaseType::Identity(), eps);
	}

};




} // namespace Types

#endif /* HOMOGMATRIX_HPP_ */
