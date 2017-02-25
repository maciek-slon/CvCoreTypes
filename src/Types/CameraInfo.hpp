/*
 * CameraInfo.hpp
 *
 *  Created on: 22-11-2012
 *      Author: mstefanc
 */

#ifndef CAMERAINFO_HPP_
#define CAMERAINFO_HPP_

#include <opencv2/core/core.hpp>

#include <boost/array.hpp>

namespace Types {

class CameraInfo {
public:
	CameraInfo(int w = 640, int h = 480, float cx = 320, float cy = 240, float fx = 1, float fy = 1) :
		m_width(w), m_height(h),
        m_camera_matrix(cv::Mat(3, 3, CV_32F)),
        m_dist_coeffs(cv::Mat(1, 5, CV_32F)),
        m_proj_matrix(cv::Mat(3, 4, CV_32F)),
        m_rectif_matrix(cv::Mat(3, 3, CV_32F)),
        m_rotation_matrix(cv::Mat(3, 3, CV_32F)),
        m_translation_matrix(cv::Mat(3, 1, CV_32F))
	{
		setCx(cx);
		setCy(cy);
		setFx(fx);
		setFy(fy);
	}

	float cx() const {
		return m_camera_matrix.at<float>(0, 2);
	}

	void setCx(float cx) {
		m_camera_matrix.at<float>(0, 2) = cx;
	}

	float cy() const {
		return m_camera_matrix.at<float>(1, 2);
	}

	void setCy(float cy) {
		m_camera_matrix.at<float>(1, 2) = cy;
	}

	float fx() const {
		return m_camera_matrix.at<float>(0, 0);
	}

	void setFx(float fx) {
		m_camera_matrix.at<float>(0, 0) = fx;
	}

	float fy() const {
		return m_camera_matrix.at<float>(1, 1);
	}

	void setFy(float fy) {
		m_camera_matrix.at<float>(1, 1) = fy;
	}

	int height() const {
		return m_height;
	}

	void setHeight(int height) {
		m_height = height;
	}

	int width() const {
		return m_width;
	}

	void setWidth(int width) {
		m_width = width;
	}

	cv::Size size() {
		return cv::Size(m_width, m_height);
	}

	void setSize(const cv::Size & s) {
		m_width = s.width;
		m_height = s.height;
	}

	cv::Mat cameraMatrix() const {
		return m_camera_matrix;
	}

	void setCameraMatrix(const cv::Mat mat) {
		m_camera_matrix = mat.clone();
	}

	cv::Mat distCoeffs() const {
		return m_dist_coeffs;
	}

	void setDistCoeffs(const cv::Mat mat) {
		m_dist_coeffs = mat.clone();
	}
	
	template<typename T, std::size_t N>
	void setDistCoeffs(boost::array<T, N> arr) {
		for (int i = 0; i < 5; ++i)
			m_dist_coeffs.at<float>(0, i) = arr[i];
	}

    cv::Mat projectionMatrix() const {
        return m_proj_matrix;
    }

    void setProjectionMatrix(const cv::Mat mat) {
        m_proj_matrix = mat.clone();
    }

    cv::Mat rectificationMatrix() const {
        return m_rectif_matrix;
    }

    void setRectificationMatrix(const cv::Mat mat) {
        m_rectif_matrix = mat.clone();
    }

    cv::Mat rotationMatrix() const {
        return m_rotation_matrix;
    }

    void setRotationMatrix(const cv::Mat mat) {
        m_rotation_matrix = mat.clone();
    }

    cv::Mat translationMatrix() const {
        return m_translation_matrix;
    }

    void setTranlationMatrix(const cv::Mat mat) {
        m_translation_matrix = mat.clone();
    }
    
    bool operator== (const CameraInfo & rhs) {
	bool ret = (m_width == rhs.m_width) && (m_height == rhs.m_height);
	ret = ret && cmpMat(m_camera_matrix, rhs.m_camera_matrix);
	ret = ret && cmpMat(m_dist_coeffs, rhs.m_dist_coeffs);
	ret = ret && cmpMat(m_proj_matrix, rhs.m_proj_matrix);
	ret = ret && cmpMat(m_rectif_matrix, rhs.m_rectif_matrix);
	ret = ret && cmpMat(m_rotation_matrix, rhs.m_rotation_matrix);
	ret = ret && cmpMat(m_translation_matrix, rhs.m_translation_matrix);
	
	return ret;
    }
    
    bool operator!= (const CameraInfo & rhs) {
	return ! ( (*this) == rhs );
    }
    
    bool cmpMat(cv::Mat m1, cv::Mat m2) {
	if (m1.size() != m2.size()) return false;
	if (m1.type() != m2.type()) return false;
	//cv::Mat diff; // = m1;
	//cv::compare(m1, m2, diff, cv::CMP_NE);
	cv::Mat diff = m1 != m2;
	int nz = cv::countNonZero(diff);
	return (nz <= 0);
    }


private:
	int m_width;
	int m_height;

	cv::Mat m_camera_matrix;
	cv::Mat m_dist_coeffs;
	cv::Mat m_proj_matrix;
	cv::Mat m_rectif_matrix;
	cv::Mat m_rotation_matrix;
	cv::Mat m_translation_matrix;

};

}

#endif /* CAMERAINFO_HPP_ */
