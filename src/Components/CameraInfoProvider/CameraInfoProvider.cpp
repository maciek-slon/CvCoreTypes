/*!
 * \file
 * \brief
 * \author Maciej Stefańczyk [maciek.slon@gmail.com]
 */

#include <memory>
#include <string>

#include "CameraInfoProvider.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace CameraInfoProvider {

CameraInfoProvider::CameraInfoProvider(const std::string & name) :
		Base::Component(name), width("width", 640), height("height", 480),
		camera_matrix("camera_matrix", cv::Mat(cv::Mat::eye(3, 3, CV_32FC1))),
		dist_coeffs("dist_coeffs", cv::Mat(cv::Mat::zeros(1, 5, CV_32FC1))),
		rectificaton_matrix("rectificaton_matrix", cv::Mat(cv::Mat::eye(3, 3, CV_32FC1))),
		projection_matrix("projection_matrix", cv::Mat(cv::Mat::zeros(3, 4, CV_32FC1))),
		rotation_matrix("rotation_matrix", cv::Mat(cv::Mat::eye(3, 3, CV_32FC1))),
		translation_matrix("translation_matrix", cv::Mat(cv::Mat::zeros(3, 1, CV_32FC1))),
		data_file("data_file", std::string(""))
{
	width.addConstraint("1");
	width.addConstraint("9999");
	registerProperty(width);

	height.addConstraint("1");
	height.addConstraint("9999");
	registerProperty(height);

	registerProperty(camera_matrix);
	
	registerProperty(dist_coeffs);
	dist_coeffs.setCallback(boost::bind(&CameraInfoProvider::distCallback, this, _1, _2));
	
	registerProperty(rectificaton_matrix);
	registerProperty(projection_matrix);
	registerProperty(rotation_matrix);
	registerProperty(translation_matrix);
	registerProperty(data_file);
}

CameraInfoProvider::~CameraInfoProvider() {

}

void CameraInfoProvider::prepareInterface() {
	// Register data streams.
	registerStream("out_camera_info", &out_camerainfo);
	registerStream("in_camera_info", &in_camerainfo);

	//"Generate data" handler.
	registerHandler("generate_data", boost::bind(&CameraInfoProvider::generate_data, this));
	addDependency("generate_data", NULL);

	//"Reload file" handler.
	registerHandler("reload_file", boost::bind(&CameraInfoProvider::reload_file, this));
	
	//"Save file" handler.
	registerHandler("save_file", boost::bind(&CameraInfoProvider::save_file, this));

	//"Update params" handler.
	registerHandler("update_params", boost::bind(&CameraInfoProvider::update_params, this));
	addDependency("update_params", &in_camerainfo);
}

bool CameraInfoProvider::onInit() {
	if (data_file != "") {
		CLOG(LINFO) << "reload_file";
		reload_file();
	}

	return true;
}

bool CameraInfoProvider::onFinish() {
	return true;
}

bool CameraInfoProvider::onStop() {
	return true;
}

bool CameraInfoProvider::onStart() {
	return true;
}

void CameraInfoProvider::generate_data() {
	CLOG(LDEBUG) << "setWidth " << width;
	camera_info.setWidth(width);
	CLOG(LDEBUG) << "setHeight " << height;
	camera_info.setHeight(height);
	CLOG(LDEBUG) << "setCameraMatrix " << camera_matrix;
	camera_info.setCameraMatrix(camera_matrix);
	CLOG(LDEBUG) << "setDistCoeffs " << dist_coeffs;
	camera_info.setDistCoeffs(dist_coeffs);
	CLOG(LDEBUG) << "setRectificationMatrix " << rectificaton_matrix;
	camera_info.setRectificationMatrix(rectificaton_matrix);
	CLOG(LDEBUG) << "setProjectionMatrix " << projection_matrix;
	camera_info.setProjectionMatrix(projection_matrix);
	CLOG(LDEBUG) << "setRotationMatrix " << rotation_matrix;
	camera_info.setRotationMatrix(rotation_matrix);
	CLOG(LDEBUG) << "setTranlationMatrix " << translation_matrix;
	camera_info.setTranlationMatrix(translation_matrix);
	CLOG(LDEBUG) << "write";

	out_camerainfo.write(camera_info);
}

void CameraInfoProvider::update_params() {
	Types::CameraInfo camera_info = in_camerainfo.read();
	width = camera_info.width();
	height = camera_info.height();
	camera_matrix = camera_info.cameraMatrix();
	dist_coeffs = camera_info.distCoeffs();
	rectificaton_matrix = camera_info.rectificationMatrix();
	projection_matrix = camera_info.projectionMatrix();
	rotation_matrix = camera_info.rotationMatrix();
	translation_matrix = camera_info.translationMatrix();
}

void CameraInfoProvider::reload_file() {
	CLOG(LDEBUG) << "Loading from " << data_file;
	cv::FileStorage fs(data_file(), cv::FileStorage::READ);
	if (!fs.isOpened()) {
		CLOG(LERROR) << "Can't open file [" << data_file << "]";
		return;
	}
	cv::Mat oTempMat;
	try {
		fs["M"] >> oTempMat;
		camera_matrix = oTempMat;
	} catch (...) {
		CLOG(LWARNING) << "No camera matrix in " << data_file;
	}
	try {
		fs["D"] >> oTempMat;
		dist_coeffs = oTempMat;
	} catch (...) {
		CLOG(LWARNING) << "No distortion coefficients in " << data_file;
	}
	try {
		fs["R"] >> oTempMat;
		rectificaton_matrix = oTempMat;
	} catch (...) {
		CLOG(LWARNING) << "No rectificaton matrix in " << data_file;
	}
	try {
		fs["P"] >> oTempMat;
		projection_matrix = oTempMat;
	} catch (...) {
		CLOG(LWARNING) << "No projection matrix in " << data_file;
	}
	try {
		fs["ROT"] >> oTempMat;
		rotation_matrix = oTempMat;
	} catch (...) {
		CLOG(LWARNING) << "No rotation matrix in " << data_file;
	}
	try {
		fs["T"] >> oTempMat;
		translation_matrix = oTempMat;
	} catch (...) {
		CLOG(LWARNING) << "No translation matrix in " << data_file;
	}
	fs.release();	
}

void CameraInfoProvider::save_file() {
	CLOG(LDEBUG) << "Saving to " << data_file;
	cv::FileStorage fs(data_file(), cv::FileStorage::WRITE);
	if (!fs.isOpened()) {
		CLOG(LERROR) << "Can't open file [" << data_file << "]";
		return;
	}
	fs << "M" << camera_matrix();
	fs << "D" << dist_coeffs();
	fs << "R" << rectificaton_matrix();
	fs << "P" << projection_matrix();
	fs << "ROT" << rotation_matrix();
	fs << "T" << translation_matrix();
	fs.release();	
}

void CameraInfoProvider::distCallback(cv::Mat old_value, cv::Mat new_value) {
	int cnt = 0;
	if (new_value.size().width == 1) {
		cnt = new_value.size().height;
	} else if (new_value.size().height == 1) {
		cnt = new_value.size().width;
	} else {
		CLOG(LWARNING) << "New value for distortion has wrong shape (" << new_value.size().height << "x" << new_value.size().width << "). Should be 1xN or Nx1 vector.";
		dist_coeffs = old_value.clone();
		return;
	}
	
	if ( (cnt != 4) && (cnt != 5) && (cnt != 8) && (cnt != 12) && (cnt != 14) ) {
		CLOG(LWARNING) << "New value for distortion has wrong size (" << cnt << "). Should contain 4, 5, 8, 12 or 14 values.";
		dist_coeffs = old_value.clone();
	}
}

} //: namespace CameraInfoProvider
} //: namespace Processors
