/*
 * HomogenousMatrixProvider.cpp
 *
 *  Created on: 13-08-2014
 *      Author: lzmuda
 */

#include "HomogenousMatrixProvider.hpp"
#include "Common/Logger.hpp"

#include <sstream>
#include "Base/Property.hpp"
#include <boost/foreach.hpp>

namespace Processors {

namespace HomogenousMatrixProvider {

using namespace std;
using namespace cv;
using namespace boost;
using Types::HomogMatrix;

HomogenousMatrixProvider::HomogenousMatrixProvider(const std::string & n) :
	Component(n),
	prop_x("offset.x", 0),
	prop_y("offset.y", 0),
	prop_z("offset.z", 0),
	prop_roll("offset.roll", 0),
	prop_pitch("offset.pitch", 0),
	prop_yaw("offset.yaw", 0)
	{
		registerProperty(prop_x);
		registerProperty(prop_y);
		registerProperty(prop_z);
		registerProperty(prop_roll);
		registerProperty(prop_pitch);
		registerProperty(prop_yaw);
}

HomogenousMatrixProvider::~HomogenousMatrixProvider()
{
}

void HomogenousMatrixProvider::prepareInterface() {
	// Register the output stream.
	registerStream("out_homogMatrix", &out_homogMatrix);

	// Register the default handler, activated in every step.
	registerHandler("generateHomogenousMatrix", boost::bind(&HomogenousMatrixProvider::generateHomogenousMatrix,this));
	addDependency("generateHomogenousMatrix", NULL);
}

bool HomogenousMatrixProvider::onStart()
{
	LOG(LTRACE) << "onStart()";
	return true;
}

bool HomogenousMatrixProvider::onStop()
{
	LOG(LTRACE) << "onStop()";
	return true;
}

bool HomogenousMatrixProvider::onInit()
{
	LOG(LTRACE) << "onInit()";


	return true;
}

bool HomogenousMatrixProvider::onFinish()
{
	LOG(LTRACE) << "onFinish()";
	return true;
}

bool HomogenousMatrixProvider::onStep()
{
	LOG(LTRACE) << "onStep()";
	return true;
}

void HomogenousMatrixProvider::generateHomogenousMatrix()
{
	CLOG(LTRACE) << "generateHomogenousMatrix()";

//	Mat_<double> rvec;
//	Mat_<double> tvec;

	// Create homogenous matrix.
//    Mat_<double> rotationMatrix (3,3);
    //Rodrigues(rvec, rotationMatrix);
/*
	// Roll - rotation around the X (blue) axis.
	cv::Matx44d roll;
	roll <<
		1,          0,           0, 0,
		0, cos(prop_roll), -sin(prop_roll), 0,
		0, sin(prop_roll),  cos(prop_roll), 0,
		0, 0, 0, 1 ;

	CLOG(LDEBUG) << "roll (Matx44d):\n" << roll;

	// Pitch - rotation around the Y (green) axis.
	cv::Matx44d pitch;
	pitch <<
		cos(prop_pitch), 0, sin(prop_pitch), 0,
		0, 1, 0, 0,
		-sin(prop_pitch),  0,	cos(prop_pitch), 0,
		0, 0, 0, 1 ;
	CLOG(LDEBUG) << "pitch (Matx44d):\n" << pitch;

	// Yaw - rotation around the Z (red) axis.
	cv::Matx44d yaw;
	yaw <<
		cos(prop_yaw), -sin(prop_yaw), 0, 0,
		sin(prop_yaw),  cos(prop_yaw), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	CLOG(LDEBUG) << "yaw (Matx44d):\n" << yaw;

	// translation
	cv::Matx44d t;
	t <<
		0, 0, 0, prop_x,
		0, 0, 0, prop_y,
		0, 0, 0, prop_z,
		0, 0, 0, 0;
	CLOG(LDEBUG) << "trans (Matx44d):\n" << t;

	// Build the whole transform.
	cv::Matx44d outputMatrix = t + yaw * pitch * roll;


	CLOG(LDEBUG) << "output (Matx44d):\n" << outputMatrix;
*/
	// Create matrix from XYZ and RPY angles.
	HomogMatrix hm;
	hm.setFromXYZRPY(prop_x, prop_y, prop_z, prop_roll, prop_pitch, prop_yaw);

	// Debug: display created matrix.
	CLOG(LDEBUG) << "HM (HomogMatrix):\n" << hm;

	out_homogMatrix.write(hm);
}

} // namespace HomogenousMatrixProvider

} // namespace Processors
