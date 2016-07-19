/*!
 * \file HomogenousMatrixSequence.cpp
 * \brief Class responsible for retrieving vector of homogenous matrices - methods definition.
 * \author Tomek Kornuta, tkornuta@gmail.com
 */

#include "HomogenousMatrixSequence.hpp"

#include <opencv2/highgui/highgui.hpp>

namespace Sources {
namespace HomogenousMatrixSequence {

HomogenousMatrixSequence::HomogenousMatrixSequence(const std::string & n) :
	Base::Component(n),
	prop_filename("filename", std::string("")),
	prop_loop("mode.loop", false),
	prop_read_on_init("mode.read_on_init", true),
	prop_auto_publish("mode.auto_publish", true),
	prop_auto_next("mode.auto_next", true),
	prop_auto_prev("mode.auto_prev", false)
{
	registerProperty(prop_filename);
	registerProperty(prop_read_on_init);
	registerProperty(prop_loop);
	registerProperty(prop_auto_publish);
	registerProperty(prop_auto_next);
	registerProperty(prop_auto_prev);

	CLOG(LTRACE) << "Constructed";
}

HomogenousMatrixSequence::~HomogenousMatrixSequence() {
	CLOG(LTRACE) << "Destroyed";
}


void HomogenousMatrixSequence::prepareInterface() {
	// Register streams.
	registerStream("out_homogMatrix", &out_homogMatrix);
	registerStream("out_end_of_sequence_trigger", &out_end_of_sequence_trigger);
	registerStream("in_publish_trigger", &in_publish_trigger);
	registerStream("in_next_trigger", &in_next_trigger);
	registerStream("in_prev_trigger", &in_prev_trigger);

	// Register handlers - loads image, NULL dependency.
	registerHandler("onLoad", boost::bind(&HomogenousMatrixSequence::onLoad, this));
	addDependency("onLoad", NULL);

	// Register handlers - next matrix, can be triggered manually (from GUI) or by new data present in_load_next_trigger dataport.
	registerHandler("Next", boost::bind(&HomogenousMatrixSequence::onLoadNext, this));
	addDependency("Next", &in_next_trigger);

	// Register handlers - prev matrix, can be triggered manually (from GUI) or by new data present in_load_next_trigger dataport.
	registerHandler("Previous", boost::bind(&HomogenousMatrixSequence::onLoadPrev, this));
	addDependency("Previous", &in_prev_trigger);

	// Register handlers - reloads HomogenousMatrixSequence, triggered manually.
	registerHandler("Reload Sequence", boost::bind(&HomogenousMatrixSequence::onSequenceReload, this));

	registerHandler("Publish", boost::bind(&HomogenousMatrixSequence::onPublish, this));
	addDependency("Publish", &in_publish_trigger);

}

bool HomogenousMatrixSequence::onInit() {
	CLOG(LTRACE) << "initialize\n";

	// Set indices.
	index = 0;
	previous_index = -1;

	// Initialize flags.
	next_flag = false;
	prev_flag = false;
	reload_flag = true;

	return true;
}

bool HomogenousMatrixSequence::onFinish() {
	CLOG(LTRACE) << "onFinish";
	return true;
}

void HomogenousMatrixSequence::onPublish() {
	CLOG(LTRACE) << "onPublish";

	if (!in_publish_trigger.empty())
		in_publish_trigger.read();

	publish_flag = true;
}

void HomogenousMatrixSequence::onLoad() {
	CLOG(LTRACE) << "onLoad";

	CLOG(LDEBUG) << "Before index=" << index << " previous_index=" << previous_index;

	
	if(reload_flag) {
		// Reload the sequence.

		try {
			cv::FileStorage fs(prop_filename(), cv::FileStorage::READ);
			fs["XYZRPY"] >> matrices;
			CLOG(LDEBUG) << "Loaded matrix of XYZRPY Hm's:\n" << matrices;

		} catch(...) {
			CLOG(LERROR) << "Could not load matrix of XYZRPY  from file: " << prop_filename;
		}//: catch

		// Reset index and flag.
		index = 0;
		reload_flag = false;
	} else if (previous_index == -1) {
		// Special case - start!
			index = 0;
	} else {
		// Check triggering mode.
		if ((prop_auto_next) || (next_flag)) {
			index++;
		}//: if

		if ((prop_auto_prev) || (prev_flag)) {
			index--;
		}//: if

		// Anyway, reset flags.
		next_flag = false;
		prev_flag = false;

		// Check index range - first matrix.
		if (index <0){
			out_end_of_sequence_trigger.write(Base::UnitType());
			if (prop_loop) {
				index = matrices.rows -1;
				CLOG(LDEBUG) << "Loop";
			} else {
				// Sequence ended - truncate index, but do not return matrix.
				index = 0;
				CLOG(LINFO) << "End of sequence";
				return;
			}//: else
		}//: if

		// Check index range - last image.
		if (index >= matrices.rows) {
			out_end_of_sequence_trigger.write(Base::UnitType());
			CLOG(LINFO) << "End of HomogenousMatrixSequence";
			if (prop_loop) {
				index = 0;
				CLOG(LINFO) << "loop";
			} else {
				// Sequence ended - truncate index, but do not return matrix.
				index = matrices.rows -1;
				return;
			}//: else
		}//: if
	}//: else

	// Check whether there are any matrices loaded.
	if(matrices.rows == 0){
		CLOG(LNOTICE) << "Empty sequence!";
		return;
	}//: else

	// Check publishing flags.
	if(!prop_auto_publish && !publish_flag)
		return;
	publish_flag = false;

	CLOG(LDEBUG) << "After: index=" << index << " previous_index=" << previous_index ;

	try {
		cv::Vec6d* hm_vector = matrices.ptr<cv::Vec6d>(index);
		//cv::Mat hm_vector = matrices.row(index);
		CLOG(LDEBUG) << "Returning matrix (" << index << "): " <<  (*hm_vector);

		// Create matrix from XYZ and RPY angles.
		Types::HomogMatrix hm;
		hm.setFromXYZRPY(*hm_vector);

		CLOG(LDEBUG) <<"Returned matrix:\n"<<hm;
		previous_index = index;
		// Write to the output port.
		out_homogMatrix.write(hm);

	} catch (...) {
		CLOG(LWARNING) << "Publish failed on index " << index;
	}//: catch

}



void HomogenousMatrixSequence::onLoadNext(){
	CLOG(LDEBUG) << "onLoadNext - next matrix will be loaded";
	if(!in_next_trigger.empty())
		in_next_trigger.read();
	next_flag = true;
}


void HomogenousMatrixSequence::onSequenceReload() {
	CLOG(LDEBUG) << "onSequenceReload";
	reload_flag = true;
}



void HomogenousMatrixSequence::onLoadPrev(){
	CLOG(LDEBUG) << "onLoadPrev - prev matrix will be loaded";
	if(!in_prev_trigger.empty())
		in_prev_trigger.read();
	prev_flag = true;
}


bool HomogenousMatrixSequence::onStart() {
	return true;
}

bool HomogenousMatrixSequence::onStop() {
	return true;
}


}//: namespace HomogenousMatrixSequence
}//: namespace Sources
