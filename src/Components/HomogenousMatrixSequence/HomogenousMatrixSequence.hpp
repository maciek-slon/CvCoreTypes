/*!
 * \file HomogenousMatrixSequence.hpp
 * \brief Class responsible for retrieving vector of homogenous matrices - class declaration.
 */


#ifndef HomogenousMatrixSequence_HPP_
#define HomogenousMatrixSequence_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include "Types/HomogMatrix.hpp"

//#include <vector>
//#include <string>

#include <opencv2/core/core.hpp>

/**
 * \defgroup HomogenousMatrixSequence HomogenousMatrixSequence
 *
 * \brief HomogenousMatrix sequence provider.
 */

namespace Sources {
namespace HomogenousMatrixSequence {

/*!
 * \class HomogenousMatrixSequence
 * \brief Class responsible for retrieving vector of homogenous matrices.
 */
class HomogenousMatrixSequence : public Base::Component {

public:
	/*!
	 * Constructor. Sets ID and startup variables.
	 */
	HomogenousMatrixSequence(const std::string & name = "HomogenousMatrixSequence");

	/*!
	 * Destructor.
	 */
	virtual ~HomogenousMatrixSequence();

	virtual void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

	/// Trigger - used for hm publishing.
	Base::DataStreamIn<Base::UnitType, Base::DataStreamBuffer::Newest> in_publish_trigger;

	/// Trigger - used for loading next matrix.
	Base::DataStreamIn<Base::UnitType, Base::DataStreamBuffer::Newest> in_next_trigger;

	/// Trigger - used for loading previous matrix.
	Base::DataStreamIn<Base::UnitType, Base::DataStreamBuffer::Newest> in_prev_trigger;

	/// Output data stream
	Base::DataStreamOut <Types::HomogMatrix> out_homogMatrix;

	/// Output event - sequence ended.
	Base::DataStreamOut<Base::UnitType> out_end_of_sequence_trigger;

	/*!
	* Event handler function - moves index to next matrix from sequence.
	*/
	void onLoadNext();

	/*!
	* Event handler function - moves index to previous matrix from sequence.
	*/
	void onLoadPrev();


	/*!
	 * Event handler function - loads next matrix from sequence.
	 */
	void onLoad();

	/*!
	 * Event handler function - reload the sequence.
	 */
	void onSequenceReload();

	/*!
	* Event handler function - publishes matrix.
	*/
	void onPublish();

private:
	/// Matrix containing list of homogenous matrices - each vector represents a single HM in the form of XYZRPY.
	cv::Mat matrices;


	/// Index of current matrix.
	int index;

	/// Index of matrix returned in the previous step.
	int previous_index;


	/// Flag indicating whether the matrix should be published.
	bool publish_flag;

	/// Flag indicating whether the next matrix should loaded or not.
	bool next_flag;

	/// Flag indicating whether the previous matrix should loaded or not.
	bool prev_flag;

	/// Flag indicating whether the sequence should be reloaded or not.
	bool reload_flag;


	/// File containing the vector of matrices (in the form of matrix, each row containing one HM in the form of XYZRPY).
	Base::Property<std::string> prop_filename;

	/// Publish mode: auto vs triggered.
	Base::Property<bool> prop_auto_publish;

	/// Next matrix loading mode: next vs triggered
	Base::Property<bool> prop_auto_next;

	/// Prev image  loading mode: previous vs triggered
	Base::Property<bool> prop_auto_prev;

	/// Loading mode: images loaded in the loop.
	Base::Property<bool> prop_loop;

	/// Loads whole sequence at start.
	Base::Property<bool> prop_read_on_init;

};

}//: namespace HomogenousMatrixSequence
}//: namespace Sources

/*
 * Register source component.
 */
REGISTER_COMPONENT("HomogenousMatrixSequence", Sources::HomogenousMatrixSequence::HomogenousMatrixSequence)

#endif /* HomogenousMatrixSequence_HPP_ */
