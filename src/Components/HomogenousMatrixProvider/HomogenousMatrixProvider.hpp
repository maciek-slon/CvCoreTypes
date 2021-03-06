/*!
 * \file CvSolvePnP_Processor.hpp
 * \brief Position estimation component.
 * \date Oct 20, 2010
 * \author mboryn
 */

#ifndef HOMOGONEUSMATRIXPROVIDE_HPP_
#define HOMOGONEUSMATRIXPROVIDE_HPP_

#include "Base/Component_Aux.hpp"
#include "Base/Component.hpp"
#include "Base/EventHandler.hpp"
#include "Base/DataStream.hpp"
#include "Base/Property.hpp"

#include "Types/HomogMatrix.hpp"

#include <opencv2/core/core.hpp>



namespace Processors {
namespace HomogenousMatrixProvider {

class HomogenousMatrixProvider: public Base::Component
{
public:
	HomogenousMatrixProvider(const std::string & n);
	virtual ~HomogenousMatrixProvider();

	void prepareInterface();
protected:
	/*!
	 * Method called when component is started
	 * \return true on success
	 */
	virtual bool onStart();

	/*!
	 * Method called when component is stopped
	 * \return true on success
	 */
	virtual bool onStop();

	/*!
	 * Method called when component is initialized
	 * \return true on success
	 */
	virtual bool onInit();

	/*!
	 * Method called when component is finished
	 * \return true on success
	 */
	virtual bool onFinish();

	/*!
	 * Method called when step is called
	 * \return true on success
	 */
	virtual bool onStep();

private:

	void generateHomogenousMatrix();

	Base::Property<double> prop_x;
	Base::Property<double> prop_y;
	Base::Property<double> prop_z;
	Base::Property<double> prop_roll;
	Base::Property<double> prop_pitch;
	Base::Property<double> prop_yaw;


	Base::DataStreamOut <Types::HomogMatrix> out_homogMatrix;

	Base::EventHandler <HomogenousMatrixProvider> h_generateHomogenousMatrix;
};

} // namespace HomogenousMatrixProvider

} // namespace Processors

REGISTER_COMPONENT("HomogenousMatrixProvider", Processors::HomogenousMatrixProvider::HomogenousMatrixProvider)

#endif /* HomogenousMatrixProvide_HPP_ */
