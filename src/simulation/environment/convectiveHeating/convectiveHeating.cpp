/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */
// #include "moduleTemplates/cppModuleTemplate/cppModuleTemplate.h"
#include <iostream>
#include "architecture/utilities/linearAlgebra.h"
#include "architecture/utilities/astroConstants.h"
#include "convectiveHeating.h"
#include <math.h>       

/*! This is the constructor for the module class.  It sets default variable
    values and initializes the various parts of the model */
ConvectiveHeating::ConvectiveHeating()
//! - Set the default convective heating properties to yield a zero response
{
    this->coreParams.suttonGravesConstant = 0.0;
    this->coreParams.noseRadius = 0.0;
    this->v_B.fill(0.0);
    this->heatFlux = 0.0;
}

/*! Module Destructor.  */
ConvectiveHeating::~ConvectiveHeating()
{
    return;
}


/*! This method is used to reset the module.
    @return void
 */
void ConvectiveHeating::Reset(uint64_t CurrentSimNanos)
{
    // check if input message has not been included
    if (!this->atmoDensInMsg.isLinked()) {
        bskLogger.bskLog(BSK_ERROR, "convectiveHeating.atmoDensInMsg was not linked.");
    }
}

/*! The convective heating does not write output messages to the rest of the sim.
@return void
 */
void ConvectiveHeating::WriteOutputMessages(uint64_t CurrentClock)
{
	return;
}

/*! This method is used to read the incoming density message and update the internal density/
atmospheric data.
 @return void
 */
bool ConvectiveHeating::ReadInputs()
{
	bool dataGood;
    this->atmoInData = this->atmoDensInMsg();
    dataGood = this->atmoDensInMsg.isWritten();
	return(dataGood);
}

/*!
    This method is used to link the ConvectiveHeating to the hub velocity,
    which are required for calculating heat flux.
    @return void
    @param states simulation states
 */
void ConvectiveHeating::linkInStates(DynParamManager& states){
	this->hubVelocity = states.getStateObject("hubVelocity");
}


/*! This method updates the heat flux based on the spacecraft velocity and atmospheric density.
*/
void ConvectiveHeating::computeHeatFlux(){

	this->v_B = this->hubVelocity->getState(); // [m/s] sc velocity
	this->v_B_mag = this->v_B.norm();
    this->heatFlux = this->coreParams.suttonGravesConstant*pow((this->atmoInData.neutralDensity/this->coreParams.noseRadius),0.5)*pow(this->v_B_mag,3);

	return;
}

/*! This method is called to update the local atmospheric conditions at each timestep.
Naturally, this means that conditions are held piecewise-constant over an integration step.
 @return void
 @param CurrentSimNanos The current simulation time in nanoseconds
 */
void ConvectiveHeating::UpdateState(uint64_t CurrentSimNanos)
{
    ReadInputs();
}
