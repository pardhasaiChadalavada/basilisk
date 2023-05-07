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

#ifndef CONVECTIVE_HEATING_H
#define CONVECTIVE_HEATING_H

#include <Eigen/Dense>
#include <vector>
#include "simulation/dynamics/_GeneralModuleFiles/dynamicEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/_GeneralModuleFiles/sys_model.h"

#include "architecture/msgPayloadDefC/AtmoPropsMsgPayload.h"
#include "architecture/messaging/messaging.h"

#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/avsEigenSupport.h"
#include "architecture/utilities/bskLogging.h"

//! @brief Container for basic drag parameters - the spacecraft's atmosphere-relative velocity, its projected area, and its drag coefficient.
typedef struct {
    double suttonGravesConstant;                    //!< sutton graves constant fro computing the convective heating
    double noseRadius;                              // Nose radius of the blunt body
}ConvectiveHeatingData;

/*! @brief Convective Heating C++ module class */
class ConvectiveHeating: public SysModel {
public:
    ConvectiveHeating();
    ~ConvectiveHeating();
    void Reset(uint64_t CurrentSimNanos);
    void UpdateState(uint64_t CurrentSimNanos);
    void WriteOutputMessages(uint64_t CurrentClock);
    bool ReadInputs();
    void computeHeatFlux();
    void linkInStates(DynParamManager& states);             //!< class method


public:
    ConvectiveHeatingData coreParams;                               //!< -- Struct used to hold convective heating parameters
    ReadFunctor<AtmoPropsMsgPayload> atmoDensInMsg;        //!< -- message used to read density inputs  
    Eigen::Vector3d v_B;                      //!< m/s local variable to hold the inertial velocity
    double v_B_mag;                           // Magnitude of the velocity
    StateData *hubVelocity;                                //!< m/s Hub inertial velocity vector   
    double heatFlux;                               // convective heat flux      
    BSKLogger bskLogger;              //!< -- BSK Logging

private:
    AtmoPropsMsgPayload atmoInData;

};


#endif
