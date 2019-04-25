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

#ifndef _INERTIAL3D_
#define _INERTIAL3D_

#include "messaging/static_messaging.h"
#include <stdint.h>
#include "fswMessages/attRefFswMsg.h"

/*! \defgroup inertial3D
 * @brief This attitude guidance module create a reference attitude message that points in fixed inertial direction. The module [PDF Description](Basilisk-Inertial3D-2016-01-15.pdf) contains further information on this module's function,
     how to run it, as well as testing.
 * @{
 */


/*!@brief Data structure for module to compute the Inertial-3D pointing navigation solution.
 */
typedef struct {
    double sigma_R0N[3];                            //!<        MRP from inertial frame N to corrected reference frame R
    char outputDataName[MAX_STAT_MSG_LENGTH];       //!<        The name of the output message
    int32_t outputMsgID;                            //!< (-)    ID for the outgoing message
}inertial3DConfig;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_inertial3D(inertial3DConfig *configData, uint64_t moduleID);
    void CrossInit_inertial3D(inertial3DConfig *configData, uint64_t moduleID);
    void Update_inertial3D(inertial3DConfig *configData, uint64_t callTime, uint64_t moduleID);
    void Reset_inertial3D(inertial3DConfig *configData, uint64_t callTime, uint64_t moduleID);

    void computeInertialPointingReference(inertial3DConfig *ConfigData, AttRefFswMsg *attRefOut);

#ifdef __cplusplus
}
#endif

/*! @} */

#endif