/*
Copyright (c) 2016, Autonomous Vehicle Systems Lab, Univeristy of Colorado at Boulder

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

#ifndef _NAV_AGGREGATE_H_
#define _NAV_AGGREGATE_H_

#include "messaging/static_messaging.h"
#include "attDetermination/_GeneralModuleFiles/navStateOut.h"

#define MAX_AGG_NAV_MSG 10

typedef struct {
    char inputNavName[MAX_STAT_MSG_LENGTH]; /*!< The name of the input message*/
    int32_t inputNavID; /*!< Sensor IDs tied to the input name*/
    NavStateOut msgStorage; /*! [-] Local buffer to store nav message*/
}AggregateMsgInput;

/*! @brief Top level structure for the CSS sensor interface system.  Contains all parameters for the
 CSS interface*/
typedef struct {
    AggregateMsgInput navMsgs[MAX_AGG_NAV_MSG]; /*!< [-] The incoming nav message buffer */
    char outputNavName[MAX_STAT_MSG_LENGTH]; /*!< The name of the input message*/
    uint32_t timeIdx;       /*!< [-] The index of the message to use for time */
    uint32_t attIdx;        /*!< [-] The index of the message to use for inertial MRP*/
    uint32_t rateIdx;       /*!< [-] The index of the message to use for attitude rate*/
    uint32_t posIdx;        /*!< [-] The index of the message to use for inertial position*/
    uint32_t velIdx;        /*!< [-] The index of the message to use for inertial velocity*/
    uint32_t dvIdx;         /*!< [-] The index of the message to use for accumulated DV */
    uint32_t sunIdx;        /*!< [-] The index of the message to use for sun pointing*/
    uint32_t msgCount;      /*!< [-] The total number of messages available as inputs */
    
    int32_t outputNavMsgID;    /*!< [-] The ID associated with the outgoing message*/
    
    NavStateOut outputData; /*!< [-] The local storage of the outgoing message data*/
}NavAggregateData;

#ifdef __cplusplus
extern "C" {
#endif
    
    void SelfInit_aggregateNav(NavAggregateData *ConfigData, uint64_t moduleID);
    void CrossInit_aggregateNav(NavAggregateData *ConfigData, uint64_t moduleID);
    void Update_aggregateNav(NavAggregateData *ConfigData, uint64_t callTime,
        uint64_t moduleID);
    
#ifdef __cplusplus
}
#endif

/*! @} */

#endif