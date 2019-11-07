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

#include "simpleStorageUnit.h"
#include "utilities/bsk_Print.h"

/*! The constructor creates a SimpleBattery instance with zero stored data */
SimpleStorageUnit::SimpleStorageUnit(){
    this->storageCapacity = -1.0;
    this->storedDataSum = 0.0;
    return;
}

SimpleStorageUnit::~SimpleStorageUnit(){
    return;
}

/*! Custom reset function/
 * @param CurrentClock
 */
void SimpleStorageUnit::customReset(__uint64_t CurrentClock){
    if (this->storageCapacity <= 0.0) {
        BSK_PRINT(MSG_ERROR, "The storageCapacity variable must be set to a positive value.");
    }
    return;
}