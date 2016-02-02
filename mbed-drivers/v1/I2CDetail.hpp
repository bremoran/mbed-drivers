/* mbed Microcontroller Library
 * Copyright (c) 2016 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_DRIVERS_V1_I2CDETAIL_HPP
#define MBED_DRIVERS_V1_I2CDETAIL_HPP

#include "mbed-drivers/platform.h"

#if DEVICE_I2C && DEVICE_I2C_ASYNCH

#include "EphemeralBuffer.hpp"
#include "core-util/FunctionPointer.h"
#include "PinNames.h"

namespace mbed_drivers {
using namespace mbed;
namespace v1 {
// Forward declaration of the I2CTransaction
class I2CTransaction;

namespace detail {
enum class I2CDirection {Transmit, Receive};

class I2CSegment : public EphemeralBuffer {
public:
    I2CSegment() :
        EphemeralBuffer(), _next(nullptr), _irqCB(nullptr)
    {}
    I2CSegment(I2CSegment & s) :
        _dir(s._dir), _next(nullptr), _irqCB(s._irqCB)
    {
        set(s.get_buf(), s.get_len());
    }
    void set_next(I2CSegment * next) {
        _next = next;
    }
    I2CSegment * get_next() const {
        return _next;
    }
    void set_irq_cb(mbed::util::FunctionPointer1<void, I2CSegment *> cb) {
        _irqCB = cb;
    }
    void call_irq_cb() {
        if (_irqCB) {
            _irqCB(_next);
        }
    }
    void set_dir(I2CDirection dir) {
        _dir = dir;
    }
    I2CDirection get_dir() const {
        return _dir;
    }
protected:
    enum I2CDirection _dir;          ///< The direction of the transfer
    I2CSegment * _next;
    mbed::util::FunctionPointer1<void, I2CSegment *> _irqCB;
};
/**
 * The base resource manager class for I2C
 */

class I2CResourceManager {
public:
    /* Copying and moving Resource Managers would have unpredictable results */
    I2CResourceManager(const I2CResourceManager&) = delete;
    I2CResourceManager(I2CResourceManager&&) = delete;
    const I2CResourceManager& operator =(const I2CResourceManager&) = delete;
    const I2CResourceManager& operator =(I2CResourceManager&&) = delete;
    /* Initialize the I/O pins
     * While the resource manager is initialized statically, it may need runtime initialization as well.
     * init is called each time a new I2C
     */
    virtual int init(PinName sda, PinName scl) = 0;
    /* Add a transaction to the transaction queue of the associated logical I2C port
     *
     * If the peripheral is idle, calls power_up()
     * @param[in] transaction Queue this transaction
     * @return the result of validating the transaction
     */
    // int post_transaction(const I2CTransaction &transaction);
    int post_transaction(I2CTransaction *transaction);
protected:
    /** These APIs are the interfaces that must be supplied by a derived Resource Manager */
    /** Starts the transaction at the head of the queue */
    virtual int start_transaction() = 0;
    virtual int start_segment() = 0;

    /** Validates the transaction according to the criteria of the derived Resource Manager */
    virtual int validate_transaction(I2CTransaction *transaction) const = 0;
    /** Powers down the associated I2C controller */
    virtual int power_down() = 0;
    /** Power up the associated I2C controller */
    virtual int power_up() = 0;
protected:
    /** Handle an event
     * Starts the next transfer
     * If there are no more transfers queued, calls power_down()
     * Then, calls the event associated with the completed transfer.
     * Finally, frees the associated event.
     *
     * @param[in] event the source of the current handler call
     */
    void process_event(int event);
    void event_cb(I2CTransaction * t, int event);
    I2CResourceManager();
    ~I2CResourceManager();
    // A shared pool of transaction objects for all I2C resource managers
    // static mbed::util::PoolAllocator TransactionPool;
    // The head of the transaction queue
    volatile I2CTransaction * TransactionQueue;
};

extern I2CResourceManager * I2COwners[];

} // namespace detail
} // namespace v1
} // namespace mbed_drivers
#endif // DEVICE_I2C && DEVICE_I2C_ASYNCH

#endif // MBED_DRIVERS_V1_I2CDETAIL_HPP
