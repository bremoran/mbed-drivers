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

#include "mbed-drivers/I2CDetail.hpp"

#if DEVICE_I2C && DEVICE_I2C_ASYNCH

#include "mbed-drivers/I2C.h"
#include "core-util/CriticalSectionLock.h"
#include "minar/minar.h"

namespace mbed {
namespace detail {
// TODO: Increase the size of I2COwners to accept Resource Mangers for non-onchip I2C masters
I2CResourceManager * I2COwners[MODULES_SIZE_I2C] = {nullptr};

int I2CResourceManager::post_transaction(I2CTransaction *t) {
    int rc;
    if ((rc = validate_transaction(t))) {
        return rc;
    }
    // Optimization NOTE: this could be lock-free with atomic_cas.
    mbed::util::CriticalSectionLock lock;
    // Find the tail of the queue
    I2CTransaction * tx = (I2CTransaction *)TransactionQueue;
    if (!tx) {
        TransactionQueue = t;
        power_up();
        start_transaction();
        return 0;
    }
    while (tx->next) {
        tx = tx->next;
    }
    tx->next = t;
    return 0;
}

void I2CResourceManager::process_event(int event) {
    I2CTransaction * t;
    CORE_UTIL_ASSERT(TransactionQueue != nullptr);
    if (!TransactionQueue) {
        return;
    }
    // Get the current item in the transaction queue
    t = (I2CTransaction *) TransactionQueue;
    if (!t) {
        return;
    }
    // Fire the irqcallback for the segment
    t->current->call_irq_cb();
    // If there was an event that is not a complete event
    // or there was a complete event and the next segment is nullptr
    if ((event & I2C_EVENT_ALL & ~I2C_EVENT_TRANSFER_COMPLETE) ||
        ((event & I2C_EVENT_TRANSFER_COMPLETE) && t->current->get_next() == nullptr))
    {
        // fire the handler
        using cbtype = mbed::util::FunctionPointer2<void,I2CTransaction *, int>;
        minar::Scheduler::postCallback(cbtype(this, &I2CResourceManager::event_cb).bind(t,event));
        // TODO: This could be lock-free
        // Enter a critical section
        mbed::util::CriticalSectionLock lock;
        // Advance to the next transaction
        TransactionQueue = t->next;
        if (TransactionQueue) {
            // Initiate the next transaction
            start_transaction();
        } else {
            power_down();
        }
    } else {
        // TODO: this could be lock-free
        mbed::util::CriticalSectionLock lock;
        t->current = t->current->get_next();
        start_segment();
    }
}
void I2CResourceManager::event_cb(I2CTransaction * t, int event) {
    // TODO: Should these be else-ifs?
    if (event & I2C_EVENT_ERROR_NO_SLAVE) {
        t->_noslaveCB(t, event);
    }
    if (event & I2C_EVENT_TRANSFER_EARLY_NACK) {
        t->_nakCB(t, event);
    }
    if (event & I2C_EVENT_ERROR) {
        t->_errorCB(t, event);
    }
    if (event & I2C_EVENT_TRANSFER_COMPLETE) {
        t->_doneCB(t, event);
    }
    t->issuer->free(t);
}


I2CResourceManager::I2CResourceManager() : TransactionQueue(nullptr) {}
I2CResourceManager::~I2CResourceManager() {
    while (TransactionQueue) {
        I2CTransaction * tx = (I2CTransaction *)(TransactionQueue);
        TransactionQueue = tx->next;
        tx->issuer->free(tx);
    }
}

class HWI2CResourceManager : public I2CResourceManager {
public:
    HWI2CResourceManager(const size_t id, void(*handler)(void)):
        _i2c(),
        _id(id),
        _usage(DMA_USAGE_NEVER),
        _inited(false),
        _handler(handler)
    {
        CORE_UTIL_ASSERT(I2COwners[id] == nullptr);
        I2COwners[id] = this;
    }
    int init(PinName sda, PinName scl) {
        // calling init during a transaction could cause communication artifacts
        if (!_inited) {
            // The init function also set the frequency to 100000
            i2c_init(&_i2c, sda, scl);
            _inited = true;
        } else {
            // Only support an I2C master on a single pair of pins
            if (_scl != scl || _sda != sda) {
                return -1;
            }
        }
        return 0;
    }
    int start_segment () {
        I2CTransaction * t = (I2CTransaction *)TransactionQueue;
        if (!t) {
            return -1;
        }
        I2CSegment * s = t->current;
        if (!s) {
            return -1;
        }
        bool stop = (s->get_next() == nullptr) && (!t->repeated);
        if (s->get_dir() == I2CDirection::Transmit) {
            i2c_transfer_asynch(&_i2c, s->get_buf(), s->get_len(), nullptr, 0, t->address,
                stop, (uint32_t)_handler, I2C_EVENT_ALL, _usage);
        } else {
            i2c_transfer_asynch(&_i2c, nullptr, 0, s->get_buf(), s->get_len(), t->address,
                stop, (uint32_t)_handler, I2C_EVENT_ALL, _usage);
        }
        return 0;
    }
    int start_transaction() {
        // TODO: This needs a higher-level start/stop lock
        if (i2c_active(&_i2c)) {
            return -1; // transaction ongoing
        }
        mbed::util::CriticalSectionLock lock;
        I2CTransaction * t = (I2CTransaction *)TransactionQueue;
        if (!t) {
            return -1;
        }

        // We want to update the frequency even if we are already the bus owners
        i2c_frequency(&_i2c, t->hz);
        t->current = t->root;
        return start_segment();
    }
    int validate_transaction(I2CTransaction *t) const
    {
        (void) t;
        // TODO
        return 0;
    }
    int power_down() {
        // TODO
        return 0;
    }
    int power_up() {
        // TODO
        return 0;
    }
    void irq_handler() {
        int event = i2c_irq_handler_asynch(&_i2c);
        process_event(event);
    }
protected:
    PinName _scl;
    PinName _sda;
    i2c_t _i2c;
    const size_t _id;
    DMAUsage _usage;
    bool _inited;
    void (*const _handler)(void);
};

// template <size_t N> struct HWI2CResourceManagers;
template <size_t N> struct HWI2CResourceManagers : HWI2CResourceManagers<N-1> {
    HWI2CResourceManagers() : rm(N, irq_handler_asynch){}
    HWI2CResourceManager rm;
    static void irq_handler_asynch(void)
    {
        HWI2CResourceManager *rm = static_cast<HWI2CResourceManager *>(I2COwners[N]);
        rm->irq_handler();
    }
};
template <> struct HWI2CResourceManagers<0> {
    HWI2CResourceManagers() : rm(0, irq_handler_asynch){}
    HWI2CResourceManager rm;
    static void irq_handler_asynch(void)
    {
        HWI2CResourceManager *rm = static_cast<HWI2CResourceManager *>(I2COwners[0]);
        rm->irq_handler();
    }
};

/* Instantiate the HWI2CResourceManager */
struct HWI2CResourceManagers<MODULES_SIZE_I2C-1> HWManagers;

}
}
#endif // DEVICE_I2C && DEVICE_I2C_ASYNCH
