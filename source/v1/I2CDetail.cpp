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

#include "mbed-drivers/platform.h"

#if DEVICE_I2C && DEVICE_I2C_ASYNCH

#include "mbed-drivers/v1/I2C.hpp"
#include "core-util/CriticalSectionLock.h"
#include "core-util/atomic_ops.h"
#include "core-util/assert.h"
#include "minar/minar.h"

namespace mbed_drivers {
using namespace mbed;
namespace v1 {
namespace detail {

I2CError I2CResourceManager::post_transaction(I2CTransaction *t) {
    CORE_UTIL_ASSERT(t != nullptr);
    if (!t) {
        return I2CError::NullTransaction;
    }
    I2CError rc = validate_transaction(t);
    if (rc != I2CError::None) {
        return rc;
    }

    // This can't be lock free because of the need to call append() on TransactionQueue.
    mbed::util::CriticalSectionLock lock;
    // A C-style cast is required to strip the volatile qualifier.
    I2CTransaction * tx = TransactionQueue;

    if (tx) {
        tx->append(t);
    } else {
        TransactionQueue = t;
        power_up();
        return start_transaction();
    }
    return I2CError::None;
}

void I2CResourceManager::process_event(uint32_t event) {
    I2CTransaction * t;
    CORE_UTIL_ASSERT(TransactionQueue != nullptr);
    if (!TransactionQueue) {
        return;
    }
    // Get the current item in the transaction queue
    // A C-style cast is required to strip the volatile qualifier.
    t = TransactionQueue;
    if (!t) {
        return;
    }
    // Fire the irqcallback for the segment
    t->call_irq_cb(event);
    // This can't be done with atomics: there are too many side-effects.
    mbed::util::CriticalSectionLock lock;
    // If there is another segment to process, advance the segment pointer
    // Record whether there was another segment
    bool TransactionDone = !t->advance_segment();
    // If there was an event that is not a complete event
    // or there was a complete event and the next segment is nullptr
    if ((event & I2C_EVENT_ALL & ~I2C_EVENT_TRANSFER_COMPLETE) ||
        ((event & I2C_EVENT_TRANSFER_COMPLETE) && TransactionDone))
    {
        // fire the handler
        minar::Scheduler::postCallback(event_callback_t(this, &I2CResourceManager::handle_event).bind(t,event));
        // Advance to the next transaction
        TransactionQueue = t->get_next();
        if (TransactionQueue) {
            // Initiate the next transaction
            start_transaction();
        } else {
            power_down();
        }
    } else if (!TransactionDone) {
        start_segment();
    }
}

void I2CResourceManager::handle_event(I2CTransaction *t, uint32_t event) {
    t->process_event(event);
    // This happens after the callbacks have all been called
    t->get_issuer()->free(t);
}

I2CResourceManager::I2CResourceManager() : TransactionQueue(nullptr) {}
I2CResourceManager::~I2CResourceManager() {
    mbed::util::CriticalSectionLock lock;
    while (TransactionQueue) {
        // A C-style cast is required to strip the volatile qualifier.
        I2CTransaction * tx = (TransactionQueue);
        TransactionQueue = tx->get_next();
        tx->get_issuer()->free(tx);
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
    }

    I2CError init(PinName sda, PinName scl) {
        // calling init during a transaction could cause communication artifacts
        if (!_inited) {
            // The init function also set the frequency to 100000
            i2c_init(&_i2c, sda, scl);
            _inited = true;
        } else {
            CORE_UTIL_ASSERT_MSG(_scl == scl && _sda == sda, "Each I2C peripheral may only be used on one set of pins");
            // Only support an I2C master on a single pair of pins
            if (_scl != scl || _sda != sda) {
                return I2CError::PinMismatch;
            }
        }
        return I2CError::None;
    }

    I2CError start_segment () {
        // A C-style cast is required to strip the volatile qualifier.
        I2CTransaction * t = TransactionQueue;
        CORE_UTIL_ASSERT(t != nullptr);
        if (!t) {
            return I2CError::NullTransaction;
        }
        I2CSegment * s = t->get_current();
        CORE_UTIL_ASSERT(s != nullptr);
        if (!s) {
            return I2CError::NullSegment;
        }
        bool stop = (s->get_next() == nullptr) && (!t->repeated());
        if (s->get_dir() == I2CDirection::Transmit) {
            i2c_transfer_asynch(&_i2c, s->get_buf(), s->get_len(), nullptr, 0, t->address(),
                stop, (uint32_t)_handler, I2C_EVENT_ALL, _usage);
        } else {
            i2c_transfer_asynch(&_i2c, nullptr, 0, s->get_buf(), s->get_len(), t->address(),
                stop, (uint32_t)_handler, I2C_EVENT_ALL, _usage);
        }
        return I2CError::None;
    }

    I2CError start_transaction() {
        if (i2c_active(&_i2c)) {
            return I2CError::Busy; // transaction ongoing
        }
        mbed::util::CriticalSectionLock lock;
        // A C-style cast is required to strip the volatile qualifier.
        I2CTransaction * t = TransactionQueue;
        CORE_UTIL_ASSERT(t != nullptr);
        if (!t) {
            return I2CError::NullTransaction;
        }
        i2c_frequency(&_i2c, t->freq());
        t->reset_current();
        return start_segment();
    }

    I2CError validate_transaction(I2CTransaction *t) const
    {
        (void) t;
        return I2CError::None;
    }

    I2CError power_down() {
        return I2CError::None;
    }

    I2CError power_up() {
        return I2CError::None;
    }

    void irq_handler() {
        uint32_t event = i2c_irq_handler_asynch(&_i2c);
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

template <size_t N> struct HWI2CResourceManagers : public HWI2CResourceManagers<N-1> {
    HWI2CResourceManagers() : rm(N, irq_handler_asynch){}
    HWI2CResourceManager rm;
    static void irq_handler_asynch(void)
    {
        HWI2CResourceManager *rm = static_cast<HWI2CResourceManager *>(get_i2c_owner(N));
        rm->irq_handler();
    }
    I2CResourceManager * get_rm(size_t I) {
        CORE_UTIL_ASSERT(I > N);
        if (I > N) {
            return nullptr;
        } else if (I == 0) {
            return &rm;
        } else {
            return HWI2CResourceManagers<N-1>::get_rm(I-1);
        }
    }
};

template <> struct HWI2CResourceManagers<0> {
    HWI2CResourceManagers() : rm(0, irq_handler_asynch){}
    HWI2CResourceManager rm;
    static void irq_handler_asynch(void)
    {
        HWI2CResourceManager *rm = static_cast<HWI2CResourceManager *>(get_i2c_owner(0));
        rm->irq_handler();
    }
    I2CResourceManager * get_rm(size_t I) {
        CORE_UTIL_ASSERT(I);
        if (I) {
            return nullptr;
        } else {
            return &rm;
        }
    }
};

I2CResourceManager * get_i2c_owner(int I)
{
    // Trap a failed pinmap_merge()
    CORE_UTIL_ASSERT_MSG(I >=0, "The scl, sda combination must exist in the peripheral pin map");
    if (I < 0) {
        return nullptr;
    }
    // Instantiate the HWI2CResourceManager
    static struct HWI2CResourceManagers<MODULES_SIZE_I2C-1> HWManagers;
    if (I < MODULES_SIZE_I2C) {
        return HWManagers.get_rm(I);
    } else {
        CORE_UTIL_ASSERT(false);
        return nullptr;
    }
    // NOTE: It should be possible to include other forms of I2C Resource manager.
    // Internal Ref: IOTSFW-1945
}

I2CEventHandler::I2CEventHandler():_cb(), _eventmask(0) {}

void I2CEventHandler::call(I2CTransaction *t, uint32_t event)
{
    _cb(t,event);
}

void I2CEventHandler::set(const event_callback_t &cb, uint32_t event)
{
    _cb = cb;
    _eventmask = event;
}

I2CEventHandler::operator bool() const
{
    return _eventmask && _cb;
}

} // namespace detail
} // namespace v1
} // namespace mbed_drivers
#endif // DEVICE_I2C && DEVICE_I2C_ASYNCH
