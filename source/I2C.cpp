/* mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
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
#include "mbed-drivers/I2C.h"
#include "minar/minar.h"
#include "ualloc/ualloc.h"
#include "core-util/CriticalSectionLock.h"
#include "PeripheralPins.h"

#if DEVICE_I2C && DEVICE_I2C_ASYNCH


namespace mbed {
namespace detail {
// TODO: Increase the size of I2COwners to accept Resource Mangers for non-onchip I2C masters
I2CResourceManager * I2COwners[MODULES_SIZE_I2C] = {nullptr};

// use a class instead of a structure so that we get the copy constructor for free.

#ifdef YOTTA_CFG_MBED_DRIVERS_I2C_POOL_ELEMENTS
#define I2C_TRANSACTION_POOL_ELEMENTS YOTTA_CFG_MBED_DRIVERS_I2C_POOL_ELEMENTS
#else
#define I2C_TRANSACTION_POOL_ELEMENTS (MODULES_SIZE_I2C * 4)
#endif

#define I2C_TRANSACTION_POOL_SIZE (I2C_TRANSACTION_POOL_ELEMENTS * sizeof(I2CTransaction))

mbed::util::PoolAllocator I2CResourceManager::TransactionPool(
    mbed_ualloc(I2C_TRANSACTION_POOL_SIZE,
        {.flags=UALLOC_TRAITS_NEVER_FREE}),
        I2C_TRANSACTION_POOL_ELEMENTS,
        sizeof(I2CTransaction)
);

int I2CResourceManager::post_transaction(const I2CTransaction &t) {
    int rc;
    if ((rc = validate_transaction(t))) {
        return rc;
    }
    // Allocate a new transaction.
    I2CTransaction * pNewT;
    {
        I2CTransaction NewT;
        pNewT = &NewT;
        const I2CTransaction * pOldT = &t;
        do {
            I2CTransaction * newtx = reinterpret_cast<I2CTransaction *>(TransactionPool.alloc());
            if (newtx == nullptr) {
                pNewT = NewT.next;
                while (pNewT) {
                    newtx = pNewT->next;
                    TransactionPool.free(pNewT);
                    pNewT = newtx;
                }
                return -1;
            }
            new(newtx) I2CTransaction(t);
            pNewT->next = newtx;
            pNewT = newtx;
            pOldT = pOldT->next;
        } while (pOldT);
    }
    // Optimization NOTE: this could be lock-free with atomic_cas.
    mbed::util::CriticalSectionLock lock;
    // Find the tail of the queue
    I2CTransaction * tx = (I2CTransaction *)TransactionQueue;
    if (!tx) {
        TransactionQueue = pNewT;
        power_up();
        start_transaction();
        return 0;
    }
    while (tx->next) {
        tx = tx->next;
    }
    tx->next = pNewT;
    return 0;
}

void I2CResourceManager::process_event(int event) {
    I2CTransaction * t;
    CORE_UTIL_ASSERT(TransactionQueue != nullptr);
    if (!TransactionQueue) {
        return;
    }
    {
        mbed::util::CriticalSectionLock lock;
        t = (I2CTransaction *) TransactionQueue;
        TransactionQueue = t->next;
        if(TransactionQueue) {
            start_transaction();
        } else {
            power_down();
        }
    }
    if (t->event & event) {
        t->callback(t->dir, t->b, event);
    }
    TransactionPool.free(t);
}

I2CResourceManager::I2CResourceManager() : TransactionQueue(nullptr) {}
I2CResourceManager::~I2CResourceManager() {
    while (TransactionQueue) {
        I2CTransaction * tx = (I2CTransaction *)(TransactionQueue);
        TransactionQueue = tx->next;
        TransactionPool.free(tx);
    }
}

class HWI2CResourceManager : public I2CResourceManager {
public:
    HWI2CResourceManager(size_t id, void(*handler)(void)):
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
    int start_transaction() {
        // TODO: This needs a higher-level start/stop lock
        if (i2c_active(&_i2c)) {
            return -1; // transaction ongoing
        }
        mbed::util::CriticalSectionLock lock;
        I2CTransaction * t = (I2CTransaction *)TransactionQueue;

        // We want to update the frequency even if we are already the bus owners
        i2c_frequency(&_i2c, t->hz);

        int stop = (t->repeated) ? 0 : 1;
        if (t->dir == I2CDirection::Transmit) {
            i2c_transfer_asynch(&_i2c, t->b.get_buf(), t->b.get_len(), nullptr, 0, t->address,
                stop, (uint32_t)_handler, t->event, _usage);
        } else {
            i2c_transfer_asynch(&_i2c, nullptr, 0, t->b.get_buf(), t->b.get_len(), t->address,
                stop, (uint32_t)_handler, t->event, _usage);
        }
        return 0;
    }
    int validate_transaction(const I2CTransaction &t) const
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
    void (*_handler)(void);
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
struct HWI2CResourceManagers<MODULES_SIZE_I2C> HWManagers;

} // namespace detail
} // namespace mbed

namespace mbed {

I2C::I2C(PinName sda, PinName scl) :
    _hz(100000)
{
    // Select the appropriate I2C Resource Manager
    uint32_t i2c_sda = pinmap_peripheral(sda, PinMap_I2C_SDA);
    uint32_t i2c_scl = pinmap_peripheral(scl, PinMap_I2C_SCL);
    ownerID = pinmap_merge(i2c_sda, i2c_scl);
    int rc = detail::I2COwners[ownerID]->init(sda, scl);
    if (rc) {
        ownerID = (size_t) -1;
    }
}

void I2C::frequency(int hz)
{
    _hz = hz;
}
I2C::TransferAdder I2C::transfer_to(int address) {
    TransferAdder t(this, address, _hz);
    return t;
}

int I2C::post_transaction(const detail::I2CTransaction & t) {
    if (ownerID == (size_t) -1) {
        return -1;
    }
    return detail::I2COwners[ownerID]->post_transaction(t);
}

I2C::TransferAdder::TransferAdder(I2C *i2c, int address, uint32_t hz) :
    _xact(address, hz), _i2c(i2c), _posted(false), _rc(0), _parent(nullptr)
{}
I2C::TransferAdder::TransferAdder(I2C *i2c) :
    _xact(), _i2c(i2c), _posted(false), _rc(0), _parent(nullptr)
{}
I2C::TransferAdder::TransferAdder(const I2C::TransferAdder &adder) {
    *this = adder;
}
I2C::TransferAdder::TransferAdder(TransferAdder * parent) :
    _xact(parent->_xact.address, parent->_xact.hz), _i2c(parent->_i2c), _posted(false), _rc(0), _parent(parent)
{
    *this = *parent;
    parent->_xact.next = &_xact;
    parent->_xact.repeated = true;
}
const I2C::TransferAdder & I2C::TransferAdder::operator =(const I2C::TransferAdder &adder) {
    _xact = adder._xact;
    _i2c = adder._i2c;
    _posted = false;
    _rc = 0;
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::callback(const event_callback_t& callback, int event)
{
    _xact.callback = callback;
    _xact.event = event;
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::repeated_start()
{
    _xact.repeated = true;
    return *this;
}
int I2C::TransferAdder::apply()
{
    if (_posted) {
        return _rc;
    }
    TransferAdder * ta = this;
    while (ta->_parent) { ta = ta->_parent; }
    _rc = _i2c->post_transaction(ta->_xact);
    return _rc;
}
I2C::TransferAdder::~TransferAdder()
{
    apply();
}
I2C::TransferAdder I2C::TransferAdder::then()
{
    TransferAdder adder(this);
    return adder;
}


I2C::TransferAdder & I2C::TransferAdder::tx(void *buf, size_t len) {
    _xact.b.set(buf,len);
    _xact.dir = detail::I2CDirection::Transmit;
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::tx(const Buffer & buf) {
    _xact.b.set(buf);
    _xact.dir = detail::I2CDirection::Transmit;
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::rx(void *buf, size_t len) {
    _xact.b.set(buf,len);
    _xact.dir = detail::I2CDirection::Receive;
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::rx(const Buffer & buf) {
    _xact.b.set(buf);
    _xact.dir = detail::I2CDirection::Receive;
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::rx(size_t len) {
    _xact.b.set(nullptr,len);
    _xact.dir = detail::I2CDirection::Receive;
    return *this;
}
} // namespace mbed

#endif
