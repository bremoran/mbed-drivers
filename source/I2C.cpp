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
I2CTransaction::I2CTransaction(uint16_t address, uint32_t hz, bool irqsafe, I2C * issuer):
    address(address), hz(hz), repeated(false), irqsafe(irqsafe), issuer(issuer)
{}
I2CTransaction::~I2CTransaction() {
    current = root;
    while (current) {
        detail::I2CSegment * next = current->get_next();
        issuer->free(current, irqsafe);
        current = next;
    }
}

detail::I2CSegment * I2CTransaction::new_segment()
{
    detail::I2CSegment * s = issuer->new_segment(irqsafe);
    CORE_UTIL_ASSERT(s != nullptr);
    if (!s) {
        return nullptr;
    }
    s->set_next(nullptr);
    mbed::util::CriticalSectionLock lock;
    if (root == NULL) {
        root = s;
        current = s;
    } else {
        current->set_next(s);
        current = s;
    }
    return s;
}

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
struct HWI2CResourceManagers<MODULES_SIZE_I2C> HWManagers;

} // namespace detail

void I2CBuffer::set(const Buffer & b) {
    set(b.buf,b.length);
}
void I2CBuffer::set(void * buf, size_t len) {
    if (len <= sizeof(_packed.data)) {
        _packed.small = true;
        _packed.len = len;
        if (buf) {
            memcpy(_packed.data, buf, len);
        }
    } else {
        _ref.len = len;
        _ref.data = buf;
    }
}
void * I2CBuffer::get_buf() {
    if (_packed.small) {
        return _packed.data;
    } else {
        return _ref.data;
    }
}
size_t I2CBuffer::get_len() const {
    if (_packed.small) {
        return _packed.len;
    } else {
        return _ref.len;
    }
}
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
    TransferAdder t(this, address, _hz, false);
    return t;
}

int I2C::post_transaction(I2CTransaction * t) {
    if (ownerID == (size_t) -1) {
        return -1;
    }
    return detail::I2COwners[ownerID]->post_transaction(t);
}

detail::I2CSegment * I2C::new_segment(bool irqsafe) {
    detail::I2CSegment * newseg = nullptr;
    if (irqsafe) {
        if (!SegmentPool) {
            return nullptr;
        }
        void * space = SegmentPool->alloc();
        if (!space) {
            return nullptr;
        }
        newseg = new(space) detail::I2CSegment();
    } else {
        newseg = new detail::I2CSegment();
    }
    return newseg;
}

I2CTransaction * I2C::new_transaction(uint16_t address, uint32_t hz, bool irqsafe, I2C * issuer)
{
    I2CTransaction * t;
    if (irqsafe) {
        if (!TransactionPool) {
            return nullptr;
        }
        void *space = TransactionPool->alloc();
        if (!space) {
            return nullptr;
        }
        t = new(space) I2CTransaction(address, hz, irqsafe, issuer);
    } else {
        t = new I2CTransaction(address, hz, irqsafe, issuer);
    }
    return t;
}
void I2C::free(detail::I2CSegment * s, bool irqsafe)
{
    if (irqsafe) {
        s->~I2CSegment();
        SegmentPool->free(s);
    } else {
        delete s;
    }
}
void I2C::free(I2CTransaction * t)
{
    if (t->irqsafe) {
        t->~I2CTransaction();
        TransactionPool->free(t);
    } else {
        delete t;
    }
}


I2C::TransferAdder::TransferAdder(I2C *i2c, int address, uint32_t hz, bool irqsafe) :
    _i2c(i2c), _posted(false), _irqsafe(irqsafe), _rc(0)
{
    _xact = i2c->new_transaction(address, hz, irqsafe, i2c);
    CORE_UTIL_ASSERT(_xact != nullptr);
    if (!_xact) {
        return;
    }
    _xact->address = address;
    _xact->hz = hz;
    _xact->irqsafe = irqsafe;
}
I2C::TransferAdder & I2C::TransferAdder::repeated_start()
{
    _xact->repeated = true;
    return *this;
}
int I2C::TransferAdder::apply()
{
    if (_posted) {
        return _rc;
    }
    _rc = _i2c->post_transaction(_xact);
    return _rc;
}
I2C::TransferAdder & I2C::TransferAdder::on(int event, const event_callback_t & cb)
{
    if (event == I2C_EVENT_ERROR) {
        _xact->_errorCB = cb;
    }
    if (event == I2C_EVENT_ERROR_NO_SLAVE) {
        _xact->_noslaveCB = cb;
    }
    if (event == I2C_EVENT_TRANSFER_COMPLETE) {
        _xact->_doneCB = cb;
    }
    if (event == I2C_EVENT_TRANSFER_EARLY_NACK) {
        _xact->_nakCB = cb;
    }
    return *this;
}


I2C::TransferAdder::~TransferAdder()
{
    apply();
}

I2C::TransferAdder & I2C::TransferAdder::tx(void *buf, size_t len) {
    detail::I2CSegment * s = _xact->new_segment();
    s->set(buf,len);
    s->set_dir(detail::I2CDirection::Transmit);
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::tx(const Buffer & buf) {
    detail::I2CSegment * s = _xact->new_segment();
    s->set(buf);
    s->set_dir(detail::I2CDirection::Transmit);
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::rx(void *buf, size_t len) {
    detail::I2CSegment * s = _xact->new_segment();
    s->set(buf,len);
    s->set_dir(detail::I2CDirection::Receive);
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::rx(const Buffer & buf) {
    detail::I2CSegment * s = _xact->new_segment();
    s->set(buf);
    s->set_dir(detail::I2CDirection::Receive);
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::rx(size_t len) {
    detail::I2CSegment * s = _xact->new_segment();
    s->set(nullptr,len);
    s->set_dir(detail::I2CDirection::Receive);
    return *this;
}
} // namespace mbed

#endif
