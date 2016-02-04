/* mbed Microcontroller Library
 * Copyright (c) 2006-2016 ARM Limited
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
#include "mbed-drivers/v1/I2C.hpp"
#include "minar/minar.h"
#include "ualloc/ualloc.h"
#include "core-util/CriticalSectionLock.h"
#include "PeripheralPins.h"

#if DEVICE_I2C && DEVICE_I2C_ASYNCH

namespace mbed_drivers {
using namespace mbed;
namespace v1 {
I2CTransaction::I2CTransaction(uint16_t address, uint32_t hz, bool irqsafe, I2C *issuer):
    _next(nullptr),
    _address(address),
    _root(nullptr),
    _current(nullptr),
    _hz(hz),
    _repeated(false),
    _irqsafe(irqsafe),
    _issuer(issuer)
{}
I2CTransaction::~I2CTransaction() {
    mbed::util::CriticalSectionLock lock;
    _current = _root;
    while (_current) {
        detail::I2CSegment * next = _current->get_next();
        _issuer->free(_current, _irqsafe);
        _current = next;
    }
}

void I2CTransaction::append(I2CTransaction *t) {
    CORE_UTIL_ASSERT(t != nullptr);
    if (!t) {
        return;
    }
    // Append is called from within a critical section, so an atomic_cas is not necessary
    if (_next == nullptr) {
        _next = t;
    } else {
        _next->append(t);
    }
}

void I2CTransaction::call_irq_cb(uint32_t event) {
    if (_current) {
        _current->call_irq_cb(event);
    }
}

bool I2CTransaction::advance_segment() {
    if (!_current) {
        return false;
    }
    _current = _current->get_next();
    return _current != nullptr;
}

detail::I2CSegment * I2CTransaction::new_segment()
{
    detail::I2CSegment * s = _issuer->new_segment(_irqsafe);
    CORE_UTIL_ASSERT(s != nullptr);
    if (!s) {
        return nullptr;
    }
    s->set_next(nullptr);
    mbed::util::CriticalSectionLock lock;
    if (_root == NULL) {
        _root = s;
        _current = s;
    } else {
        _current->set_next(s);
        _current = s;
    }
    return s;
}
bool I2CTransaction::add_event(uint32_t event, const event_callback_t & cb) {
    size_t i;
    for (i = 0; i < I2C_TRANSACTION_NHANDLERS; i++) {
        if (!_handlers[i]) {
            _handlers[i].set(cb, event);
            break;
        }
    }
    return i < I2C_TRANSACTION_NHANDLERS;
}

void I2CTransaction::process_event(uint32_t event) {
    size_t i;
    for (i = 0; i < I2C_TRANSACTION_NHANDLERS; i++) {
        if (_handlers[i]) {
            _handlers[i].call(this, event);
        }
    }
}

I2C::I2C(PinName sda, PinName scl) :
    _hz(100000)
{
    // Select the appropriate I2C Resource Manager
    uint32_t i2c_sda = pinmap_peripheral(sda, PinMap_I2C_SDA);
    uint32_t i2c_scl = pinmap_peripheral(scl, PinMap_I2C_SCL);
    int ownerID = pinmap_merge(i2c_sda, i2c_scl);
    _owner = detail::get_i2c_owner(ownerID);
    CORE_UTIL_ASSERT(I2CError::None == _owner->init(sda, scl));
}

void I2C::frequency(int hz)
{
    _hz = hz;
}
I2C::TransferAdder I2C::transfer_to(int address) {
    TransferAdder t(this, address, _hz, false);
    return t;
}

I2CError I2C::post_transaction(I2CTransaction *t) {
    if (!_owner) {
        return I2CError::InvalidMaster;
    }
    return _owner->post_transaction(t);
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

I2CTransaction * I2C::new_transaction(uint16_t address, uint32_t hz, bool irqsafe, I2C *issuer)
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
void I2C::free(detail::I2CSegment *s, bool irqsafe)
{
    if (irqsafe) {
        s->~I2CSegment();
        SegmentPool->free(s);
    } else {
        delete s;
    }
}
void I2C::free(I2CTransaction *t)
{
    if (t->is_irqsafe()) {
        t->~I2CTransaction();
        TransactionPool->free(t);
    } else {
        delete t;
    }
}


I2C::TransferAdder::TransferAdder(I2C *i2c, int address, uint32_t hz, bool irqsafe) :
    _i2c(i2c), _posted(false), _irqsafe(irqsafe), _rc(I2CError::None)
{
    _xact = i2c->new_transaction(address, hz, irqsafe, i2c);
    CORE_UTIL_ASSERT(_xact != nullptr);
    if (!_xact) {
        return;
    }
}
I2C::TransferAdder & I2C::TransferAdder::repeated_start()
{
    _xact->repeated(true);
    return *this;
}
I2CError I2C::TransferAdder::apply()
{
    if (_posted) {
        return _rc;
    }
    return _i2c->post_transaction(_xact);
}
I2C::TransferAdder & I2C::TransferAdder::on(uint32_t event, const event_callback_t & cb)
{
    _xact->add_event(event, cb);
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::frequency(uint32_t hz)
{
    _xact->freq(hz);
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
    s->set_ephemeral(nullptr,len);
    s->set_dir(detail::I2CDirection::Receive);
    return *this;
}
} // namespace v1
} // namespace mbed

#endif
