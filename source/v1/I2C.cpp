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
#include "mbed-drivers/v1/I2C.hpp"
#include "minar/minar.h"
#include "ualloc/ualloc.h"
#include "core-util/CriticalSectionLock.h"
#include "PeripheralPins.h"

#if DEVICE_I2C && DEVICE_I2C_ASYNCH

namespace mbed_drivers {
using namespace mbed;
namespace v1 {
I2CTransaction::I2CTransaction(uint16_t address, uint32_t hz, bool irqsafe, I2C * issuer):
    next(nullptr),
    address(address),
    root(nullptr),
    current(nullptr),
    hz(hz),
    repeated(false),
    irqsafe(irqsafe),
    issuer(issuer)
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

I2C::I2C(PinName sda, PinName scl) :
    _hz(100000)
{
    // Select the appropriate I2C Resource Manager
    uint32_t i2c_sda = pinmap_peripheral(sda, PinMap_I2C_SDA);
    uint32_t i2c_scl = pinmap_peripheral(scl, PinMap_I2C_SCL);
    ownerID = pinmap_merge(i2c_sda, i2c_scl);
    int rc = detail::get_I2C_owner(ownerID)->init(sda, scl);
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
    return detail::get_I2C_owner(ownerID)->post_transaction(t);
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
    s->set_ephemeral(nullptr,len);
    s->set_dir(detail::I2CDirection::Receive);
    return *this;
}
} // namespace v1
} // namespace mbed

#endif
