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

#define I2C_TRANSACTION_POOL_SIZE (I2C_TRANSACTION_POOL_ELEMENTS * sizeof(struct I2CTransaction));

PoolAllocator I2CResourceManager::TransactionPool(
    mbed_ualloc(I2C_TRANSACTION_POOL_SIZE,UALLOC_TRAITS_NEVER_FREE),
    I2C_TRANSACTION_POOL_ELEMENTS,
    sizeof(struct I2CTransaction)
);

int I2CResourceManager::post_transaction(const I2CTransaction &transaction) {
    // Allocate a new transaction.
    I2CTransaction * newtx = TransactionPool.allocate();
    if (newtx == nullptr) {
        return -1;
    }
    new(newtx) I2CTransaction(transaction);
    newtx->next = nullptr;
    mbed::util::CriticalSectionLock lock;
    // Find the tail of the queue
    I2CTransaction * tx = TransactionQueue;
    if (!tx) {
        TransactionQueue = newtx;
        power_up();
        start_transaction();
        return 0;
    }
    while (tx->next) {
        tx = tx->next;
    }
    tx->next = newtx;
    return 0;
}

void I2CResourceManager::process_event(int event) {
    I2CTransaction * t;
    CORE_UTIL_ASSERT(TransactionQueue != nullptr);
    if (!TransactionQueue) {
        return;
    }
    {
        CriticalSectionLock lock;
        t = TransactionQueue;
        TransactionQueue = t->next;
        if(TransactionQueue) {
            start_transaction();
        } else {
            power_down();
        }
    }
    if (t->event & event) {
        t->callback(t->tx, t->rx, event);
    }
    TransactionPool.free(t);
}

I2CResourceManager::I2CResourceManager() : TransactionQueue(nullptr) {}
I2CResourceManager::~I2CResourceManager() {
    while (TransactionQueue) {
        I2CTransaction * tx = TransactionQueue;
        TransactionQueue = tx->next;
        TransactionPool.free(tx);
    }
}

class HWI2CResourceManager : public I2CResourceManager {
public:
    HWI2CResourceManager(size_t id):
        id(id),
        _usage(DMA_USAGE_NEVER),
        _i2c()
    {
        CORE_UTIL_ASSERT(I2COwners[id] == nullptr);
        I2COwners[id] = this;

        // The init function also set the frequency to 100000
        i2c_init(&_i2c, sda, scl);
    }
    int start_transaction() {
        if (i2c_active(&_i2c)) {
            return -1; // transaction ongoing
        }
        CriticalSectionLock lock;
        I2CTransaction * t = TransactionQueue;

        // We want to update the frequency even if we are already the bus owners
        i2c_frequency(&_i2c, t.hz);

        int stop = (t.repeated) ? 0 : 1;
        i2c_transfer_asynch(&_i2c, t.tx.buf, t.tx.length, t.rx.buf, t.rx.length, t.address,
            stop, &irq_handler_asynch<id>, t.event, _usage);
        return 0;
    }
    int validate_transaction(I2CTransaction &transaction)
    {
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
    template <const size_t ID>
    static void irq_handler_asynch(void)
    {
        HWI2CResourceManager *rm = static_cast<HWI2CResourceManager *>(I2COwners[ID]);
        int event = i2c_irq_handler_asynch(&(rm->_i2c));
        rm->process_event(event);
    }
protected:
    i2c_t _i2c;
    const size_t id;
    DMAUsage _usage;
};

/* Instantiate the HWI2CResourceManager */
HWI2CResourceManager HWResourceOwners[MODULES_SIZE_I2C];

} // namespace detail
} // namespace mbed

namespace mbed {

I2C::I2C(PinName sda, PinName scl) :
    _hz(100000)
{}

void I2C::frequency(int hz)
{
    _hz = hz;
}

I2C::TransferAdder::TransferAdder(I2C *i2c, int address) :
    _address(address), _callback(NULL), _event(0), _repeated(false), _i2c(i2c), _posted(false), _rc(0), _hz
{}
I2C::TransferAdder::TransferAdder(I2C *i2c) :
_address(0), _callback(NULL), _event(0), _repeated(false), _i2c(i2c), _posted(false), _rc(0)
{}
I2C::TransferAdder::TransferAdder(const I2C::TransferAdder &adder) {
    *this = adder;
}
const I2C::TransferAdder & I2C::TransferAdder::operator =(const I2C::TransferAdder &adder) {
    _address = adder._address;
    _callback = adder._callback;
    _tx_buf = adder._tx_buf;
    _rx_buf = adder._rx_buf;
    _callback = adder._callback;
    _event = adder._event;
    _repeated = adder._repeated;
    _i2c = adder._i2c;
    _posted = false;
    _rc = 0;
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::rx(uint8_t *buf, size_t len) {
    _tx_buf.buf = buf;
    _tx_buf.length = len;
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::tx(uint8_t *buf, size_t len) {
    _rx_buf.buf = buf;
    _rx_buf.length = len;
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::callback(const event_callback_t& callback, int event)
{
    _callback = &callback;
    _event = event;
    return *this;
}
I2C::TransferAdder & I2C::TransferAdder::repreatedStart()
{
    _repeated = true;
    return *this;
}
int I2C::TransferAdder::apply()
{
    if (! _posted) {
        _rc = _i2c->transfer(_address, _tx_buf, _rx_buf, *_callback, _event, _repeated);
    }
    return _rc;
}
I2C::TransferAdder::~TransferAdder()
{
    apply();
}

#endif

} // namespace mbed

#endif
