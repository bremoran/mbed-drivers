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
#ifndef MBED_I2C_H
#define MBED_I2C_H

#include "platform.h"

#if DEVICE_I2C

#include "i2c_api.h"

#if DEVICE_I2C_ASYNCH
#include "CThunk.h"
#include "dma_api.h"
#include "core-util/FunctionPointer.h"
#include "core-util/PoolAllocator.h"
#include "Transaction.h"
#endif

namespace mbed {

namespace detail {
class I2CResourceManager;
class I2CTransaction {
public:
    I2CTransaction():
        next(nullptr), address(0), callback(nullptr), event(0), hz(100000), repeated(false)
    {}
    I2CTransaction(uint16_t address):
        next(nullptr), address(address), callback(nullptr), event(0), hz(100000), repeated(false)
    {}
    I2CTransaction(const I2CTransaction & t):
        next(nullptr), address(t.address),
        tx(t.tx), rx(t.rx),
        callback(t.callback), event(t.event), hz(t.hz), repeated(t.repeated)
    {}
    volatile I2CTransaction * next;
    int address;
    Buffer tx;
    Buffer rx;
    event_callback_t callback;
    int event;
    unsigned hz;
    bool repeated;
};

extern I2CResourceManager * I2COwners[MODULES_SIZE_I2C];

class I2CResourceManager {
public:
    I2CResourceManager(const I2CResourceManager&) = delete;
    I2CResourceManager(I2CResourceManager&&) = delete;
    operator =(const I2CResourceManager&) = delete;
    operator =(I2CResourceManager&&) = delete;
    int post_transaction(I2CTransaction &transaction);
protected:
    virtual int start_transaction() = 0;
    virtual int validate_transaction(I2CTransaction &transaction) = 0;
    void process_event(int event);
    I2CResourceManager();
    ~I2CResourceManager();
    static PoolAllocator TransactionPool;
    volatile I2CTransaction * TransactionQueue;
};
} // namespace detail

/** An I2C Master, used for communicating with I2C slave devices
 *
 * Example:
 * @code
 * // Read from I2C slave at address 0x62
 *
 * #include "mbed.h"
 *
 * I2C i2c(p28, p27);
 *
 * int main() {
 *     int address = 0x62;
 *     char data[2];
 *     i2c.read(address, data, 2);
 * }
 * @endcode
 */
class I2C {

public:
    enum RxStatus {
        NoData,
        MasterGeneralCall,
        MasterWrite,
        MasterRead
    };

    enum Acknowledge {
        NoACK = 0,
        ACK   = 1
    };

    /** Create an I2C Master interface, connected to the specified pins
     *
     *  @param sda I2C data line pin
     *  @param scl I2C clock line pin
     */
    I2C(PinName sda, PinName scl);

    /** Set the frequency of the I2C interface
     *
     *  @param hz The bus frequency in hertz
     */
    void frequency(int hz);

    /** I2C transfer callback
     *  @param Buffer the tx buffer
     *  @param Buffer the rx buffer
     *  @param int the event that triggered the calback
     */
    typedef mbed::util::FunctionPointer3<void, Buffer, Buffer, int> event_callback_t;

    class TransferAdder {
        friend I2C;
    private:
        TransferAdder(I2C *i2c, int address);
        TransferAdder(I2C *i2c);
        TransferAdder(const TransferAdder &adder);
        const TransferAdder & operator =(const TransferAdder &adder);
    public:
        TransferAdder & rx(uint8_t *buf, size_t len);
        TransferAdder & tx(uint8_t *buf, size_t len);
        TransferAdder & rx(Buffer buf);
        TransferAdder & tx(Buffer buf);
        TransferAdder & callback(const event_callback_t& callback, int event = I2C_EVENT_TRANSFER_COMPLETE);
        TransferAdder & repreated_start();
        int apply();
        ~TransferAdder();
    private:
        I2CTransaction xact;
        I2C* _i2c;
        bool _posted;
        int _rc;
    };

    TransferAdder transfer_to(int address);

protected:
    typedef TwoWayTransaction<event_callback_t> transaction_data_t;
    typedef Transaction<I2C, transaction_data_t> transaction_t;

    void irq_handler_asynch(void);
    transaction_data_t _current_transaction;
    CThunk<I2C> _irq;
    DMAUsage _usage;
#endif

protected:
    void aquire();

    i2c_t _i2c;
    static I2C  *_owner;
    int         _hz;
};

} // namespace mbed

#endif

#endif
