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

/**
 * \file
 * \brief A generic interface for I2C peripherals
 *
 * The I2C class interfaces with an I2C Resource manager in order to initiate Transactions
 * and receive events. The I2CTransaction class encapsulates all I2C transaction parameters.
 * The I2CResourceManager class is a generic interface for implementing I2C resource managers.
 * This will allow for additional classes of I2C device, for example, a bitbanged I2C master.
 *
 * I2C Resource managers are instantiated statically and initialized during global init.
 * There is one Resource Manager per logical port. Logical ports could consist of:
 *
 * * Onchip I2C controllers
 * * I2C Bridges (SPI->I2C bridge, I2C->I2C bridge, etc.)
 * * Bit banged I2C
 * * Bit banged I2C over SPI GPIO expander
 * * More...
 *
 * Currently only onchip I2C controllers are supported.
 *
 * ## Constructing I2C transactions
 * To construct an I2C transaction with more than one segment, use the then member,
 * or the repeated_start member, with a CriticalSectionLock:
 *
 * ```C++
 * void doneCB(bool dir, I2CBuffer buf, int Event) {
 *     // Do something
 * }
 * I2C i2c0(sda, scl);
 * I2C i2c1(sda, scl);
 * void app_start (int, char **) {
 *     uint8_t cmd[2] = {0xaa, 0x55};
 *     i2c0.transfer_to(addr).tx(cmd,2).then().rx(4).callback(doneCB, I2C_EVENT_ALL);
 *     // OR
 *     CriticalSectionLock lock;
 *     i2c1.transfer_to(addr).tx(cmd,2).repeated_start();
 *     i2c1.transfer_to(addr).rx(4).callback(doneCB, I2C_EVENT_ALL);
 * }
 * ```
 *
 * TODO: Use helpers to make it impossible to call rx or tx more than once on the same Transaction
 */

namespace mbed {

class I2CBuffer {
public:
    void set(const Buffer & b) {
        set(b.buf,b.length);
    }
    void set(void * buf, size_t len) {
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
    void * get_buf() {
        if (_packed.small) {
            return _packed.data;
        } else {
            return _ref.data;
        }
    }
    size_t get_len() const {
        if (_packed.small) {
            return _packed.len;
        } else {
            return _ref.len;
        }
    }
protected:
    union {
        struct {
            void * data;
            size_t len;
        } _ref;
        struct {
            uint8_t data[sizeof(_ref)-1];
            unsigned small:1;
            unsigned len:7;
        } _packed;
    };
};

namespace detail {
/**
 */
enum class I2CDirection {Transmit, Receive};
/**
 * A Transaction container for I2C
 */
class I2CTransaction {
public:
    /** I2C transfer callback
     *  @param I2CDirection the direction of the transaction (true for tx or false for rx)
     *  @param Buffer the rx buffer
     *  @param int the event that triggered the calback
     */
    typedef mbed::util::FunctionPointer3<void, I2CDirection, I2CBuffer, int> event_callback_t;
    I2CTransaction():
        address(0), callback(nullptr), event(0), hz(100000), repeated(false)
    {}
    /**
     * Construct an I2C transaction and set the destination address at the same time
     *
     * @param[in] address set the I2C destination address
     */
    I2CTransaction(uint16_t address, uint32_t hz):
        address(address), callback(nullptr), event(0), hz(hz), repeated(false)
    {}
    /**
     * Copy an I2C transaction
     * Does not copy the next pointer or the callback pointer
     */
    I2CTransaction(const I2CTransaction & t)
    {
        *this = t;
    }
    const I2CTransaction & operator = (const I2CTransaction & rhs)
    {
        next = nullptr;
        address = rhs.address;
        b = rhs.b;
        dir = rhs.dir;
        callback = rhs.callback;
        event = rhs.event;
        hz = rhs.hz;
        repeated = rhs.repeated;
        return *this;
    }
public:
    I2CTransaction * next;     ///< NOTE: Should thie be volatile?
    uint16_t address;          ///< The target I2C address to communicate with
    I2CBuffer b;               ///< The buffer (```void *```/size pair) to receive into or transmit from
    I2CDirection dir;          ///< The direction of the transfer
    event_callback_t callback; ///< The function pointer to call when the transaction terminates
    int event;                 ///< The event mask for the callback
    unsigned hz;               ///< The I2C frequency to use for the transaction
    bool repeated;             ///< If repeated is true, do not generate a stop condition
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
    int post_transaction(const I2CTransaction &transaction);
protected:
    /** These APIs are the interfaces that must be supplied by a derived Resource Manager */
    /** Starts the transaction at the head of the queue */
    virtual int start_transaction() = 0;
    /** Validates the transaction according to the criteria of the derived Resource Manager */
    virtual int validate_transaction(const I2CTransaction &transaction) const = 0;
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
    I2CResourceManager();
    ~I2CResourceManager();
    // A shared pool of transaction objects for all I2C resource managers
    static mbed::util::PoolAllocator TransactionPool;
    // The head of the transaction queue
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
 * void tx_done
 * void app_start(int, char **) {
 *     static char txdata[2] = {0xaa, 0x55};
 *     i2c.transfer_to(address).tx(data, 2).callback();
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

    using event_callback_t = detail::I2CTransaction::event_callback_t;
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

    class TransferAdder {
        friend I2C;
    protected:
        const TransferAdder & operator =(const TransferAdder &adder);
        TransferAdder(I2C *i2c, int address, uint32_t hz);
        TransferAdder(TransferAdder * parent);
        TransferAdder(I2C *i2c);
        TransferAdder(const TransferAdder &adder);
    public:
        TransferAdder & frequency(uint32_t hz);
        TransferAdder & callback(const event_callback_t & callback,
            int event = I2C_EVENT_TRANSFER_COMPLETE);
        TransferAdder & repeated_start();
        int apply();
        TransferAdder then();

        TransferAdder & tx(void *buf, size_t len);
        TransferAdder & tx(const Buffer & buf);

        TransferAdder & rx(void *buf, size_t len);
        TransferAdder & rx(const Buffer & buf);
        TransferAdder & rx(size_t len);

        ~TransferAdder();
    protected:
        detail::I2CTransaction _xact;
        I2C* _i2c;
        bool _posted;
        int _rc;
        TransferAdder * _parent;
    };
    TransferAdder transfer_to(int address);
    int post_transaction(const detail::I2CTransaction & t);

protected:
    int _hz;
    size_t ownerID;
};

} // namespace mbed

#endif

#endif
