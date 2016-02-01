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
 */

namespace mbed {
class I2C;
/**
 * The I2CBuffer class is a variant of the Buffer class.
 * Instead of just storing a buffer pointer and a size, if the buffer is less than
 * 8 bytes long, it packs the whole buffer into the space occupied by the pointer and
 * size variable. This is indicated by setting the MSB in size.
 */
class I2CBuffer {
public:
    /**
     * Set buffer pointer and length.
     *
     * @param[in] b A buffer to duplicate
     */
    void set(const Buffer & b);
    /**
     * Set the buffer pointer and length
     *
     * If the length of the buffer is less than 8 bytes it is stored internally
     *
     * @param[in] buf the buffer pointer to duplicate
     * @param[in] len the length of the buffer to duplicate
     */
    void set(void * buf, size_t len);
    /**
     * Get a pointer to the buffer.
     *
     * If the buffer is the internal storage, return it, otherwise return the reference
     *
     * @return the buffer pointer
     */
    void * get_buf();
    /**
     * Get the length
     *
     * @return the length of the buffer
     */
    size_t get_len() const;
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
class I2CTransaction;

namespace detail {
/**
 */
enum class I2CDirection {Transmit, Receive};

class I2CSegment : public I2CBuffer {
public:
    I2CSegment() :
        I2CBuffer(), _next(nullptr), _irqCB(nullptr)
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
} // namespace detail

/**
 * A Transaction container for I2C
 */
class I2CTransaction {
public:
    /** I2C transfer callback
     *  @param The transaction that was running when the callback was triggered
     *  @param int the event that triggered the calback
     */
    typedef mbed::util::FunctionPointer2<void, I2CTransaction*, int> event_callback_t;
    /**
     * Construct an I2C transaction and set the destination address at the same time
     *
     * @param[in] address set the I2C destination address
     */
    I2CTransaction(uint16_t address, uint32_t hz, bool irqsafe, I2C * issuer);
    ~I2CTransaction();

    detail::I2CSegment * new_segment();
public:
    I2CTransaction * next;         ///< NOTE: Should thie be volatile?
    uint16_t address;              ///< The target I2C address to communicate with
    detail::I2CSegment * root;     ///< The first I2CSegment in the transaction
    detail::I2CSegment * current;  ///< The current I2CSegment
    unsigned hz;                   ///< The I2C frequency to use for the transaction
    bool repeated;                 ///< If repeated is true, do not generate a stop condition
    /// Flag to indicate that the Transaction and its Segments were allocated with an irqsafe allocator
    bool irqsafe;
    I2C * issuer;                  ///< The I2C Object that launched this transaction
public:
    event_callback_t _doneCB;
    event_callback_t _nakCB;
    event_callback_t _noslaveCB;
    event_callback_t _errorCB;
};

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

    using event_callback_t = I2CTransaction::event_callback_t;
    /** Create an I2C Master interface, connected to the specified pins
     *
     *  @param sda I2C data line pin
     *  @param scl I2C clock line pin
     */
    I2C(PinName sda, PinName scl);
    I2C(PinName sda, PinName scl, mbed::util::PoolAllocator * TransactionPool, mbed::util::PoolAllocator * SegmentPool);

    /** Set the frequency of the I2C interface
     *
     *  @param hz The bus frequency in hertz
     */
    void frequency(int hz);

    class TransferAdder {
        friend I2C;
    protected:
        TransferAdder(I2C *i2c, int address, uint32_t hz, bool irqsafe);
    public:
        TransferAdder & frequency(uint32_t hz);
        TransferAdder & on(int event, const event_callback_t & cb);

        TransferAdder & repeated_start();
        int apply();

        TransferAdder & tx(void *buf, size_t len);
        TransferAdder & tx(const Buffer & buf);

        TransferAdder & rx(void *buf, size_t len);
        TransferAdder & rx(const Buffer & buf);
        TransferAdder & rx(size_t len);

        ~TransferAdder();
    protected:
        I2CTransaction * _xact;
        I2C* _i2c;
        bool _posted;
        bool _irqsafe;
        int _rc;
    };
    TransferAdder transfer_to(int address);
    TransferAdder transfer_to_irqsafe(int address);

    detail::I2CSegment * new_segment(bool irqsafe);

    void free(I2CTransaction * t);
    void free(detail::I2CSegment * s, bool irqsafe);

protected:
    friend TransferAdder;
    int post_transaction(I2CTransaction * t);
    I2CTransaction * new_transaction(uint16_t address, uint32_t hz, bool irqsafe, I2C * issuer);

    int _hz;
    size_t ownerID;

    mbed::util::PoolAllocator * TransactionPool;
    mbed::util::PoolAllocator * SegmentPool;

};

} // namespace mbed

#endif

#endif
