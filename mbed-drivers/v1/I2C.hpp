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
#ifndef MBED_DRIVERS_V1_I2C_HPP
#define MBED_DRIVERS_V1_I2C_HPP

#include "mbed-drivers/platform.h"

#if DEVICE_I2C && DEVICE_I2C_ASYNCH


#include "mbed-hal/i2c_api.h"
#include "mbed-hal/dma_api.h"

#include "mbed-drivers/CThunk.h"
#include "core-util/FunctionPointer.h"
#include "core-util/PoolAllocator.h"

// Forward declarations
namespace mbed_drivers {
namespace v1 {
enum class I2CError;
}
}

#include "I2CDetail.hpp"
#include "EphemeralBuffer.hpp"

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
 *
 * ```C++
 * void doneCB(bool dir, EphemeralBuffer buf, uint32_t event) {
 *     // Do something
 * }
 * I2C i2c0(sda, scl);
 * I2C i2c1(sda, scl);
 * void app_start (int, char **) {
 *     uint8_t cmd[2] = {0xaa, 0x55};
 *     i2c0.transfer_to(addr).tx(cmd,2).rx(4).on(I2C_EVENT_ALL, doneCB);
 * }
 * ```
 */
namespace mbed_drivers {
using namespace mbed;
namespace v1 {
// Forward declaration of I2C
class I2C;
enum class I2CError {None, InvalidMaster, PinMismatch, Busy, NullTransaction, NullSegment};
/**
 * A Transaction container for I2C
 */
class I2CTransaction {
public:
    /** I2C transfer callback
     *  @param The transaction that was running when the callback was triggered
     *  @param the event that triggered the calback
     */
    using event_callback_t = detail::event_callback_t;
    /**
     * Construct an I2C transaction and set the destination address at the same time
     *
     * @param[in] address set the I2C destination address
     */
    I2CTransaction(uint16_t address, uint32_t hz, bool irqsafe, I2C *issuer);
    ~I2CTransaction();

    detail::I2CSegment * new_segment();
    bool add_event(uint32_t event, const event_callback_t & cb);
    /**
     * The resource manager calls this API.
     * Calls the event handlers and
     */
    void process_event(uint32_t event);

    /**
     * Set the next transaction in the queue
     * This is an atomic operation that will append to the queue.
     */
    void append(I2CTransaction *t);

    /**
     * Forwards the irq-context callback to the segment
     * Also adds a pointer to this transaction to the callback
     * @param[in] the event that triggered this callback
     */
    void call_irq_cb(uint32_t event);

    /**
     * If the current segment is valid advance the segment pointer
     *
     * @retval true if the current segment is valid after this operation
     * @retval false if the current segment is not valid after this operation
     */
    bool advance_segment();

    /**
     * Reset the current segment to the root segment
     */
    void reset_current() {
        _current  = _root;
    }

    /**
     * Accessor for the next pointer
     * @return the next transaction
     */
    I2CTransaction * get_next() {
        return _next;
    }

    /**
     * Accessor for the Transactions's issuer
     * @return the I2C object that issued this transaction
     */
    I2C * get_issuer() {
        return _issuer;
    }

    detail::I2CSegment * get_current() {
        return _current;
    }

    bool is_irqsafe() const {
        return _irqsafe;
    }

    void repeated(bool r) {
        _repeated = r;
    }

    bool repeated() const {
        return _repeated;
    }

    uint32_t freq() const {
        return _hz;
    }

    void freq(uint32_t hz) {
        _hz = hz;
    }

    uint16_t address() const {
        return _address;
    }
protected:
    /** The next transaction in the queue
     * This field is used for chaining transactions together into a queue.
     *
     * This field is not volatile because it is only accessed from within a
     * critical section. This will still be valid if the critical section is
     * replaced with an atomic operation.
     */
    I2CTransaction * _next;         ///< The next transaction in the queue
    uint16_t _address;              ///< The target I2C address to communicate with
    /**
     * The first I2CSegment in the transaction
     *
     * This field is not volatile because it is only accessed from within a
     * critical section. This will still be valid if the critical section is
     * replaced with an atomic operation.
     */
    detail::I2CSegment * _root;
    /**
     * The first I2CSegment in the transaction
     *
     * This is a helper field for building or processing I2C transactions.
     * It allows the I2CTransaction to easily locate the end of the queue when
     * composing the transaction and, equally, the currently transferring segment
     * while processing the transaction.
     *
     * This field is not volatile because it is only accessed from within a
     * critical section. This will still be valid if the critical section is
     * replaced with an atomic operation.
     */
    detail::I2CSegment * _current;  ///< The current I2CSegment
    unsigned _hz;                   ///< The I2C frequency to use for the transaction
    bool _repeated;                 ///< If repeated is true, do not generate a stop condition
    /// Flag to indicate that the Transaction and its Segments were allocated with an irqsafe allocator
    bool _irqsafe;
    I2C * _issuer;                  ///< The I2C Object that launched this transaction
    /// An array of I2C Event Handlers
    detail::I2CEventHandler _handlers[I2C_TRANSACTION_NHANDLERS];
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
    using event_callback_t = detail::event_callback_t;
    /** Create an I2C Master interface, connected to the specified pins
     *
     *  @param sda I2C data line pin
     *  @param scl I2C clock line pin
     */
    I2C(PinName sda, PinName scl);
    I2C(PinName sda, PinName scl, mbed::util::PoolAllocator *TransactionPool, mbed::util::PoolAllocator *SegmentPool);

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
        TransferAdder & on(uint32_t event, const event_callback_t & cb);

        TransferAdder & repeated_start();
        I2CError apply();

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
        I2CError _rc;
    };
    TransferAdder transfer_to(int address);
    TransferAdder transfer_to_irqsafe(int address);

    detail::I2CSegment * new_segment(bool irqsafe);

    void free(I2CTransaction *t);
    void free(detail::I2CSegment *s, bool irqsafe);

protected:
    friend TransferAdder;
    I2CError post_transaction(I2CTransaction *t);
    I2CTransaction * new_transaction(uint16_t address, uint32_t hz, bool irqsafe, I2C *issuer);

    int _hz;
    detail::I2CResourceManager * _owner;

    mbed::util::PoolAllocator * TransactionPool;
    mbed::util::PoolAllocator * SegmentPool;

};
} // namespace v1
} // namespace mbed_drivers

#endif

#endif // MBED_DRIVERS_V1_I2C_HPP
