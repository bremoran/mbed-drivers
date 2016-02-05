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
#ifndef MBED_DRIVERS_V1_I2CDETAIL_HPP
#define MBED_DRIVERS_V1_I2CDETAIL_HPP

#include "mbed-drivers/platform.h"

#if DEVICE_I2C && DEVICE_I2C_ASYNCH

#include "EphemeralBuffer.hpp"
#include "core-util/FunctionPointer.h"
#include "PinNames.h"

namespace mbed_drivers {
using namespace mbed;
namespace v1 {
// Forward declaration of the I2CTransaction
class I2CTransaction;

namespace detail {
enum class I2CDirection {Transmit, Receive};

/** I2C transfer callback
 *  @param The transaction that was running when the callback was triggered
 *  @param The event that triggered the calback
 */
typedef mbed::util::FunctionPointer2<void, I2CTransaction *, uint32_t> event_callback_t;

/**
 * @brief A class that contains the information requires for an individial chunk of an I2C transaction
 *
 * I2CTransaction can be composed of several segments, each of which can be either transmit or receive segments. The
 * segments are formed into a linked list through the _next pointer. Each segment can have an associated callback that
 * executes in IRQ context. This allows for I2C transactions to be modified on the fly. For example, it might be useful
 * in some protocols to read a length, then transfer the number of bytes specified by the length.
 */
class I2CSegment : public EphemeralBuffer {
public:
    /**
     * I2C transfer callback
     * @param The segment that was running when the callback was triggered
     * @param The event that triggered the calback
     */
    using IRQCallback = mbed::util::FunctionPointer2<void, I2CSegment *, uint32_t>;
    I2CSegment() :
        EphemeralBuffer(), _next(nullptr), _irqCB(nullptr)
    {}

    /**
     * @brief Copies an existing I2CSegment
     *
     * Copying the underlying EphermeralBuffer has two different operations:
     *
     * * If the buffer is in ephemeral mode, it duplicates the data
     * * If it is not in ephemeral mode, it duplicates the pointer.
     *
     * Following segments are not duplicated.
     *
     * @param[in] s the I2CSegment to copy
     */
    I2CSegment(const I2CSegment & s) :
        EphemeralBuffer(static_cast<const EphemeralBuffer &>(s)), _dir(s._dir), _next(nullptr), _irqCB(s._irqCB)
    {}
    /**
     * @brief Move constructor
     *
     * Steals the references from the incoming rvalue reference
     * @param[in] rref The I2CSegment to move from
     */
    I2CSegment(I2CSegment && rref) :
        EphemeralBuffer(static_cast<EphemeralBuffer &&>(rref)), _dir(rref._dir), _next(rref._next), _irqCB(rref._irqCB)
     {}

    /**
     * @brief Set the following I2CSegment.
     *
     * In an I2CTransaction, this allows composition of a complex I2C transfer
     * @param[in] next an I2CSegment to append to the current one
     */
    void set_next(I2CSegment *next) {
        _next = next;
    }

    /**
     * @brief get a pointer to the next I2CSegment
     *
     * @return If another segment is appended, a pointer to that segment,
     * otherwise nullptr
     */
    I2CSegment * get_next() const {
        return _next;
    }

    /**
     * @brief Set the callback to execute immediately when this segment is
     * completed.
     *
     * This should typically be nullptr. No event filtering is provided on the
     * irq callback.
     * @param[in] cb the FunctionPointer to call when this segment completes
     */
    void set_irq_cb(IRQCallback cb) {
        _irqCB = cb;
    }

    /**
     * @brief Trigger the attached callback
     *
     * If there is an attached irq-context callback, call it with the incoming
     * event and the this pointer.
     *
     * @param[in] event the event which caused this callback.
     */
    void call_irq_cb(uint32_t event) {
        if (_irqCB) {
            _irqCB(this, event);
        }
    }

    /**
     * @brief Set the direction of the I2C transfer
     *
     * Set whether this segment is a transmit segment or a receive segment.
     *
     * @param[in] dir the direction of the transfer
     */
    void set_dir(I2CDirection dir) {
        _dir = dir;
    }

    /**
     * @brief Read the transfer direction
     *
     * @retval I2CDirection::Transmit this segment is a transmission segment
     * @retval I2CDirection::Receive this segment is a reception segment
     */
    I2CDirection get_dir() const {
        return _dir;
    }

protected:
    enum I2CDirection _dir;         ///< Direction of the transfer
    I2CSegment *      _next;        ///< Next segment to execute
    IRQCallback       _irqCB;       ///< Callback to execute in irq context
};
/**
 * @brief The base resource manager class for I2C
 *
 * The %I2CResourceManager manager is a multiplexer that guarantees mutually exclusive access to the underlying hardware
 * I2C master. It does this by serializing transactions and ensuring that they are processed atomically. This way, there
 * can be many users of the I2C bus, without access conflicts. In addition, the I2CResourceManager has simple power
 * management hooks.
 *
 * The %I2CResourceManager is the interface required to manage a particular I2C master. It provides several common
 * operations, but explicitly does not permit copy or move construction or assignment.
 *
 * The common operations provided by the Resource manager are:
 *
 * * Adding a transaction to the queue
 * * Processing an event
 *
 * These operations are common to all resource managers, so they are provided by the interface class.
 *
 * ## Event Handling overview
 *
 * When an interrupt occurs, the HAL processes it; if anything needs to be handled by the upper layers, an event is
 * generated. The derived resource manager should call I2CResourceManager::process_event with this event. If
 * appropriate, the %I2CResourceManager will call the %I2CSegment's irq callback handler.
 *
 * The resource manager handles events according to the following pseudocode:
 *
 * ```
 * If there is an error condition:
 *     Schedule handle_event() with the current transaction and event
 * Otherwise, if there are more segments to process:
 *     Start the next segment
 * Otherwise,
 *     Schedule handle_event() with the current transaction and the done flag
 * If another segment was not started,
 *     Start the next transaction
 * ```
 *
 * handle_event() calls the registered event handlers for the current transaction, then frees the Transaction, using the
 * I2C object that originally issued the transaction.
 *
 * ## Porting
 * To add a new type of I2C master (e.g. bitbanged), the a new resource manager must be implemented. It must implement
 * all of the pure virtual functions in %I2CResourceManager:
 *
 * * ```virtual I2CError init(PinName sda, PinName scl)```
 * * ```virtual I2CError start_transaction()```
 * * ```virtual I2CError start_segment()```
 * * ```virtual I2CError validate_transaction(I2CTransaction *transaction)```
 * * ```virtual I2CError power_down()```
 * * ```virtual I2CError power_up()```
 *
 * ENHANCEMENT: Resource Manager registration
 * It is not currently possible to derive a resource manager because it needs to be selectable through the
 * ```get_i2c_owner()``` API. A mechanism to register a resource manager is necessary before new I2C masters can be
 * ported. This is also tied in with the peripheral pin map. In order to discover the correct owner of an I2C master on
 * a given set of pins, the master must exist in the pinmap.
 */
class I2CResourceManager {
public:
    /* Copying and moving Resource Managers would have unpredictable results, so copy and move constructors are
     * explicitly deleted.
     */
    I2CResourceManager(const I2CResourceManager&) = delete;
    I2CResourceManager(I2CResourceManager&&) = delete;
    const I2CResourceManager& operator =(const I2CResourceManager&) = delete;
    const I2CResourceManager& operator =(I2CResourceManager&&) = delete;
    /**
     * @brief Initialize the I/O pins
     *
     * While the resource manager is initialized statically, it may need runtime initialization as well. init is called
     * each time a new I2C object is created.
     *
     * @param[in] sda the SDA pin of the I2C master to bind
     * @param[in] scl the SCL pin of the I2C master to bind
     */
    virtual I2CError init(PinName sda, PinName scl) = 0;
    /**
     * @brief Add a transaction to the transaction queue of the associated logical I2C port
     *
     * If the peripheral is idle, calls power_up()
     *
     * @param[in] transaction Queue this transaction
     * @return the result of validating the transaction
     */
    I2CError post_transaction(I2CTransaction *transaction);
protected:
    /* These APIs are the interfaces that must be supplied by a derived Resource Manager */
    /**
     * @brief Starts the transaction at the head of the queue
     * @return 0 if
     */
    virtual I2CError start_transaction() = 0;
    virtual I2CError start_segment() = 0;

    /** Validates the transaction according to the criteria of the derived Resource Manager */
    virtual I2CError validate_transaction(I2CTransaction *transaction) const = 0;
    /** Powers down the associated I2C controller */
    virtual I2CError power_down() = 0;
    /** Power up the associated I2C controller */
    virtual I2CError power_up() = 0;
protected:
    /** Handle an event
     * Starts the next transfer
     * If there are no more transfers queued, calls power_down()
     * Then, calls the event associated with the completed transfer.
     * Finally, frees the associated event.
     *
     * @param[in] event the source of the current handler call
     */
    void process_event(uint32_t event);
    void handle_event(I2CTransaction *t, uint32_t event);
    I2CResourceManager();
    ~I2CResourceManager();

    // The head of the transaction queue
    I2CTransaction * volatile TransactionQueue;
};

I2CResourceManager * get_i2c_owner(int I);

#define I2C_TRANSACTION_NHANDLERS 4

class I2CEventHandler {
public:
    I2CEventHandler();
    void call(I2CTransaction *t, uint32_t event);
    void set(const event_callback_t &cb, uint32_t event);
    operator bool() const;
protected:
    event_callback_t _cb;
    uint32_t _eventmask;
};

} // namespace detail
} // namespace v1
} // namespace mbed_drivers
#endif // DEVICE_I2C && DEVICE_I2C_ASYNCH

#endif // MBED_DRIVERS_V1_I2CDETAIL_HPP
