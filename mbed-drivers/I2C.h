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
 */

namespace mbed {
namespace detail {
/**
 * A Transaction container for I2C
 */
class I2CTransaction {
public:
    /** I2C transfer callback
     *  @param Buffer the tx buffer
     *  @param Buffer the rx buffer
     *  @param int the event that triggered the calback
     */
    typedef mbed::util::FunctionPointer3<void, Buffer, Buffer, int> event_callback_t;
    I2CTransaction():
        next(nullptr), address(0), callback(nullptr), event(0), hz(100000), repeated(false)
    {}
    /**
     * Construct an I2C transaction and set the destination address at the same time
     *
     * @param[in] address set the I2C destination address
     */
    I2CTransaction(uint16_t address):
        next(nullptr), address(address), callback(nullptr), event(0), hz(100000), repeated(false)
    {}
    /**
     * Copy an I2C transaction
     * Does not copy the next pointer or the callback pointer
     */
    I2CTransaction(const I2CTransaction & t):
        next(nullptr), address(t.address),
        tx(t.tx), rx(t.rx),
        callback(t.callback), event(t.event), hz(t.hz), repeated(t.repeated)
    {}
    /* The next pointer is used to chain Transactions together into a queue.
     * TODO: Should this be added as a wrapper class in the cpp file, since it's not relevant to the user-facing API?
     */
    volatile I2CTransaction * next;
    uint16_t address;          ///< The target I2C address to communicate with
    Buffer tx;                 ///< The buffer (```void *```/size pair) to send from
    Buffer rx;                 ///< The buffer (```void *```/size pair) to receive into
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
    operator =(const I2CResourceManager&) = delete;
    operator =(I2CResourceManager&&) = delete;
    /* Initialize the I/O pins
     * While the resource manager is initialized statically, it may need runtime initialization as well.
     */
    virtual int init(PinName sda, PinName scl) = 0;

    /* Add a transaction to the transaction queue of the associated logical I2C port
     *
     * If the peripheral is idle, calls power_up()
     * @param[in] transaction Queue this transaction
     * @return the result of validating the transaction
     */
    int post_transaction(I2CTransaction &transaction);
protected:
    /** These APIs are the interfaces that must be supplied by a derived Resource Manager */
    /** Starts the transaction at the head of the queue */
    virtual int start_transaction() = 0;
    /** Validates the transaction according to the criteria of the derived Resource Manager */
    virtual int validate_transaction(I2CTransaction &transaction) = 0;
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
    static PoolAllocator TransactionPool;
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
        TransferAdder & frequency(uint32_t hz);
        TransferAdder & callback(const detail::I2CTransaction::event_callback_t& callback,
            int event = I2C_EVENT_TRANSFER_COMPLETE);
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
    int _hz;
    size_t ownerID;
};

} // namespace mbed

#endif

#endif
