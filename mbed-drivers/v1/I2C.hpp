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
#ifndef MBED_DRIVERS_V1_I2C_HPP
#define MBED_DRIVERS_V1_I2C_HPP

#include "mbed-drivers/platform.h"

#if DEVICE_I2C && DEVICE_I2C_ASYNCH


#include "mbed-hal/i2c_api.h"
#include "mbed-hal/dma_api.h"

#include "mbed-drivers/CThunk.h"
#include "core-util/FunctionPointer.h"
#include "core-util/PoolAllocator.h"

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
 * void doneCB(bool dir, EphemeralBuffer buf, int Event) {
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
} // namespace v1
} // namespace mbed_drivers

#endif

#endif // MBED_DRIVERS_V1_I2C_HPP
