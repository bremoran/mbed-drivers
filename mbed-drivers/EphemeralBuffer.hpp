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
#ifndef MBED_DRIVERS_EPHEMERAL_BUFFER_H
#define MBED_DRIVERS_EPHEMERAL_BUFFER_H

#include <cstdint>
#include "mbed-drivers/Buffer.h"

namespace mbed {
using std::uint8_t;
/**
 * The EphemeralBuffer class is a variant of the Buffer class.
 * Instead of just storing a buffer pointer and a size, if the buffer is less than
 * 8 bytes long, it packs the whole buffer into the space occupied by the pointer and
 * size variable. This is indicated by setting the MSB in size.
 */
class EphemeralBuffer {
public:
    /**
     * Set buffer pointer and length.
     *
     * @param[in] b A buffer to duplicate
     */
    void set(const Buffer & b);
    /**
     * Set buffer pointer and length.
     * If the buffer is 7 or fewer bytes, copy it into the contents of EphemeralBuffer
     * instead of keeping a pointer to it.
     *
     * @param[in] b A buffer to duplicate
     */
    void set_ephemeral(const Buffer & b);
    /**
     * Set the buffer pointer and length
     *
     * @param[in] buf the buffer pointer to duplicate
     * @param[in] len the length of the buffer to duplicate
     */
    void set(void * buf, size_t len);
    /**
     * Set buffer pointer and length.
     * If the buffer is 7 or fewer bytes, copy it into the contents of EphemeralBuffer
     * instead of keeping a pointer to it.
     *
     * @param[in] buf the buffer pointer to duplicate
     * @param[in] len the length of the buffer to duplicate
     */
    void set_ephemeral(void * buf, size_t len);
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

    /**
     * Check if the buffer is ephemeral.
     *
     * @retval true The buffer contains data, rather than a pointer
     * @retval false The buffer contains a pointer to data
     */
    bool is_ephemeral() const;
protected:
    union {
        struct {
            void * _dataPtr;
            size_t _ptrLen:31;
            unsigned _reserved:1;
        };
        struct {
            uint8_t _data[sizeof(void *) + sizeof(size_t) - 1];
            size_t _len:7;
            unsigned _ephemeral:1;
        };
    };
};
} // namespace mbed
#endif // MBED_DRIVERS_EPHEMERAL_BUFFER_H
