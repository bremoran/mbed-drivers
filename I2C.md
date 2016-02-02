# Asynchronous I2C
I2C is a 2-wire serial bus protocol, designed to provide easy communication between peripherals on the same circuit board.

The I2C class provides an asynchronous usage model for I2C master mode.

In order to facilitate sharing an I2C master mode controller between drivers for connected peripherals, an I2C resource manager is required. This resource manager provides a single interface: ```post_transaction()```, an API which adds a transaction to the transaction queue.

Different resource managers are required for each kind of I2C master. By default, only the resource manager matching the on-chip I2C master is provided, however it is possible to add others by inheriting from the ```I2CResourceManager``` class and extending the ```I2COwners``` array.

Transaction queues are composed of one or more transmit or receive segments, each of which contains a buffer, a direction and, optionally, an callback to execute in IRQ context between finishing the current segment and starting the next.

Typically, transactions should only be created in non-IRQ context, since they require allocating memory. However, if two pool allocators (one for the ```I2CTransaction``` and one for the ```I2CSegments```) is provided to the ```I2C``` object on construction, it can use ```transfer_to_irqsafe``` to build a transaction in IRQ context.

# Example (non-IRQ context)

```C++
using namespace mbed_drivers::v1;
void done(I2CTransaction * t, int event) {
    printf("Transaction completed with code %d (%s)\r\n", event,
        event == I2C_EVENT_TRANSFER_COMPLETE? "done" : "error");
    uint32_t rxData;
    memcpy(&rxData, t->root->next->get_buf(), sizeof(uint32_t));
    printf("Data received: %08X\r\n", rxData);
}

I2C i2c(scl, sda);
void app_start(int, char**) {
    i2c.transfer_to(address).tx("\xAA\x55", 2).rx(4).on(I2C_EVENT_TRANSFER_COMPLETE, done);
}

```
