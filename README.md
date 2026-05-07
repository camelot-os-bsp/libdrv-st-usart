# libdrv-st-usart

`libdrv-st-usart` is a userspace STM32 USART/UART driver built for the Merlin ecosystem.
It provides a small, label-based API to probe, initialize, use, and release USART peripherals from device-tree metadata.

## building the driver

```bash
$ defconfig configs/defconfig
$ meson setup --cross-file cm33-sentry-camelot-eabi-gcc.ini -Ddts=dts/sample.dts -Dconfig=.config -Ddts-include-dirs=$SDK_INSTALL_BASE/share/dts --reconfigure --wipe builddir
```

## Context: Merlin Integration

This driver is designed to be used with [Merlin](https://github.com/camelot-os/merlin).
It relies on Merlin platform services for:

- Device registration from DTS labels
- Device mapping/unmapping
- GPIO configuration for USART pins
- IRQ dispatch and acknowledge path
- Memory-mapped I/O helpers and polling primitives

In Meson, `libdrv-st-usart` links against Merlin (`merlin_dep`) and `shield`.

## Scope and Compatibility

- Target IP: STM32 USART/UART block shared across STM32 families (F0/F1/F2/F3/F4/F7/G0/G4/H5/H7/L0/L4/U5/WB/WL...)
- Multiple instances: up to `STM32_USART_MAX_INSTANCES` (currently 4)
- Instance identifier: DTS label (`uint32_t`), passed to all public APIs

## Public API

Header: `include/usart/usart_driver.h`

- `int stm32_usart_probe(uint32_t label)`
	Register one USART instance in Merlin from its DTS label.

- `int stm32_usart_init(uint32_t label, const struct usart_config *cfg)`
	Map device, configure GPIOs, apply USART line configuration, enable IRQs.

- `int stm32_usart_write(uint32_t label, const uint8_t data)`
	Transmit one byte (polled TX).

- `int stm32_usart_read(uint32_t label, uint8_t *rdbuf)`
	Read one byte from ISR-captured RX data.
	Returns:
	- `0`: one byte copied to `rdbuf[0]`
	- `1`: no byte available yet
	- `-1`: error

- `int stm32_usart_flush(uint32_t label)`
	Wait for transmission complete (TC flag).

- `int stm32_usart_release(uint32_t label)`
	Disable peripheral, unmap device, and free the instance slot.

## Runtime Behavior

- TX path: blocking polled write on TXE, byte by byte
- RX path: interrupt-driven capture (RXNE), byte consumed via `stm32_usart_read`
- Error handling: RX error flags are cleared in ISR via ICR

The typical RX flow is:

1. Application waits for an interrupt event.
2. Merlin dispatches ISR to the driver.
3. Application calls `stm32_usart_read` to consume the received byte.

## Minimal Usage Sequence

Here is a basic `main()` implementation that support a sample usart-driven event loop.
Not that this is a very simple, not hardened, code implementation that illustrate how the
driver can be used in an application event loop.

```c
  #define USART3_LABEL 0x103U

  uint8_t rx_char;
  const struct usart_config uart3_cfg = {
        .baudrate     = 115200U,
        .mode         = USART_MODE_ASYNCHRONOUS,
        .parity       = USART_PARITY_NONE,
        .stop_bits    = USART_STOP_BITS_1,
        .word_length  = USART_WORD_LENGTH_8,
        .flow_control = USART_FLOW_CONTROL_NONE,
        .tx_enable    = true,
        .rx_enable    = true,
    };

    if (usart_probe(USART3_LABEL) != 0) {
        goto err;
    }

    if (usart_init(USART3_LABEL, &uart3_cfg) != 0) {
        goto err;
    }

    for (;;) {
        /* kernel input event buffer */
        uint8_t event_buf[128];
        exchange_event_t *event = (exchange_event_t *)event_buf;
        uint32_t *IRQn = (uint32_t *)(&event->data[0]);

        /* wait for event as long as enough */
        if (__sys_wait_for_event(EVENT_TYPE_IRQ, 0) != STATUS_OK) {
            /* nothing received, yield CPU */
            __sys_sched_yield();
            continue;
        }
        /* get back IRQ event from kernel */
        copy_from_kernel(event_buf, 128UL);

        /* get back IRQn, encoded 32 bits, from data tab field and ask merlin to dispatch */
        merlin_platform_driver_irq_displatch(*IRQn);

        if (usart_read(USART3_LABEL, &rx_char) != 0) {
            continue;
        }
        /* do something with the received char */

        /* emit a char if needed */
        if (usart_write(USART3_LABEL, '.'))
    }
```
