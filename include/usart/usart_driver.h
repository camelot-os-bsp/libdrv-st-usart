// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 H2Lab Development Team

#ifndef STM32_USART_DRIVER_H
#define STM32_USART_DRIVER_H

#include <inttypes.h>
#include <merlin/buses/usart.h>

/**
 * Maximum number of USART/UART instances this driver can manage
 * simultaneously.  Increase as needed for targets with more serial
 * controllers.
 */
#define STM32_USART_MAX_INSTANCES 4U

/**
 * @brief Probe a STM32 USART/UART controller.
 *
 * Registers the driver instance into Merlin using the DTS label that
 * uniquely identifies the controller peripheral.  This only performs
 * registration; the device is neither mapped nor initialised at this
 * stage.
 *
 * This driver is compatible with the USART IP block shared across the
 * STM32 product families (F0/F1/F2/F3/F4/F7/G0/G4/H5/H7/L0/L4/U5/WB…).
 *
 * @param label sentry,label value as declared in the DTS node
 *
 * @return 0 on success, -1 if no free instance slot is available or
 *         if the DTS registration fails
 */
int usart_probe(uint32_t label);

/**
 * @brief Initialise a STM32 USART/UART controller.
 *
 * Maps the device into the caller address space, configures the associated
 * GPIOs (TX/RX and optional CTS/RTS pins) and programs the line parameters.
 * Must be called after a successful stm32_usart_probe().
 *
 * @param label DTS label identifying the controller instance
 * @param cfg   line configuration (baudrate, parity, stop bits, …)
 *
 * @return 0 on success, -1 on failure
 */
int usart_init(uint32_t label, const struct usart_config *cfg);

/**
 * @brief Transmit a byte buffer over a USART/UART TX path.
 *
 * Blocking, polled write.  Each byte is sent only after the previous one
 * has been accepted by the transmit data register.
 *
 * @param label DTS label identifying the controller instance
 * @param wrbuf bytes to transmit
 * @param len   number of bytes to transmit
 *
 * @return 0 on success, -1 on failure
 */
int usart_write(uint32_t label, const uint8_t data);

/**
 * @brief Receive bytes from a USART/UART RX path.
 *
 * Blocking, polled read.  The function waits for each byte to arrive in
 * the receive data register before advancing.
 *
 * @param label DTS label identifying the controller instance
 * @param rdbuf buffer used to store received bytes
 * @param len   number of bytes to receive
 *
 * @return 0 on success, -1 on failure
 */
int usart_read(uint32_t label, uint8_t *rdbuf);

/**
 * @brief Wait for all pending TX data to be shifted out.
 *
 * Polls the Transmission Complete flag until it is set or the retry
 * counter expires.
 *
 * @param label DTS label identifying the controller instance
 *
 * @return 0 on success, -1 if the TC flag was not set in time
 */
int usart_flush(uint32_t label);

/**
 * @brief Release a USART/UART controller.
 *
 * Disables the controller, unmaps the device from the caller address
 * space and frees the instance slot so it can be re-probed.
 *
 * @param label DTS label identifying the controller instance
 *
 * @return 0 on success, -1 on failure
 */
int usart_release(uint32_t label);



#endif /* STM32_USART_DRIVER_H */
