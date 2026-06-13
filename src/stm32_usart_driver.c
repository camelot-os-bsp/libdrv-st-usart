// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026 H2Lab Development Team

/*
 * Generic STM32 USART/UART bus driver example for the Merlin platform.
 *
 * Compatible with the USART IP block present in all STM32 product families
 * (F0/F1/F2/F3/F4/F7/G0/G4/H5/H7/L0/L4/U5/WB/WL …).  The register layout
 * is identical across these families (USART_CR1, USART_CR2, USART_CR3,
 * USART_BRR, USART_ISR, USART_ICR, USART_RDR, USART_TDR).
 *
 * The driver supports up to STM32_USART_MAX_INSTANCES controllers
 * simultaneously.  Each instance is identified by its DTS label; all public
 * functions take that label as first argument so a single task can drive
 * several USART/UART peripherals at the same time.
 *
 * Operation modes:
 * - TX (Transmit): Polled mode. Each byte transmission is verified by polling
 *   the TXE flag to ensure the character is fully accepted before proceeding
 *   to the next one.
 * - RX (Receive): Interrupt mode. The RXNE interrupt is enabled. When a
 *   character is received, the ISR sets a flag and stores the character data.
 *   The read function checks this flag; if set, it reads the character and
 *   clears the flag. If not set, it returns a code indicating no character
 *   is available yet.
 *
 * Expected application usage flow for RX:
 * 1. Application waits for an IRQ event from the kernel
 * 2. Application calls dispatch_isr via merlin, which invokes the driver's ISR
 * 3. Application calls the read function to retrieve the received character
 */

#include <inttypes.h>
#include <stddef.h>
#include <stdbool.h>

#include <merlin/io.h>

#include <usart/usart_driver.h>

/* -------------------------------------------------------------------------
 * STM32 USART/UART driver private state structure
 * ---------------------------------------------------------------------- */
typedef struct {
	bool rxne_received;     /**< Flag set by the ISR when a byte is received. */
	uint8_t rxne_data;      /**< Byte captured by the ISR. */
} stm32_usart_private_t;

/* -------------------------------------------------------------------------
 * STM32 USART/UART register offsets
 * Identical across F0/F1/F2/F3/F4/F7/G0/G4/H5/H7/L0/L4/U5/WB series.
 * ---------------------------------------------------------------------- */
#define USART_CR1_OFFSET    0x00UL
#define USART_CR2_OFFSET    0x04UL
#define USART_CR3_OFFSET    0x08UL
#define USART_BRR_OFFSET    0x0CUL
#define USART_ISR_OFFSET    0x1CUL
#define USART_ICR_OFFSET    0x20UL
#define USART_RDR_OFFSET    0x24UL
#define USART_TDR_OFFSET    0x28UL

/* CR1 bits */
#define USART_CR1_UE        (1UL << 0)   /**< USART enable */
#define USART_CR1_RE        (1UL << 2)   /**< Receiver enable */
#define USART_CR1_TE        (1UL << 3)   /**< Transmitter enable */
#define USART_CR1_RXNEIE    (1UL << 5)   /**< RXNE interrupt enable */
#define USART_CR1_TXEIE     (1UL << 7)   /**< TXE interrupt enable */
#define USART_CR1_PCE       (1UL << 10)  /**< Parity control enable */
#define USART_CR1_PS        (1UL << 9)   /**< Parity selection: 0=even, 1=odd */
#define USART_CR1_M0        (1UL << 12)  /**< Word length bit 0 */
#define USART_CR1_M1        (1UL << 28)  /**< Word length bit 1 */
#define USART_CR1_OVER8     (1UL << 15)  /**< Oversampling by 8 */

/* CR2 bits */
#define USART_CR2_STOP_SHIFT    12U
#define USART_CR2_STOP_MASK     (0x3UL << USART_CR2_STOP_SHIFT)
#define USART_CR2_STOP_1        (0x0UL << USART_CR2_STOP_SHIFT)
#define USART_CR2_STOP_0_5      (0x1UL << USART_CR2_STOP_SHIFT)
#define USART_CR2_STOP_2        (0x2UL << USART_CR2_STOP_SHIFT)
#define USART_CR2_STOP_1_5      (0x3UL << USART_CR2_STOP_SHIFT)
#define USART_CR2_CLKEN         (1UL << 11) /**< Clock enable (synchronous mode) */

/* CR3 bits */
#define USART_CR3_RTSE      (1UL << 8)  /**< RTS enable */
#define USART_CR3_CTSE      (1UL << 9)  /**< CTS enable */

/* ISR bits */
#define USART_ISR_RXNE      (1UL << 5)  /**< Read data register not empty */
#define USART_ISR_TC        (1UL << 6)  /**< Transmission complete */
#define USART_ISR_TXE       (1UL << 7)  /**< Transmit data register empty */
#define USART_ISR_ORE       (1UL << 3)  /**< Overrun error */
#define USART_ISR_NE        (1UL << 2)  /**< Noise error */
#define USART_ISR_FE        (1UL << 1)  /**< Framing error */
#define USART_ISR_PE        (1UL << 0)  /**< Parity error */

/* ICR bits */
#define USART_ICR_ORECF     (1UL << 3)
#define USART_ICR_NECF      (1UL << 2)
#define USART_ICR_FECF      (1UL << 1)
#define USART_ICR_PECF      (1UL << 0)
#define USART_ICR_TCCF      (1UL << 6)

#define USART_ERROR_MASK    (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)
#define USART_ERROR_CLR     (USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF | USART_ICR_PECF)

#define USART_POLL_RETRIES  10000U

static int stm32_usart_fops_configure(struct usart_driver *self,
                                      const struct usart_config *config);
static int stm32_usart_fops_write(struct usart_driver *self,
                                  const uint8_t *wrbuf, size_t len);
static int stm32_usart_fops_read(struct usart_driver *self,
                                 uint8_t *rdbuf, size_t len);
static int stm32_usart_fops_flush(struct usart_driver *self);

static int stm32_usart_isr(void *self, uint32_t IRQn);

/*
 * Each slot is initialised at probe time. All instances share the same
 * fops table since the STM32 USART/UART IP is register-compatible across
 * all families.
 */
static struct usart_driver g_usart_instances[STM32_USART_MAX_INSTANCES];
static stm32_usart_private_t g_usart_private[STM32_USART_MAX_INSTANCES];
static bool g_usart_slot_used[STM32_USART_MAX_INSTANCES];

/**
 * @brief Look up an active USART instance by DTS label.
 * @param label DTS label used at probe time.
 * @return Pointer to the matching instance, or NULL when not found.
 */
static struct usart_driver *stm32_usart_instance_get(uint32_t label)
{
    for (size_t i = 0U; i < STM32_USART_MAX_INSTANCES; i++) {
        if (g_usart_slot_used[i] &&
            g_usart_instances[i].platform.label == label) {
            return &g_usart_instances[i];
        }
    }
    return NULL;
}

/**
 * @brief Allocate and initialise a free USART instance slot.
 * @return Pointer to an initialised slot, or NULL when all slots are used.
 */
static struct usart_driver *stm32_usart_instance_alloc(void)
{
    for (size_t i = 0U; i < STM32_USART_MAX_INSTANCES; i++) {
        if (!g_usart_slot_used[i]) {
            struct usart_driver *drv = &g_usart_instances[i];
            stm32_usart_private_t *priv = &g_usart_private[i];

            priv->rxne_received = false;
            priv->rxne_data = 0U;

            drv->platform.devh = 0;
            drv->platform.label = 0UL;
            drv->platform.devinfo = NULL;
            drv->platform.name = "stm32 usart/uart driver";
            drv->platform.compatible = "st,stm32-usart";
            drv->platform.platform_fops.isr = stm32_usart_isr;
            drv->platform.type = DEVICE_TYPE_USART;
            drv->platform.private_data = priv;
            drv->private_data = priv;

            g_usart_slot_used[i] = true;
            return drv;
        }
    }
    return NULL;
}

/**
 * @brief Release an allocated USART instance slot.
 * @param drv Instance pointer previously returned by the allocator.
 */
static void stm32_usart_instance_free(struct usart_driver *drv)
{
    for (size_t i = 0U; i < STM32_USART_MAX_INSTANCES; i++) {
        if (&g_usart_instances[i] == drv) {
            g_usart_private[i].rxne_received = false;
            g_usart_private[i].rxne_data = 0U;
            drv->platform.platform_fops.isr = NULL;
            drv->platform.private_data = NULL;
            drv->platform.devh = 0;
            drv->platform.label = 0UL;
            drv->platform.devinfo = NULL;
            drv->private_data = NULL;
            g_usart_slot_used[i] = false;
            return;
        }
    }
}

/**
 * @brief Compute the absolute register address for an instance.
 * @param drv USART instance.
 * @param offset Register offset from peripheral base.
 * @return Absolute memory-mapped register address.
 */
static inline size_t usart_reg(const struct usart_driver *drv, size_t offset)
{
    return ((size_t)drv->platform.devinfo->baseaddr + offset);
}

/**
 * @brief Read a 32-bit USART register.
 * @param drv USART instance.
 * @param offset Register offset from peripheral base.
 * @return 32-bit register value.
 */
static inline uint32_t usart_read32(const struct usart_driver *drv,
                                    size_t offset)
{
    return merlin_ioread32(usart_reg(drv, offset));
}

/**
 * @brief Write a 32-bit USART register.
 * @param drv USART instance.
 * @param offset Register offset from peripheral base.
 * @param value Value to write.
 */
static inline void usart_write32(const struct usart_driver *drv,
                                 size_t offset, uint32_t value)
{
    merlin_iowrite32(usart_reg(drv, offset), value);
}

/**
 * @brief Check whether a USART instance is mapped and ready.
 * @param drv USART instance.
 * @return true when mapped and device metadata is available, false otherwise.
 */
static inline bool usart_is_ready(const struct usart_driver *drv)
{
    return (drv->platform.devh != 0) &&
           (drv->platform.devinfo != NULL) &&
           (drv->private_data != NULL);
}

/**
 * @brief Disable USART peripheral.
 * @param drv USART instance.
 */
static void stm32_usart_disable(struct usart_driver *drv)
{
    uint32_t cr1 = usart_read32(drv, USART_CR1_OFFSET);
    cr1 &= ~USART_CR1_UE;
    usart_write32(drv, USART_CR1_OFFSET, cr1);
}

/**
 * @brief Enable USART peripheral.
 * @param drv USART instance.
 */
static void stm32_usart_enable(struct usart_driver *drv)
{
    uint32_t cr1 = usart_read32(drv, USART_CR1_OFFSET);
    cr1 |= USART_CR1_UE;
    usart_write32(drv, USART_CR1_OFFSET, cr1);
}

/**
 * @brief Wait until TX data register is empty.
 * @param drv USART instance.
 * @return 0 on success, -1 on timeout.
 */
static int stm32_usart_wait_txe(struct usart_driver *drv)
{
    if (merlin_iopoll32_until_set(usart_reg(drv, USART_ISR_OFFSET),
                                  USART_ISR_TXE,
                                  USART_POLL_RETRIES) != STATUS_OK) {
        return -1;
    }
    return 0;
}

/**
 * @brief Wait until RX data register contains a byte.
 * @param drv USART instance.
 * @return 0 on success, -1 on timeout.
 */
static int stm32_usart_wait_rxne(struct usart_driver *drv)
{
    if (merlin_iopoll32_until_set(usart_reg(drv, USART_ISR_OFFSET),
                                  USART_ISR_RXNE,
                                  USART_POLL_RETRIES) != STATUS_OK) {
        return -1;
    }
    return 0;
}

/**
 * @brief Wait until transmission is fully complete.
 * @param drv USART instance.
 * @return 0 on success, -1 on timeout.
 */
static int stm32_usart_wait_tc(struct usart_driver *drv)
{
    if (merlin_iopoll32_until_set(usart_reg(drv, USART_ISR_OFFSET),
                                  USART_ISR_TC,
                                  USART_POLL_RETRIES) != STATUS_OK) {
        return -1;
    }
    return 0;
}

/**
 * @brief Check and clear RX error flags.
 * @param drv USART instance.
 * @return 0 when no error is present, -1 when an error was detected.
 */
static int stm32_usart_check_and_clear_rx_errors(struct usart_driver *drv)
{
    const uint32_t isr = usart_read32(drv, USART_ISR_OFFSET);

    if ((isr & USART_ERROR_MASK) != 0UL) {
        usart_write32(drv, USART_ICR_OFFSET, USART_ERROR_CLR);
        return -1;
    }
    return 0;
}

/**
 * @brief Configure USART line parameters.
 * @param self USART instance.
 * @param config Desired USART configuration.
 * @return 0 on success, -1 on invalid parameters or hardware failure.
 */
static int stm32_usart_fops_configure(struct usart_driver *self,
                                      const struct usart_config *config)
{
    uint32_t cr1;
    uint32_t cr2;
    uint32_t cr3;
    uint32_t busfreq_mhz;
    uint32_t brr;

    if (self == NULL || config == NULL) {
        return -1;
    }

    if (!usart_is_ready(self)) {
        return -1;
    }

    stm32_usart_disable(self);

    if (config->baudrate == 0U) {
        return -1;
    }

    /*
     * BRR in 16x oversampling mode: BRR = f_CK / baudrate.
     * f_CK is retrieved through the generic Merlin platform API and is
     * returned in MHz (as provided by generated *_dt.h metadata).
     */
    if (merlin_platform_driver_get_bus_clock(&self->platform, &busfreq_mhz) != STATUS_OK) {
        return -1;
    }

    brr = (uint32_t)(((uint64_t)busfreq_mhz * 1000000ULL) / (uint64_t)config->baudrate);
    if (brr == 0U) {
        return -1;
    }
    usart_write32(self, USART_BRR_OFFSET, brr);

    /* CR1: word length, parity, TX/RX enable */
    cr1 = usart_read32(self, USART_CR1_OFFSET);
    cr1 &= ~(USART_CR1_M0 | USART_CR1_M1 | USART_CR1_PCE | USART_CR1_PS |
             USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8);

    switch (config->word_length) {
        case USART_WORD_LENGTH_7:
            cr1 |= USART_CR1_M1;
            break;
        case USART_WORD_LENGTH_9:
            cr1 |= USART_CR1_M0;
            break;
        case USART_WORD_LENGTH_8:
        default:
            break;
    }

    switch (config->parity) {
        case USART_PARITY_ODD:
            cr1 |= USART_CR1_PCE | USART_CR1_PS;
            break;
        case USART_PARITY_EVEN:
            cr1 |= USART_CR1_PCE;
            break;
        case USART_PARITY_NONE:
        default:
            break;
    }

    if (config->tx_enable) {
        cr1 |= USART_CR1_TE;
    }
    if (config->rx_enable) {
        cr1 |= USART_CR1_RE;
    }

    /*
     * RX interrupt mode: enable RXNE interrupt so the ISR is called when
     * a character is received. The ISR will set a flag that the read function
     * checks to know whether a character is available.
     */
    if (config->rx_enable) {
        cr1 |= USART_CR1_RXNEIE;
    }

    usart_write32(self, USART_CR1_OFFSET, cr1);

    /* CR2: stop bits, synchronous clock */
    cr2 = usart_read32(self, USART_CR2_OFFSET);
    cr2 &= ~(USART_CR2_STOP_MASK | USART_CR2_CLKEN);

    switch (config->stop_bits) {
        case USART_STOP_BITS_0_5:
            cr2 |= USART_CR2_STOP_0_5;
            break;
        case USART_STOP_BITS_2:
            cr2 |= USART_CR2_STOP_2;
            break;
        case USART_STOP_BITS_1_5:
            cr2 |= USART_CR2_STOP_1_5;
            break;
        case USART_STOP_BITS_1:
        default:
            cr2 |= USART_CR2_STOP_1;
            break;
    }

    if (config->mode == USART_MODE_SYNCHRONOUS) {
        cr2 |= USART_CR2_CLKEN;
    }
    usart_write32(self, USART_CR2_OFFSET, cr2);

    /* CR3: hardware flow control */
    cr3 = usart_read32(self, USART_CR3_OFFSET);
    cr3 &= ~(USART_CR3_RTSE | USART_CR3_CTSE);

    switch (config->flow_control) {
        case USART_FLOW_CONTROL_RTS:
            cr3 |= USART_CR3_RTSE;
            break;
        case USART_FLOW_CONTROL_CTS:
            cr3 |= USART_CR3_CTSE;
            break;
        case USART_FLOW_CONTROL_RTS_CTS:
            cr3 |= USART_CR3_RTSE | USART_CR3_CTSE;
            break;
        case USART_FLOW_CONTROL_NONE:
        default:
            break;
    }
    usart_write32(self, USART_CR3_OFFSET, cr3);

    stm32_usart_enable(self);

    merlin_platform_driver_enable_irqs(&self->platform);
    return 0;
}

/**
 * @brief Write a single byte on USART TX.
 * @param self USART instance.
 * @param wrbuf Buffer to transmit (only first byte is used).
 * @param len Ignored (for API compatibility).
 * @return 0 on success, -1 on invalid parameters or hardware failure.
 */
static int stm32_usart_fops_write(struct usart_driver *self,
                                  const uint8_t *wrbuf, size_t len)
{
    (void)len;  /* Unused: driver handles one byte at a time */

    if (self == NULL || wrbuf == NULL) {
        return -1;
    }

    if (!usart_is_ready(self)) {
        return -1;
    }

    /* Wait for TX data register to be empty */
    if (stm32_usart_wait_txe(self) != 0) {
        return -1;
    }

    /* Send the character */
    merlin_iowrite8(usart_reg(self, USART_TDR_OFFSET), wrbuf[0]);

    return 0;
}

/**
 * @brief Read a single byte from USART RX.
 * @param self USART instance.
 * @param rdbuf Destination buffer (only first byte is written).
 * @param len Ignored (for API compatibility).
 * @return 0 on success (character read from RX ISR flag),
 *         1 when no character is available yet,
 *         -1 on invalid parameters or hardware failure.
 */
static int stm32_usart_fops_read(struct usart_driver *self,
                                 uint8_t *rdbuf, size_t len)
{
    (void)self;
    (void)rdbuf;
    (void)len;

    /* RX is handled directly by the public API; this helper remains a stub. */
    return 0;
}

/**
 * @brief Wait until USART TX is drained.
 * @param self USART instance.
 * @return 0 on success, -1 on timeout or invalid state.
 */
static int stm32_usart_fops_flush(struct usart_driver *self)
{
    if (self == NULL || !usart_is_ready(self)) {
        return -1;
    }

    if (stm32_usart_wait_tc(self) != 0) {
        return -1;
    }

    usart_write32(self, USART_ICR_OFFSET, USART_ICR_TCCF);

    return 0;
}

/**
 * @brief ISR callback bound to the platform driver entry.
 * @param self Pointer to struct platform_device_driver.
 * @param IRQn Interrupt line number.
 * @return 0 if the matching instance was found and acknowledged, -1 otherwise.
 */
static int stm32_usart_isr(void *self, uint32_t IRQn)
{
    struct platform_device_driver *pdrv = (struct platform_device_driver *)self;
    stm32_usart_private_t *priv;

    (void)IRQn;

    /*
     * Resolve the owning usart_driver from the platform_device_driver
     * pointer by scanning the instance table.
     */
    for (size_t i = 0U; i < STM32_USART_MAX_INSTANCES; i++) {
        if (!g_usart_slot_used[i]) {
            continue;
        }
        if (&g_usart_instances[i].platform == pdrv) {
            struct usart_driver *drv = &g_usart_instances[i];
            const uint32_t isr = usart_read32(drv, USART_ISR_OFFSET);

            if (!usart_is_ready(drv)) {
                return -1;
            }

            priv = (stm32_usart_private_t *)drv->private_data;
            if (priv == NULL) {
                return -1;
            }

            /*
             * In interrupt mode:
             * - If RXNE flag is set, read the received character and set
             *   the flag to indicate it's available for the read function.
             * - Clear error flags to prevent the peripheral from hanging.
             * - Clear TC flag if it was set.
             */
            if ((isr & USART_ISR_RXNE) != 0UL) {
                priv->rxne_data = merlin_ioread8(usart_reg(drv, USART_RDR_OFFSET));
                priv->rxne_received = true;
            }

            /* Clear error flags and TC flag */
            usart_write32(drv, USART_ICR_OFFSET,
                          USART_ERROR_CLR | USART_ICR_TCCF);

            return 0;
        }
    }

    return -1;
}

/**
 * @brief Register a USART instance in Merlin.
 * @param label DTS label of the target USART peripheral.
 * @return DRV_STATUS_OK on success.
 * @return DRV_ERROR_INVSTATE if no free instance slot is available.
 * @return DRV_ERROR_CONFIGURATION if the DTS label cannot be resolved.
 * @return DRV_ERROR_NOTREGISTERED if Merlin rejects the registration.
 */
drv_status_t stm32_usart_probe(uint32_t label)
{
    struct usart_driver *drv = stm32_usart_instance_alloc();

    if (drv == NULL) {
        return DRV_ERROR_INVSTATE;
    }

    if (merlin_platform_driver_register(&drv->platform, label) != STATUS_OK) {
        stm32_usart_instance_free(drv);
        return DRV_ERROR_CONFIGURATION;
    }

    return DRV_STATUS_OK;
}

/**
 * @brief Map and configure a registered USART instance.
 * @param label DTS label of the target USART peripheral.
 * @param cfg Line configuration to apply.
 * @return DRV_STATUS_OK on success.
 * @return DRV_ERROR_INVSTATE if the instance is not registered.
 * @return DRV_ERROR_INVPARAM if cfg is NULL.
 * @return DRV_ERROR_CONFIGURATION if mapping, GPIO setup, or line setup fails.
 */
drv_status_t stm32_usart_init(uint32_t label, const struct usart_config *cfg)
{
    struct usart_driver *drv = stm32_usart_instance_get(label);

    if (drv == NULL || cfg == NULL) {
        return (drv == NULL) ? DRV_ERROR_INVSTATE : DRV_ERROR_INVPARAM;
    }

    if (merlin_platform_driver_map(&drv->platform) != STATUS_OK) {
        return DRV_ERROR_CONFIGURATION;
    }

    if (merlin_platform_driver_configure_gpio(&drv->platform) != STATUS_OK) {
        return DRV_ERROR_CONFIGURATION;
    }

    if (stm32_usart_fops_configure(drv, cfg) != 0) {
        return DRV_ERROR_CONFIGURATION;
    }

    return DRV_STATUS_OK;
}

/**
 * @brief Transmit one byte on a specific USART instance.
 * @param label DTS label of the target USART peripheral.
 * @param data Byte to transmit.
 * @return DRV_STATUS_OK on success.
 * @return DRV_ERROR_INVSTATE if the instance is not initialised.
 * @return DRV_ERROR_AGAIN if the transmitter is temporarily busy.
 */
drv_status_t stm32_usart_write(uint32_t label, const uint8_t data)
{
    struct usart_driver *drv = stm32_usart_instance_get(label);

    if (drv == NULL) {
        return DRV_ERROR_INVSTATE;
    }

    if (stm32_usart_fops_write(drv, &data, 1U) != 0) {
        return DRV_ERROR_AGAIN;
    }

    return DRV_STATUS_OK;
}

/**
 * @brief Read one byte from a specific USART instance.
 *
 * @param label DTS label of the target USART peripheral.
 * @param rdbuf Destination buffer.
 * @return DRV_STATUS_OK on success.
 * @return DRV_ERROR_INVSTATE if the instance is not initialised.
 * @return DRV_ERROR_INVPARAM if rdbuf is NULL.
 * @return DRV_ERROR_AGAIN if no byte is currently available.
 */
drv_status_t stm32_usart_read(uint32_t label, uint8_t *rdbuf)
{
    struct usart_driver *drv = stm32_usart_instance_get(label);
    stm32_usart_private_t *priv;

    if (drv == NULL) {
        return DRV_ERROR_INVSTATE;
    }

    if (rdbuf == NULL) {
        return DRV_ERROR_INVPARAM;
    }

    priv = (stm32_usart_private_t *)drv->private_data;
    if (priv == NULL) {
        return DRV_ERROR_INVSTATE;
    }

    if (priv->rxne_received) {
        rdbuf[0] = priv->rxne_data;
        priv->rxne_received = false;
        return DRV_STATUS_OK;
    }

    return DRV_ERROR_AGAIN;
}

/**
 * @brief Wait for pending TX data to leave the peripheral.
 * @param label DTS label of the target USART peripheral.
 * @return DRV_STATUS_OK on success.
 * @return DRV_ERROR_INVSTATE if the instance is not initialised.
 * @return DRV_ERROR_TIMEOUT if the transmission complete flag never asserts.
 */
drv_status_t stm32_usart_flush(uint32_t label)
{
    struct usart_driver *drv = stm32_usart_instance_get(label);

    if (drv == NULL) {
        return DRV_ERROR_INVSTATE;
    }

    if (stm32_usart_fops_flush(drv) != 0) {
        return DRV_ERROR_TIMEOUT;
    }

    return DRV_STATUS_OK;
}


/**
 * @brief Disable and unmap a USART instance, then free its slot.
 * @param label DTS label of the target USART peripheral.
 * @return DRV_STATUS_OK on success.
 * @return DRV_ERROR_INVSTATE if the instance is not initialised.
 * @return DRV_ERROR_CONFIGURATION if unmapping fails.
 */
drv_status_t stm32_usart_release(uint32_t label)
{
    struct usart_driver *drv = stm32_usart_instance_get(label);

    if (drv == NULL) {
        return DRV_ERROR_INVSTATE;
    }

    stm32_usart_disable(drv);

    if (merlin_platform_driver_unmap(&drv->platform) != STATUS_OK) {
        return DRV_ERROR_CONFIGURATION;
    }

    stm32_usart_instance_free(drv);

    return DRV_STATUS_OK;
}

/* Aliases for the exported API functions */
drv_status_t usart_probe(uint32_t label) __attribute__((alias("stm32_usart_probe")));
drv_status_t usart_init(uint32_t label, const struct usart_config *cfg) __attribute__((alias("stm32_usart_init")));
drv_status_t usart_write(uint32_t label, const uint8_t data) __attribute__((alias("stm32_usart_write")));
drv_status_t usart_read(uint32_t label, uint8_t *rdbuf) __attribute__((alias("stm32_usart_read")));
drv_status_t usart_flush(uint32_t label) __attribute__((alias("stm32_usart_flush")));
drv_status_t usart_release(uint32_t label) __attribute__((alias("stm32_usart_release")));
