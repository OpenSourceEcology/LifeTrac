#include "host_uart.h"
#include "platform.h"
#include "stm32l072_regs.h"

#include <stddef.h>
#include <string.h>

#define HOST_PROTOCOL_VER            1U
#define HOST_TYPE_ERR_PROTO          0xFEU

#define HOST_DMA_RX_CH               6U
#define HOST_DMA_RX_BUF_SIZE         512U
#define HOST_FRAME_QUEUE_DEPTH       4U
#define HOST_COBS_ENCODED_MAX        (HOST_COBS_MAX_LEN - 2U)

static volatile uint8_t s_dma_rx_buf[HOST_DMA_RX_BUF_SIZE];
static uint16_t s_dma_last_idx;

static uint8_t s_cobs_encoded[HOST_COBS_ENCODED_MAX];
static uint16_t s_cobs_encoded_len;
static uint8_t s_cobs_overflow;
static volatile uint8_t s_service_in_progress;

static host_frame_t s_frame_queue[HOST_FRAME_QUEUE_DEPTH];
static volatile uint8_t s_q_head;
static volatile uint8_t s_q_tail;
static volatile uint8_t s_q_count;

static uint32_t s_stats_dropped;
static uint32_t s_stats_errors;

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

static uint32_t usart_brr_from_baud(uint32_t baud) {
    if (baud == 0UL) {
        return 1UL;
    }

    return (platform_core_hz() + (baud / 2UL)) / baud;
}

static void gpio_set_alt(uint32_t port, uint8_t pin, uint8_t af) {
    uint32_t moder = GPIO_MODER(port);
    moder &= ~(3UL << (pin * 2U));
    moder |= (2UL << (pin * 2U));
    GPIO_MODER(port) = moder;

    GPIO_OTYPER(port) &= ~(1UL << pin);

    uint32_t speed = GPIO_OSPEEDR(port);
    speed &= ~(3UL << (pin * 2U));
    speed |= (2UL << (pin * 2U));
    GPIO_OSPEEDR(port) = speed;

    if (pin < 8U) {
        uint32_t afrl = GPIO_AFRL(port);
        afrl &= ~(0xFUL << (pin * 4U));
        afrl |= ((uint32_t)af << (pin * 4U));
        GPIO_AFRL(port) = afrl;
    } else {
        const uint8_t pin_hi = (uint8_t)(pin - 8U);
        uint32_t afrh = GPIO_AFRH(port);
        afrh &= ~(0xFUL << (pin_hi * 4U));
        afrh |= ((uint32_t)af << (pin_hi * 4U));
        GPIO_AFRH(port) = afrh;
    }
}

static void usart2_gpio_init(void) {
    RCC_IOPENR |= RCC_IOPENR_GPIOAEN;

    gpio_set_alt(GPIOA_BASE, 2U, 4U);
    gpio_set_alt(GPIOA_BASE, 3U, 4U);

    /* Keep RX high when host is disconnected. */
    uint32_t pupd = GPIO_PUPDR(GPIOA_BASE);
    pupd &= ~(3UL << (3U * 2U));
    pupd |= (1UL << (3U * 2U));
    GPIO_PUPDR(GPIOA_BASE) = pupd;
}

static void dma_rx_start(void) {
    DMA1_CCR(HOST_DMA_RX_CH) &= ~DMA_CCR_EN;
    DMA1_IFCR = DMA_IFCR_CGIF(HOST_DMA_RX_CH) |
                DMA_IFCR_CTCIF(HOST_DMA_RX_CH) |
                DMA_IFCR_CHTIF(HOST_DMA_RX_CH) |
                DMA_IFCR_CTEIF(HOST_DMA_RX_CH);

    DMA1_CPAR(HOST_DMA_RX_CH) = (uint32_t)(uintptr_t)&USART2_RDR;
    DMA1_CMAR(HOST_DMA_RX_CH) = (uint32_t)(uintptr_t)s_dma_rx_buf;
    DMA1_CNDTR(HOST_DMA_RX_CH) = HOST_DMA_RX_BUF_SIZE;

    DMA1_CCR(HOST_DMA_RX_CH) = DMA_CCR_MINC |
                               DMA_CCR_CIRC |
                               DMA_CCR_TEIE |
                               DMA_CCR_PL_HIGH;
    DMA1_CCR(HOST_DMA_RX_CH) |= DMA_CCR_EN;

    s_dma_last_idx = 0U;
}

static uint8_t queue_push(const host_frame_t *frame) {
    if (s_q_count >= HOST_FRAME_QUEUE_DEPTH) {
        s_stats_dropped++;
        return 0U;
    }

    s_frame_queue[s_q_head] = *frame;
    s_q_head = (uint8_t)((s_q_head + 1U) % HOST_FRAME_QUEUE_DEPTH);
    s_q_count++;

    return 1U;
}

static uint8_t cobs_decode(const uint8_t *src,
                           uint16_t src_len,
                           uint8_t *dst,
                           uint16_t dst_cap,
                           uint16_t *dst_len) {
    uint16_t read = 0U;
    uint16_t write = 0U;

    while (read < src_len) {
        const uint8_t code = src[read++];
        if (code == 0U) {
            return 0U;
        }

        for (uint8_t i = 1U; i < code; ++i) {
            if (read >= src_len || write >= dst_cap) {
                return 0U;
            }
            dst[write++] = src[read++];
        }

        if (code != 0xFFU && read < src_len) {
            if (write >= dst_cap) {
                return 0U;
            }
            dst[write++] = 0U;
        }
    }

    *dst_len = write;
    return 1U;
}

static uint8_t cobs_encode(const uint8_t *src,
                           uint16_t src_len,
                           uint8_t *dst,
                           uint16_t dst_cap,
                           uint16_t *dst_len) {
    if (dst_cap == 0U) {
        return 0U;
    }

    uint16_t read = 0U;
    uint16_t write = 1U;
    uint16_t code_idx = 0U;
    uint8_t code = 1U;

    while (read < src_len) {
        if (src[read] == 0U) {
            dst[code_idx] = code;
            code = 1U;
            code_idx = write;
            if (write >= dst_cap) {
                return 0U;
            }
            write++;
            read++;
            continue;
        }

        if (write >= dst_cap) {
            return 0U;
        }

        dst[write++] = src[read++];
        code++;

        if (code == 0xFFU) {
            dst[code_idx] = code;
            code = 1U;
            code_idx = write;
            if (write >= dst_cap) {
                return 0U;
            }
            write++;
        }
    }

    dst[code_idx] = code;
    *dst_len = write;
    return 1U;
}

static host_status_t parse_inner_frame(const uint8_t *inner, uint16_t inner_len, host_frame_t *frame) {
    if (inner_len < (HOST_HEADER_LEN + HOST_CRC_LEN)) {
        return HOST_STATUS_ERR_PROTO;
    }

    if (inner_len > HOST_INNER_MAX_LEN) {
        return HOST_STATUS_ERR_TOO_LARGE;
    }

    const uint16_t payload_len = (uint16_t)inner[5] | ((uint16_t)inner[6] << 8);
    if (payload_len > HOST_PAYLOAD_MAX_LEN) {
        return HOST_STATUS_ERR_TOO_LARGE;
    }

    const uint16_t expected = (uint16_t)(HOST_HEADER_LEN + payload_len + HOST_CRC_LEN);
    if (inner_len != expected) {
        return HOST_STATUS_ERR_PROTO;
    }

    const uint16_t got_crc = (uint16_t)inner[inner_len - 2U] |
                             ((uint16_t)inner[inner_len - 1U] << 8);
    const uint16_t calc_crc = crc16_ccitt(inner, (uint16_t)(inner_len - 2U));
    if (got_crc != calc_crc) {
        return HOST_STATUS_ERR_CRC;
    }

    frame->ver = inner[0];
    frame->type = inner[1];
    frame->flags = inner[2];
    frame->seq = (uint16_t)inner[3] | ((uint16_t)inner[4] << 8);
    frame->payload_len = payload_len;

    if (payload_len > 0U) {
        memcpy(frame->payload, &inner[HOST_HEADER_LEN], payload_len);
    }

    return HOST_STATUS_OK;
}

static void process_encoded_frame(void) {
    uint8_t decoded[HOST_INNER_MAX_LEN];
    uint16_t decoded_len = 0U;

    host_frame_t frame;
    if (!cobs_decode(s_cobs_encoded, s_cobs_encoded_len, decoded, (uint16_t)sizeof(decoded), &decoded_len)) {
        s_stats_errors++;
        return;
    }

    if (parse_inner_frame(decoded, decoded_len, &frame) != HOST_STATUS_OK) {
        s_stats_errors++;
        return;
    }

    (void)queue_push(&frame);
}

static void ingest_rx_byte(uint8_t byte) {
    if (s_cobs_overflow != 0U) {
        if (byte == 0U) {
            s_cobs_overflow = 0U;
            s_cobs_encoded_len = 0U;
        }
        return;
    }

    if (byte == 0U) {
        if (s_cobs_encoded_len > 0U) {
            process_encoded_frame();
        }
        s_cobs_encoded_len = 0U;
        return;
    }

    if (s_cobs_encoded_len >= HOST_COBS_ENCODED_MAX) {
        s_cobs_overflow = 1U;
        s_cobs_encoded_len = 0U;
        s_stats_errors++;
        return;
    }

    s_cobs_encoded[s_cobs_encoded_len++] = byte;
}

static void service_dma_rx(void) {
    if (s_service_in_progress != 0U) {
        return;
    }

    s_service_in_progress = 1U;

    uint16_t write_idx = (uint16_t)(HOST_DMA_RX_BUF_SIZE - DMA1_CNDTR(HOST_DMA_RX_CH));
    while (s_dma_last_idx != write_idx) {
        ingest_rx_byte(s_dma_rx_buf[s_dma_last_idx]);
        s_dma_last_idx++;
        if (s_dma_last_idx >= HOST_DMA_RX_BUF_SIZE) {
            s_dma_last_idx = 0U;
        }
    }

    s_service_in_progress = 0U;
}

static void usart2_write_byte(uint8_t byte) {
    while ((USART2_ISR & USART_ISR_TXE) == 0U) {
    }

    USART2_TDR = byte;
}

static void usart2_write_bytes(const uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0U; i < len; ++i) {
        usart2_write_byte(buf[i]);
    }

    while ((USART2_ISR & USART_ISR_TC) == 0U) {
    }
}

static void send_inner_frame(const uint8_t *inner, uint16_t inner_len) {
    uint8_t encoded[HOST_COBS_ENCODED_MAX];
    uint16_t encoded_len = 0U;

    if (!cobs_encode(inner, inner_len, encoded, (uint16_t)sizeof(encoded), &encoded_len)) {
        s_stats_errors++;
        return;
    }

    usart2_write_byte(0U);
    usart2_write_bytes(encoded, encoded_len);
    usart2_write_byte(0U);
}

void host_uart_init(uint32_t baud) {
    RCC_AHBENR |= RCC_AHBENR_DMA1EN;
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;

    usart2_gpio_init();

    USART2_CR1 = 0U;
    USART2_CR2 = 0U;
    USART2_CR3 = 0U;
    USART2_BRR = usart_brr_from_baud(baud);
    USART2_ICR = USART_ICR_PECF |
                 USART_ICR_FECF |
                 USART_ICR_NCF |
                 USART_ICR_ORECF |
                 USART_ICR_IDLECF;

    s_q_head = 0U;
    s_q_tail = 0U;
    s_q_count = 0U;
    s_cobs_encoded_len = 0U;
    s_cobs_overflow = 0U;
    s_service_in_progress = 0U;
    s_stats_dropped = 0U;
    s_stats_errors = 0U;

    dma_rx_start();

    USART2_CR3 = USART_CR3_DMAR;
    USART2_CR1 = USART_CR1_UE |
                 USART_CR1_RE |
                 USART_CR1_TE |
                 USART_CR1_IDLEIE;

    platform_irq_enable(DMA1_Channel4_5_6_7_IRQn, 2U);
    platform_irq_enable(USART2_IRQn, 1U);
}

void host_uart_poll_dma(void) {
    service_dma_rx();
}

bool host_uart_pop_frame(host_frame_t *out_frame) {
    uint32_t irq_state = cpu_irq_save();

    if (s_q_count == 0U) {
        cpu_irq_restore(irq_state);
        return false;
    }

    *out_frame = s_frame_queue[s_q_tail];
    s_q_tail = (uint8_t)((s_q_tail + 1U) % HOST_FRAME_QUEUE_DEPTH);
    s_q_count--;

    cpu_irq_restore(irq_state);
    return true;
}

void host_uart_send_urc(uint8_t type,
                        uint16_t seq,
                        uint8_t flags,
                        const uint8_t *payload,
                        uint16_t payload_len) {
    uint8_t inner[HOST_INNER_MAX_LEN];
    uint16_t idx;
    uint16_t crc;

    if (payload_len > HOST_PAYLOAD_MAX_LEN) {
        s_stats_errors++;
        return;
    }

    if (payload_len > 0U && payload == NULL) {
        s_stats_errors++;
        return;
    }

    inner[0] = HOST_PROTOCOL_VER;
    inner[1] = type;
    inner[2] = flags;
    inner[3] = (uint8_t)(seq & 0xFFU);
    inner[4] = (uint8_t)((seq >> 8) & 0xFFU);
    inner[5] = (uint8_t)(payload_len & 0xFFU);
    inner[6] = (uint8_t)((payload_len >> 8) & 0xFFU);

    idx = HOST_HEADER_LEN;
    if (payload_len > 0U && payload != NULL) {
        memcpy(&inner[idx], payload, payload_len);
        idx = (uint16_t)(idx + payload_len);
    }

    crc = crc16_ccitt(inner, idx);
    inner[idx++] = (uint8_t)(crc & 0xFFU);
    inner[idx++] = (uint8_t)((crc >> 8) & 0xFFU);

    send_inner_frame(inner, idx);
}

void host_uart_send_err_proto(uint16_t seq, uint8_t offending_type, uint8_t offending_ver) {
    uint8_t payload[2];
    payload[0] = offending_type;
    payload[1] = offending_ver;
    host_uart_send_urc(HOST_TYPE_ERR_PROTO, seq, 0U, payload, (uint16_t)sizeof(payload));
}

uint32_t host_uart_stats_dropped(void) {
    return s_stats_dropped;
}

uint32_t host_uart_stats_errors(void) {
    return s_stats_errors;
}

void USART2_IRQHandler(void) {
    const uint32_t isr = USART2_ISR;

    if ((isr & USART_ISR_IDLE) != 0U) {
        USART2_ICR = USART_ICR_IDLECF;
        service_dma_rx();
    }

    if ((isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) != 0U) {
        USART2_ICR = USART_ICR_PECF |
                     USART_ICR_FECF |
                     USART_ICR_NCF |
                     USART_ICR_ORECF;
        (void)USART2_RDR;
        s_stats_errors++;
    }
}

void DMA1_Channel4_5_6_7_IRQHandler(void) {
    const uint32_t isr = DMA1_ISR;

    if ((isr & DMA_ISR_TEIF(HOST_DMA_RX_CH)) != 0U) {
        DMA1_IFCR = DMA_IFCR_CTEIF(HOST_DMA_RX_CH) | DMA_IFCR_CGIF(HOST_DMA_RX_CH);
        dma_rx_start();
        s_stats_errors++;
    }
}
