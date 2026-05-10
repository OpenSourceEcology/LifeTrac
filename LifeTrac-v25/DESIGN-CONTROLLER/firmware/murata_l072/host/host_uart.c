#include "host_uart.h"

#include "host_cmd.h"
#include "platform.h"
#include "stm32l072_regs.h"

#include <stddef.h>
#include <string.h>

#define HOST_DMA_RX_CH               6U
#define HOST_DMA_RX_BUF_SIZE         512U
#define HOST_FRAME_QUEUE_DEPTH       4U
#define HOST_COBS_ENCODED_MAX        (HOST_COBS_MAX_LEN - 2U)
#define LPUART1_CLOCK_HZ             16000000UL
#define USART_CR1_OVER8              (1UL << 15U)

/* Stage-1 ingress diagnostic: echo raw RX bytes back out on both host TX lanes. */
#ifndef HOST_UART_RX_ECHO_DIAG
#define HOST_UART_RX_ECHO_DIAG       1U
#endif

_Static_assert((HOST_DMA_RX_BUF_SIZE % 2U) == 0U,
               "HOST_DMA_RX_BUF_SIZE must be even for HT/TC servicing");

static volatile uint8_t s_dma_rx_buf[HOST_DMA_RX_BUF_SIZE];
static uint16_t s_dma_last_idx;

static uint8_t s_cobs_encoded[HOST_COBS_ENCODED_MAX];
static uint16_t s_cobs_encoded_len;
static uint8_t s_cobs_overflow;
static volatile uint8_t s_service_in_progress;

#if HOST_AT_SHELL_ENABLE
_Static_assert(HOST_AT_LINE_MAX_LEN >= 8U,
               "HOST_AT_LINE_MAX_LEN must fit the smallest AT command");

static char s_at_line[HOST_AT_LINE_MAX_LEN];
static uint16_t s_at_line_len;
static uint8_t s_at_candidate;
static uint8_t s_at_ignore_lf;
#endif

static host_frame_t s_frame_queue[HOST_FRAME_QUEUE_DEPTH];
static volatile uint8_t s_q_head;
static volatile uint8_t s_q_tail;
static volatile uint8_t s_q_count;

static uint32_t s_stats_dropped;
static uint32_t s_stats_errors;
static uint32_t s_stats_queue_full;
static uint32_t s_stats_irq_idle;
static uint32_t s_stats_irq_ht;
static uint32_t s_stats_irq_tc;
static uint32_t s_stats_irq_te;
static uint32_t s_stats_rx_bytes;
static uint32_t s_stats_rx_lpuart_bytes;
static uint32_t s_stats_rx_usart1_bytes;
static uint32_t s_stats_parse_ok;
static uint32_t s_stats_parse_err;
static uint32_t s_stats_uart_err_lpuart;
static uint32_t s_stats_uart_err_usart1;
static volatile uint32_t s_dma_te_events;
static volatile uint8_t s_rx_seen_flags;
static volatile uint8_t s_diag_marks;
static uint8_t s_trace_first_rx_lpuart;
static uint8_t s_trace_first_rx_usart1;
static uint8_t s_trace_first_err_lpuart;
static uint8_t s_trace_first_err_usart1;

#define HOST_RX_SEEN_FLAG_LPUART             0x01U
#define HOST_RX_SEEN_FLAG_USART1             0x02U

static void host_diag_echo_rx_byte(uint8_t byte) {
#if HOST_UART_RX_ECHO_DIAG
    /* Non-blocking echo to avoid stalling RX IRQ service. */
    if ((LPUART1_ISR & USART_ISR_TXE) != 0U) {
        LPUART1_TDR = byte;
    }

    if ((USART1_ISR & USART_ISR_TXE) != 0U) {
        USART1_TDR = byte;
    }
#else
    (void)byte;
#endif
}

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

    return ((256UL * LPUART1_CLOCK_HZ) + (baud / 2UL)) / baud;
}

static uint32_t usart1_brr_from_baud(uint32_t baud) {
    if (baud == 0UL) {
        return 1UL;
    }

    /* USART1 uses OVER8 for tighter 921600 baud error at 16 MHz. */
    const uint32_t div = ((2UL * LPUART1_CLOCK_HZ) + (baud / 2UL)) / baud;
    return (div & 0xFFF0UL) | ((div & 0x000FUL) >> 1U);
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

    gpio_set_alt(GPIOA_BASE, 2U, 6U);
    gpio_set_alt(GPIOA_BASE, 3U, 6U);

    /* Keep RX high when host is disconnected. */
    uint32_t pupd = GPIO_PUPDR(GPIOA_BASE);
    pupd &= ~(3UL << (3U * 2U));
    pupd |= (1UL << (3U * 2U));
    GPIO_PUPDR(GPIOA_BASE) = pupd;

    /* Mirror host transport on USART1 as a routing fallback. */
    gpio_set_alt(GPIOA_BASE, 9U, 4U);
    gpio_set_alt(GPIOA_BASE, 10U, 4U);

    pupd = GPIO_PUPDR(GPIOA_BASE);
    pupd &= ~(3UL << (10U * 2U));
    pupd |= (1UL << (10U * 2U));
    GPIO_PUPDR(GPIOA_BASE) = pupd;
}

static uint8_t queue_push(const host_frame_t *frame) {
    uint32_t irq_state = cpu_irq_save();

    if (s_q_count >= HOST_FRAME_QUEUE_DEPTH) {
        s_stats_queue_full++;
        s_stats_dropped++;
        cpu_irq_restore(irq_state);
        return 0U;
    }

    s_frame_queue[s_q_head] = *frame;
    s_q_head = (uint8_t)((s_q_head + 1U) % HOST_FRAME_QUEUE_DEPTH);
    s_q_count++;

    cpu_irq_restore(irq_state);

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
        platform_diag_trace("H:COBS_ERR\r\n");
        s_stats_errors++;
        s_stats_parse_err++;
        s_diag_marks |= HOST_DIAG_MARK_FRAME_PARSE_ERR;
        return;
    }

    if (parse_inner_frame(decoded, decoded_len, &frame) != HOST_STATUS_OK) {
        platform_diag_trace("H:PARSE_ERR\r\n");
        s_stats_errors++;
        s_stats_parse_err++;
        s_diag_marks |= HOST_DIAG_MARK_FRAME_PARSE_ERR;
        return;
    }

    if (frame.type == HOST_TYPE_VER_REQ) {
        platform_diag_trace("H:VER_REQ_RX\r\n");
        s_diag_marks |= HOST_DIAG_MARK_VER_REQ_PARSED;
    }

    platform_diag_trace("H:FRAME_OK\r\n");
    s_stats_parse_ok++;

    if (!queue_push(&frame)) {
        platform_diag_trace("H:Q_FULL\r\n");
    }
}

static void ingest_binary_byte(uint8_t byte) {
    if (s_cobs_overflow != 0U) {
        if (byte == 0U) {
            s_cobs_overflow = 0U;
            s_cobs_encoded_len = 0U;
        }
        return;
    }

    if (byte == 0U) {
        if (s_cobs_encoded_len > 0U) {
            platform_diag_trace("H:PROC_FRAME\r\n");
            process_encoded_frame();
        }
        s_cobs_encoded_len = 0U;
        return;
    }

    if (s_cobs_encoded_len >= HOST_COBS_ENCODED_MAX) {
        platform_diag_trace("H:COBS_OVF\r\n");
        s_cobs_overflow = 1U;
        s_cobs_encoded_len = 0U;
        s_stats_errors++;
        return;
    }

    s_cobs_encoded[s_cobs_encoded_len++] = byte;
}

#if HOST_AT_SHELL_ENABLE
static bool at_is_printable(uint8_t byte) {
    return byte >= 0x20U && byte <= 0x7EU;
}

static bool at_is_t(uint8_t byte) {
    return byte == (uint8_t)'T' || byte == (uint8_t)'t';
}

static void at_reset_candidate(void) {
    s_at_line_len = 0U;
    s_at_candidate = 0U;
}

static void at_replay_candidate_to_binary(void) {
    for (uint16_t i = 0U; i < s_at_line_len; ++i) {
        ingest_binary_byte((uint8_t)s_at_line[i]);
    }

    at_reset_candidate();
}

static void at_finish_line(uint8_t terminator) {
    s_at_line[s_at_line_len] = '\0';
    platform_diag_trace("H:AT_DISPATCH\r\n");
    host_cmd_dispatch_at_line(s_at_line, s_at_line_len);
    at_reset_candidate();
    s_at_ignore_lf = (terminator == (uint8_t)'\r') ? 1U : 0U;
}
#endif

static void ingest_rx_byte(uint8_t byte) {
#if HOST_AT_SHELL_ENABLE
    if (s_at_ignore_lf != 0U) {
        s_at_ignore_lf = 0U;
        if (byte == (uint8_t)'\n') {
            return;
        }
    }

    if (s_at_candidate != 0U) {
        if (byte == (uint8_t)'\r' || byte == (uint8_t)'\n') {
            at_finish_line(byte);
            return;
        }

        if (byte == 0U) {
            at_replay_candidate_to_binary();
            ingest_binary_byte(byte);
            return;
        }

        if ((s_at_line_len == 1U && !at_is_t(byte)) ||
            !at_is_printable(byte) ||
            s_at_line_len >= (HOST_AT_LINE_MAX_LEN - 1U)) {
            at_replay_candidate_to_binary();
            ingest_binary_byte(byte);
            return;
        }

        s_at_line[s_at_line_len++] = (char)byte;
        return;
    }

    if (s_cobs_encoded_len == 0U &&
        s_cobs_overflow == 0U &&
        (byte == (uint8_t)'A' || byte == (uint8_t)'a')) {
        s_at_line[0] = (char)byte;
        s_at_line_len = 1U;
        s_at_candidate = 1U;
        return;
    }
#endif

    ingest_binary_byte(byte);
}

static uint16_t dma_rx_write_idx_now(void) {
    return (uint16_t)(HOST_DMA_RX_BUF_SIZE - DMA1_CNDTR(HOST_DMA_RX_CH));
}

static void service_dma_rx_to(uint16_t write_idx) {
    uint32_t irq_state = cpu_irq_save();

    if (s_service_in_progress != 0U) {
        cpu_irq_restore(irq_state);
        return;
    }

    s_service_in_progress = 1U;
    cpu_irq_restore(irq_state);

    while (s_dma_last_idx != write_idx) {
        ingest_rx_byte(s_dma_rx_buf[s_dma_last_idx]);
            s_stats_rx_bytes++;
        s_dma_last_idx++;
        if (s_dma_last_idx >= HOST_DMA_RX_BUF_SIZE) {
            s_dma_last_idx = 0U;
        }
    }

    irq_state = cpu_irq_save();
    s_service_in_progress = 0U;
    cpu_irq_restore(irq_state);
}

static void service_dma_rx_now(void) {
    service_dma_rx_to(dma_rx_write_idx_now());
}

static void usart2_write_byte(uint8_t byte) {
    while ((LPUART1_ISR & USART_ISR_TXE) == 0U) {
    }

    LPUART1_TDR = byte;

    while ((USART1_ISR & USART_ISR_TXE) == 0U) {
    }

    USART1_TDR = byte;
}

static void usart2_write_bytes(const uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0U; i < len; ++i) {
        usart2_write_byte(buf[i]);
    }

    while ((LPUART1_ISR & USART_ISR_TC) == 0U) {
    }

    while ((USART1_ISR & USART_ISR_TC) == 0U) {
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
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC_APB1ENR |= RCC_APB1ENR_LPUART1EN;
    RCC_CCIPR = (RCC_CCIPR & ~(3UL << 10U)) | (2UL << 10U);

    usart2_gpio_init();

    LPUART1_CR1 = 0U;
    LPUART1_CR2 = 0U;
    LPUART1_CR3 = 0U;
    LPUART1_BRR = usart_brr_from_baud(baud);
    LPUART1_ICR = USART_ICR_PECF |
                  USART_ICR_FECF |
                  USART_ICR_NCF |
                  USART_ICR_ORECF |
                  USART_ICR_IDLECF;

    USART1_CR1 = 0U;
    USART1_CR2 = 0U;
    USART1_CR3 = 0U;
    USART1_BRR = usart1_brr_from_baud(baud);
    USART1_ICR = USART_ICR_PECF |
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
#if HOST_AT_SHELL_ENABLE
    s_at_line_len = 0U;
    s_at_candidate = 0U;
    s_at_ignore_lf = 0U;
#endif
    host_uart_stats_reset();

    LPUART1_CR1 = USART_CR1_UE |
                  USART_CR1_RE |
                  USART_CR1_TE |
                  USART_CR1_RXNEIE |
                  USART_CR1_IDLEIE;

    USART1_CR1 = USART_CR1_UE |
                 USART_CR1_OVER8 |
                 USART_CR1_RE |
                 USART_CR1_TE |
                 USART_CR1_RXNEIE |
                 USART_CR1_IDLEIE;
    platform_irq_enable(RNG_LPUART1_IRQn, 1U);
    platform_irq_enable(USART1_IRQn, 1U);
}

void host_uart_poll_dma(void) {
    /* RX is interrupt-driven on the UART lane that is physically routed. */
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

void host_uart_send_ascii(const char *text) {
    if (text == NULL) {
        s_stats_errors++;
        return;
    }

    usart2_write_bytes((const uint8_t *)text, (uint16_t)strlen(text));
}

void host_uart_send_err_proto(uint16_t seq,
                              uint8_t offending_type,
                              uint8_t offending_ver,
                              uint8_t err_code,
                              uint16_t detail) {
    uint8_t payload[5];
    payload[0] = offending_type;
    payload[1] = offending_ver;
    payload[2] = err_code;
    payload[3] = (uint8_t)(detail & 0xFFU);
    payload[4] = (uint8_t)((detail >> 8) & 0xFFU);

    host_uart_send_urc(HOST_TYPE_ERR_PROTO_URC, seq, 0U, payload, (uint16_t)sizeof(payload));
}

void host_uart_stats_reset(void) {
    uint32_t irq_state = cpu_irq_save();

    s_stats_dropped = 0U;
    s_stats_errors = 0U;
    s_stats_queue_full = 0U;
    s_stats_irq_idle = 0U;
    s_stats_irq_ht = 0U;
    s_stats_irq_tc = 0U;
    s_stats_irq_te = 0U;
    s_stats_rx_bytes = 0U;
    s_stats_rx_lpuart_bytes = 0U;
    s_stats_rx_usart1_bytes = 0U;
    s_stats_parse_ok = 0U;
    s_stats_parse_err = 0U;
    s_stats_uart_err_lpuart = 0U;
    s_stats_uart_err_usart1 = 0U;
    s_dma_te_events = 0U;
    s_rx_seen_flags = 0U;
    s_diag_marks = 0U;
    s_trace_first_rx_lpuart = 0U;
    s_trace_first_rx_usart1 = 0U;
    s_trace_first_err_lpuart = 0U;
    s_trace_first_err_usart1 = 0U;

    cpu_irq_restore(irq_state);
}

uint32_t host_uart_stats_dropped(void) {
    return s_stats_dropped;
}

uint32_t host_uart_stats_errors(void) {
    return s_stats_errors;
}

uint32_t host_uart_stats_queue_full(void) {
    return s_stats_queue_full;
}

uint32_t host_uart_stats_irq_idle(void) {
    return s_stats_irq_idle;
}

uint32_t host_uart_stats_irq_ht(void) {
    return s_stats_irq_ht;
}

uint32_t host_uart_stats_irq_tc(void) {
    return s_stats_irq_tc;
}

uint32_t host_uart_stats_irq_te(void) {
    return s_stats_irq_te;
}

uint32_t host_uart_stats_rx_bytes(void) {
    return s_stats_rx_bytes;
}

uint32_t host_uart_stats_rx_lpuart_bytes(void) {
    return s_stats_rx_lpuart_bytes;
}

uint32_t host_uart_stats_rx_usart1_bytes(void) {
    return s_stats_rx_usart1_bytes;
}

uint32_t host_uart_stats_parse_ok(void) {
    return s_stats_parse_ok;
}

uint32_t host_uart_stats_parse_err(void) {
    return s_stats_parse_err;
}

uint32_t host_uart_stats_uart_err_lpuart(void) {
    return s_stats_uart_err_lpuart;
}

uint32_t host_uart_stats_uart_err_usart1(void) {
    return s_stats_uart_err_usart1;
}

uint32_t host_uart_take_dma_te_events(void) {
    uint32_t irq_state = cpu_irq_save();
    uint32_t events = s_dma_te_events;
    s_dma_te_events = 0U;
    cpu_irq_restore(irq_state);
    return events;
}

uint8_t host_uart_take_rx_seen_flags(void) {
    uint32_t irq_state = cpu_irq_save();
    uint8_t flags = s_rx_seen_flags;
    s_rx_seen_flags = 0U;
    cpu_irq_restore(irq_state);
    return flags;
}

void host_uart_note_diag_mark(uint8_t mark) {
    if (mark == 0U) {
        return;
    }

    uint32_t irq_state = cpu_irq_save();
    s_diag_marks = (uint8_t)(s_diag_marks | mark);
    cpu_irq_restore(irq_state);
}

uint8_t host_uart_take_diag_marks(void) {
    uint32_t irq_state = cpu_irq_save();
    uint8_t marks = s_diag_marks;
    s_diag_marks = 0U;
    cpu_irq_restore(irq_state);
    return marks;
}

void RNG_LPUART1_IRQHandler(void) {
    const uint32_t isr = LPUART1_ISR;

    if ((isr & USART_ISR_IDLE) != 0U) {
        LPUART1_ICR = USART_ICR_IDLECF;
        s_stats_irq_idle++;
    }

    while ((LPUART1_ISR & USART_ISR_RXNE) != 0U) {
        const uint8_t rx = (uint8_t)LPUART1_RDR;
        if (s_trace_first_rx_lpuart == 0U) {
            s_trace_first_rx_lpuart = 1U;
            platform_diag_trace("H:RX_LPUART1\r\n");
        }
        s_stats_rx_bytes++;
        s_stats_rx_lpuart_bytes++;
        s_rx_seen_flags |= HOST_RX_SEEN_FLAG_LPUART;
        host_diag_echo_rx_byte(rx);
        ingest_rx_byte(rx);
    }

    if ((isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) != 0U) {
        if (s_trace_first_err_lpuart == 0U) {
            s_trace_first_err_lpuart = 1U;
            platform_diag_trace("H:ERR_LPUART1\r\n");
        }
        LPUART1_ICR = USART_ICR_PECF |
                      USART_ICR_FECF |
                      USART_ICR_NCF |
                      USART_ICR_ORECF;
        (void)LPUART1_RDR;
        s_stats_errors++;
        s_stats_uart_err_lpuart++;
    }
}

void USART1_IRQHandler(void) {
    const uint32_t isr = USART1_ISR;

    if ((isr & USART_ISR_IDLE) != 0U) {
        USART1_ICR = USART_ICR_IDLECF;
        s_stats_irq_idle++;
    }

    while ((USART1_ISR & USART_ISR_RXNE) != 0U) {
        const uint8_t rx = (uint8_t)USART1_RDR;
        if (s_trace_first_rx_usart1 == 0U) {
            s_trace_first_rx_usart1 = 1U;
            platform_diag_trace("H:RX_USART1\r\n");
        }
        s_stats_rx_bytes++;
        s_stats_rx_usart1_bytes++;
        s_rx_seen_flags |= HOST_RX_SEEN_FLAG_USART1;
        host_diag_echo_rx_byte(rx);
        ingest_rx_byte(rx);
    }

    if ((isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) != 0U) {
        if (s_trace_first_err_usart1 == 0U) {
            s_trace_first_err_usart1 = 1U;
            platform_diag_trace("H:ERR_USART1\r\n");
        }
        USART1_ICR = USART_ICR_PECF |
                     USART_ICR_FECF |
                     USART_ICR_NCF |
                     USART_ICR_ORECF;
        (void)USART1_RDR;
        s_stats_errors++;
        s_stats_uart_err_usart1++;
    }
}
