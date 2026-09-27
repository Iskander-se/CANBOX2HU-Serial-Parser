#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"

// Marker for protocol header of CANBOX (Raise / SimpleSoft / Hiworld)
#define CANBOX_HEADER      0x2E

// Setup for UART pin configuration
#define CANBOX_UART_PORT   UART_NUM_1
#define CANBOX_TX_PIN      3
#define CANBOX_RX_PIN      2

#define HU_UART_PORT       UART_NUM_2
#define HU_TX_PIN          4
#define HU_RX_PIN          5

#define BAUDRATE           115200
#define BUF_SIZE           2048

// Ring buffers for byte-level data
static RingbufHandle_t ringbuf_can_to_hu = NULL;
static RingbufHandle_t ringbuf_hu_to_can = NULL;

// Vehicle state variables
static uint8_t g_current_speed_kmh = 0;
static bool g_speed_over_40 = false;

// Template for silence for PDC audio status (0x20)
static const uint8_t PDC_CLEAR_AUDIO[] = { 0x2E, 0x20, 0x02, 0x00, 0x00, 0xDB };

/**
 * @brief Initialization of the clearing console output on USB CDC
 */
static void init_usb_serial(void) {
    usb_serial_jtag_driver_config_t jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    jtag_config.rx_buffer_size = BUF_SIZE;
    jtag_config.tx_buffer_size = BUF_SIZE;
    usb_serial_jtag_driver_install(&jtag_config);
}

/**
 * @brief Initialization of the hardware UART for CANBOX and HU     
 */
static void init_uarts(void) {
    const uart_config_t uart_config = {
        .baud_rate  = BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_param_config(CANBOX_UART_PORT, &uart_config);
    uart_set_pin(CANBOX_UART_PORT, CANBOX_TX_PIN, CANBOX_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(CANBOX_UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);

    uart_param_config(HU_UART_PORT, &uart_config);
    uart_set_pin(HU_UART_PORT, HU_TX_PIN, HU_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(HU_UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
}

/**
 * @brief Parsing vehicle speed from CANBOX status packets
 */
static void parse_vehicle_speed(const uint8_t *pkt, size_t len) {
    uint8_t cmd = pkt[1];

    // Packet 0x14 (Vehicle status)
    if (cmd == 0x14 && len >= 5) {
        g_current_speed_kmh = pkt[4];
    }
    // Packet 0x24 (Speed / Handbrake)
    else if (cmd == 0x24 && len >= 5) {
        g_current_speed_kmh = pkt[3];
    }

    bool prev_state = g_speed_over_40;
    g_speed_over_40 = (g_current_speed_kmh > 40);

    // Upon initial exceeding the 40 km/h threshold, send a cancellation packet
    if (g_speed_over_40 && !prev_state) {
        uart_write_bytes(HU_UART_PORT, (const char *)PDC_CLEAR_AUDIO, sizeof(PDC_CLEAR_AUDIO));
    }
}

/**
 * @brief Dynamic clearing of PLA 3.0 / PDC visualization packet
 */
static void send_dynamic_pdc_clear(uint8_t *pkt, size_t len) {
    if (len < 4) return;

    uint8_t cmd = pkt[1];
    uint8_t data_len = pkt[2];

    if (cmd == 0x26) {
        // Clear all distance bytes according to the length of the incoming packet
        memset(&pkt[3], 0x00, data_len);

        // Recalculate CS (inversion of byte sum)
        uint32_t sum = 0;
        for (size_t i = 1; i < len - 1; i++) {
            sum += pkt[i];
        }
        pkt[len - 1] = (uint8_t)(~sum);

        uart_write_bytes(HU_UART_PORT, (const char *)pkt, len);
    } 
    else if (cmd == 0x20) {
        uart_write_bytes(HU_UART_PORT, (const char *)PDC_CLEAR_AUDIO, sizeof(PDC_CLEAR_AUDIO));
    }
}

/**
 * @brief Stream parser and filter for CANBOX -> HU packets
 */
static void filter_and_forward_can_to_hu(const uint8_t *in_buf, size_t in_len) {
    static uint8_t pkt_buf[256];
    static size_t pkt_idx = 0;
    static size_t expected_len = 0;

    for (size_t i = 0; i < in_len; i++) {
        uint8_t b = in_buf[i];

        if (pkt_idx == 0) {
            if (b == CANBOX_HEADER) {
                pkt_buf[pkt_idx++] = b;
            }
            continue;
        }

        pkt_buf[pkt_idx++] = b;

        if (pkt_idx == 3) {
            uint8_t data_len = pkt_buf[2];
            expected_len = 3 + data_len + 1;

            if (expected_len > sizeof(pkt_buf)) {
                pkt_idx = 0;
                continue;
            }
        }

        // Packet fully received in the buffer
        if (pkt_idx > 3 && pkt_idx == expected_len) {
            uint8_t cmd = pkt_buf[1];

            // Check the current speed
            parse_vehicle_speed(pkt_buf, pkt_idx);

            // Обработка PDC/PLA3.0 (0x26 и 0x20)
            if (cmd == 0x26 || cmd == 0x20) {
                if (g_speed_over_40) {
                    // Upon exceeding 40 km/h, intercept and replace with "all clear"
                    send_dynamic_pdc_clear(pkt_buf, pkt_idx);
                } else {
                    // For speeds <= 40 km/h, forward the original packet
                    uart_write_bytes(HU_UART_PORT, (const char *)pkt_buf, pkt_idx);
                }
            } else {
                // Other packets (steering, climate, doors) are passed through without changes
                uart_write_bytes(HU_UART_PORT, (const char *)pkt_buf, pkt_idx);
            }

            pkt_idx = 0;
            expected_len = 0;
        }
    }
}

/**
 * @brief Task for forwarding and filtering CANBOX -> Head Unit packets
 */
void canbox_to_hu_task(void *pvParameters) {
    uint8_t rx_tmp_buf[256];

    while (1) {
        int len = uart_read_bytes(CANBOX_UART_PORT, rx_tmp_buf, sizeof(rx_tmp_buf), pdMS_TO_TICKS(10));
        
        if (len > 0) {
            // Дублируем сырой поток в USB CDC для отладки на ПК при необходимости
            usb_serial_jtag_write_bytes(rx_tmp_buf, len, 0);

            xRingbufferSend(ringbuf_can_to_hu, rx_tmp_buf, len, pdMS_TO_TICKS(20));
        }

        size_t size = 0;
        uint8_t *data = (uint8_t *)xRingbufferReceiveUpTo(ringbuf_can_to_hu, &size, pdMS_TO_TICKS(5), BUF_SIZE);
        
        if (data != NULL) {
            filter_and_forward_can_to_hu(data, size);
            vRingbufferReturnItem(ringbuf_can_to_hu, (void *)data);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief Task for direct forwarding of Head Unit -> CANBOX packets (without changes)
 */
void hu_to_canbox_task(void *pvParameters) {
    uint8_t rx_tmp_buf[256];

    while (1) {
        int len = uart_read_bytes(HU_UART_PORT, rx_tmp_buf, sizeof(rx_tmp_buf), pdMS_TO_TICKS(10));
        
        if (len > 0) {
            xRingbufferSend(ringbuf_hu_to_can, rx_tmp_buf, len, pdMS_TO_TICKS(20));
        }

        size_t size = 0;
        uint8_t *data = (uint8_t *)xRingbufferReceiveUpTo(ringbuf_hu_to_can, &size, pdMS_TO_TICKS(5), BUF_SIZE);
        
        if (data != NULL) {
            uart_write_bytes(CANBOX_UART_PORT, (const char *)data, size);
            vRingbufferReturnItem(ringbuf_hu_to_can, (void *)data);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void) {
    init_usb_serial();

    ringbuf_can_to_hu = xRingbufferCreate(BUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    ringbuf_hu_to_can = xRingbufferCreate(BUF_SIZE, RINGBUF_TYPE_BYTEBUF);

    init_uarts();

    // Binding tasks to d ESP32-S3
    xTaskCreatePinnedToCore(canbox_to_hu_task, "can_to_hu", 3072, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(hu_to_canbox_task, "hu_to_can", 3072, NULL, 10, NULL, 1);
}