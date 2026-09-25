#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_dev.h"

// Конфигурация пинов
#define CANBOX_UART_PORT   UART_NUM_1
#define CANBOX_TX_PIN      3
#define CANBOX_RX_PIN      2

#define HU_UART_PORT       UART_NUM_2
#define HU_TX_PIN          4
#define HU_RX_PIN          5

#define BAUDRATE           115200
#define BUF_SIZE           2048

static RingbufHandle_t ringbuf_can_to_hu = NULL;
static RingbufHandle_t ringbuf_hu_to_can = NULL;

static void init_usb_serial(void) {
    // Настройка встроенного USB как чистого бинарного Serial порта без stdio
    usb_serial_jtag_driver_config_t jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    jtag_config.rx_buffer_size = BUF_SIZE;
    jtag_config.tx_buffer_size = BUF_SIZE;
    usb_serial_jtag_driver_install(&jtag_config);
}

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

void canbox_to_hu_task(void *pvParameters) {
    uint8_t rx_tmp_buf[256];

    while (1) {
        int len = uart_read_bytes(CANBOX_UART_PORT, rx_tmp_buf, sizeof(rx_tmp_buf), pdMS_TO_TICKS(10));
        
        if (len > 0) {
            // Можем дублировать сырой поток CANBOX прямо в USB COM-порт на ПК (без текста/логов)
            usb_serial_jtag_write_bytes(rx_tmp_buf, len, 0);

            xRingbufferSend(ringbuf_can_to_hu, rx_tmp_buf, len, pdMS_TO_TICKS(20));
        }

        size_t size = 0;
        uint8_t *data = (uint8_t *)xRingbufferReceiveUpTo(ringbuf_can_to_hu, &size, pdMS_TO_TICKS(5), BUF_SIZE);
        
        if (data != NULL) {
            uart_write_bytes(HU_UART_PORT, (const char *)data, size);
            vRingbufferReturnItem(ringbuf_can_to_hu, (void *)data);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

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

    xTaskCreatePinnedToCore(canbox_to_hu_task, "can_to_hu", 3072, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(hu_to_canbox_task, "hu_to_can", 3072, NULL, 10, NULL, 1);
}