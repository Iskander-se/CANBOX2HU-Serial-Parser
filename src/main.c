#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "board_hw.h"
#include "canbox_proto.h"

static RingbufHandle_t ringbuf_can_to_hu = NULL;
static RingbufHandle_t ringbuf_hu_to_can = NULL;

void canbox_to_hu_task(void *pvParameters) {
    uint8_t rx_tmp_buf[256];

    while (1) {
        int len = uart_read_bytes(CANBOX_UART_PORT, rx_tmp_buf, sizeof(rx_tmp_buf), pdMS_TO_TICKS(10));
        if (len > 0) {
            xRingbufferSend(ringbuf_can_to_hu, rx_tmp_buf, len, pdMS_TO_TICKS(20));
        }

        size_t size = 0;
        uint8_t *data = (uint8_t *)xRingbufferReceiveUpTo(ringbuf_can_to_hu, &size, pdMS_TO_TICKS(5), BUF_SIZE);
        if (data != NULL) {
            parser_and_forward_can_to_hu(data, size);
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
            parse_and_forward_hu_to_can(data, size);
            vRingbufferReturnItem(ringbuf_hu_to_can, (void *)data);
        }

        process_road_sign_injection(ringbuf_hu_to_can);

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