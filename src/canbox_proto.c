#include "canbox_proto.h"
#include "vehicle_state.h"
#include "board_hw.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include <string.h>
#include <stdio.h>

const char* get_packet_type_name(uint8_t cmd) {
    switch (cmd) {
        case 0x14:
        case 0x24: return "SPEED_STATUS";
        case 0x26: return "PDC_PLA3";
        case 0x21: return "STEERING_WHEEL";
        case 0x32: return "ROAD_SIGN";
        case 0x40: return "CLIMATE_CONTROL";
        case 0x28:
        case 0x50: return "SYSTEM_INFO";
        default:   return "OTHER_CMD";
    }
}

static void send_brief_to_usb(const char* dir, uint32_t pkt_num, size_t len, uint8_t cmd) {
    char msg[80];
    int msg_len = snprintf(msg, sizeof(msg), "[%s] #%lu | Len:%d | Type:%s (0x%02X)\r\n",
                          dir, pkt_num, (int)len, get_packet_type_name(cmd), cmd);
    if (msg_len > 0) {
        usb_serial_jtag_write_bytes((const uint8_t*)msg, msg_len, 0);
    }
}

void send_dynamic_pdc_clear(void) {
    size_t payload_len = (g_vehicle.pdc_payload_len > 0) ? g_vehicle.pdc_payload_len : 12;
    size_t total_pkt_len = 3 + payload_len + 1;

    if (total_pkt_len > 64) return;

    uint8_t pkt[64] = { CANBOX_HEADER, 0x26, (uint8_t)payload_len };

    uint32_t sum = 0;
    for (size_t i = 1; i < total_pkt_len - 1; i++) {
        sum += pkt[i];
    }
    pkt[total_pkt_len - 1] = (uint8_t)(~sum);

    uart_write_bytes(HU_UART_PORT, (const char *)pkt, total_pkt_len);
}

void parse_vehicle_speed(const uint8_t *pkt, size_t len) {
    uint8_t cmd = pkt[1];

    if (g_vehicle.pdc_payload_len == 0 && cmd == 0x26 && len >= 3) {
        g_vehicle.pdc_payload_len = pkt[2];
    }

    if (cmd == 0x14 && len >= 5) {
        g_vehicle.current_speed_kmh = pkt[4];
    } else if (cmd == 0x24 && len >= 5) {
        g_vehicle.current_speed_kmh = pkt[3];
    }

    bool prev_state = g_vehicle.speed_over;
    g_vehicle.speed_over = (g_vehicle.current_speed_kmh > 40);

    if (!prev_state && g_vehicle.speed_over) {
       send_dynamic_pdc_clear();
    }
}

bool inject_road_sign_to_queue(RingbufHandle_t ringbuf, uint8_t speed_limit) {
    if (ringbuf == NULL) return false;

    uint8_t pkt[8];
    pkt[0] = CANBOX_HEADER;
    pkt[1] = 0x32;
    pkt[2] = 0x04;
    pkt[3] = (speed_limit > 0) ? 0x01 : 0x00;
    pkt[4] = speed_limit;
    pkt[5] = 0x00;
    pkt[6] = 0x00;

    uint32_t sum = 0;
    for (size_t i = 1; i < 7; i++) {
        sum += pkt[i];
    }
    pkt[7] = (uint8_t)(~sum);

    BaseType_t res = xRingbufferSend(ringbuf, pkt, sizeof(pkt), pdMS_TO_TICKS(10));
    return (res == pdTRUE);
}

void process_road_sign_injection(RingbufHandle_t ringbuf) {
    static TickType_t last_send_time = 0;
    TickType_t now = xTaskGetTickCount();

    uint8_t target_sign = 30;
    if (g_vehicle.current_speed_kmh > 70) {
        target_sign = 100;
    } else if (g_vehicle.current_speed_kmh > 50) {
        target_sign = 70;
    }

    bool sign_changed = (target_sign != g_vehicle.current_speed_limit);
    bool timeout_expired = ((now - last_send_time) >= pdMS_TO_TICKS(2000));

    if (sign_changed || timeout_expired) {
        if (inject_road_sign_to_queue(ringbuf, target_sign)) {
            g_vehicle.current_speed_limit = target_sign;
            last_send_time = now;
        }
    }
}

void clear_pdc_payload(uint8_t *pkt, size_t len) {
    if (len < 4 || pkt[1] != 0x26 || len > 64) return;

    memset(&pkt[3], 0x00, len - 4);

    uint32_t sum = 0;
    for (size_t i = 1; i < len - 1; i++) {
        sum += pkt[i];
    }
    pkt[len - 1] = (uint8_t)(~sum);
}

void parser_and_forward_can_to_hu(const uint8_t *in_buf, size_t in_len) {
    static uint8_t buf[512];
    static size_t buf_len = 0;
    static uint32_t pkt_count = 0;

    if (buf_len + in_len > sizeof(buf)) {
        memmove(buf, buf + (buf_len / 2), buf_len / 2);
        buf_len /= 2;
    }
    memcpy(buf + buf_len, in_buf, in_len);
    buf_len += in_len;

    size_t pos = 0;
    while (pos < buf_len) {
        if (buf[pos] != CANBOX_HEADER) {
            pos++;
            continue;
        }

        if (buf_len - pos < 3) break;

        uint8_t cmd = buf[pos + 1];
        uint8_t data_len = buf[pos + 2];
        size_t total_pkt_len = 3 + data_len + 1;

        if (data_len == 0 || total_pkt_len > 128) {
            pos++;
            continue;
        }

        if (buf_len - pos < total_pkt_len) break;

        pkt_count++;
        send_brief_to_usb("CAN->HU", pkt_count, total_pkt_len, cmd);

        parse_vehicle_speed(&buf[pos], total_pkt_len);

        if (cmd == 0x26 && g_vehicle.speed_over) {
            clear_pdc_payload(&buf[pos], total_pkt_len);
        }

        uart_write_bytes(HU_UART_PORT, (const char *)&buf[pos], total_pkt_len);
        pos += total_pkt_len;
    }

    if (pos > 0) {
        memmove(buf, buf + pos, buf_len - pos);
        buf_len -= pos;
    }
}

void parse_and_forward_hu_to_can(const uint8_t *in_buf, size_t in_len) {
    static uint8_t pkt_buf[256];
    static size_t pkt_idx = 0;
    static size_t expected_len = 0;
    static uint32_t pkt_count = 0;

    for (size_t i = 0; i < in_len; i++) {
        uint8_t b = in_buf[i];

        if (pkt_idx == 0) {
            if (b == CANBOX_HEADER) {
                pkt_buf[pkt_idx++] = b;
            } else {
                uart_write_bytes(CANBOX_UART_PORT, (const char *)&b, 1);
            }
            continue;
        }

        pkt_buf[pkt_idx++] = b;

        if (pkt_idx == 3) {
            uint8_t data_len = pkt_buf[2];
            expected_len = 3 + data_len + 1;

            if (expected_len > sizeof(pkt_buf)) {
                uart_write_bytes(CANBOX_UART_PORT, (const char *)pkt_buf, pkt_idx);
                pkt_idx = 0;
                continue;
            }
        }

        if (pkt_idx > 3 && pkt_idx == expected_len) {
            uint8_t cmd = pkt_buf[1];
            pkt_count++;

            send_brief_to_usb("HU->CAN", pkt_count, pkt_idx, cmd);
            uart_write_bytes(CANBOX_UART_PORT, (const char *)pkt_buf, pkt_idx);

            pkt_idx = 0;
            expected_len = 0;
        }
    }
}