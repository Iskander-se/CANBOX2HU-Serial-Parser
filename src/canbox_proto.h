#ifndef CANBOX_PROTO_H
#define CANBOX_PROTO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

#define CANBOX_HEADER 0x2E

// Выносим функции обработки протокола
const char* get_packet_type_name(uint8_t cmd);
void parse_vehicle_speed(const uint8_t *pkt, size_t len);
void clear_pdc_payload(uint8_t *pkt, size_t len);
void send_dynamic_pdc_clear(void);

// Функция инжекции знаков
bool inject_road_sign_to_queue(RingbufHandle_t ringbuf, uint8_t speed_limit);
void process_road_sign_injection(RingbufHandle_t ringbuf);

// Парсеры потоков
void parser_and_forward_can_to_hu(const uint8_t *in_buf, size_t in_len);
void parse_and_forward_hu_to_can(const uint8_t *in_buf, size_t in_len);

#endif // CANBOX_PROTO_H