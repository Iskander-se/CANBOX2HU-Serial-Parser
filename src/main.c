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

// Circular buffers for byte stream data
static RingbufHandle_t ringbuf_can_to_hu = NULL;
static RingbufHandle_t ringbuf_hu_to_can = NULL;


// Packet counters for brief logs
static uint32_t g_pkt_count_can_to_hu = 0;
static uint32_t g_pkt_count_hu_to_can = 0;

// Vehicle state variables
typedef struct {
    uint8_t current_speed_kmh;   // Текущая скорость в км/ч
    bool    speed_over;       // Флаг превышения порога 40 км/ч
    size_t  pdc_payload_len;     // Длина данных (LEN, байт 2) в пакете 0x26
    uint8_t current_speed_limit; // Текущий знак ограничения (0 = нет знака)
} vehicle_state_t;

static vehicle_state_t g_vehicle = {
    .current_speed_kmh   = 0,
    .speed_over       = false,
    .pdc_payload_len     = 0,
    .current_speed_limit = 30,
};


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

// Definition of packet type by CMD for brief log
static const char* get_packet_type_name(uint8_t cmd) {
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

// Sending brief report (brief) to USB CDC
static void send_brief_to_usb(const char* dir, uint32_t pkt_num, size_t len, uint8_t cmd) {
    char msg[80];
    int msg_len = snprintf(msg, sizeof(msg), "[%s] #%lu | Len:%d | Type:%s (0x%02X)\r\n",
                          dir, pkt_num, (int)len, get_packet_type_name(cmd), cmd);
    
    if (msg_len > 0) {
        usb_serial_jtag_write_bytes((const uint8_t*)msg, msg_len, 0);
    }
}

/**
 * @brief Формирует и отправляет динамический пакет отмены PDC/PLA (0x26) в ГУ
 */
static void send_dynamic_pdc_clear(void) {
    size_t payload_len = (g_vehicle.pdc_payload_len > 0) ? g_vehicle.pdc_payload_len : 12;
    size_t total_pkt_len = 3 + payload_len + 1;

    if (total_pkt_len > 64) return;

    // Массив сразу занулен ({0}), memset больше не нужен
    uint8_t pkt[64] = { CANBOX_HEADER, 0x26, (uint8_t)payload_len };

    // Считаем CS (инверсия суммы байт от CMD до конца DATA)
    uint32_t sum = 0;
    for (size_t i = 1; i < total_pkt_len - 1; i++) {
        sum += pkt[i];
    }
    pkt[total_pkt_len - 1] = (uint8_t)(~sum);

    uart_write_bytes(HU_UART_PORT, (const char *)pkt, total_pkt_len);
}

/**
 * @brief Parsing vehicle speed from CANBOX status packets
 */
static void parse_vehicle_speed(const uint8_t *pkt, size_t len) {
    uint8_t cmd = pkt[1];

    if (g_vehicle.pdc_payload_len == 0 && cmd == 0x26 && len >= 3) {
        g_vehicle.pdc_payload_len = pkt[2];
    }

    // Packet 0x14 (Vehicle Status)
    if (cmd == 0x14 && len >= 5) {
        g_vehicle.current_speed_kmh = pkt[4];
    }
    // Packet 0x24 (Speed / Handbrake)
    else if (cmd == 0x24 && len >= 5) {
        g_vehicle.current_speed_kmh = pkt[3];
    }

    bool prev_state = g_vehicle.speed_over;
    g_vehicle.speed_over = (g_vehicle.current_speed_kmh > 40);

    // Upon initial exceeding the 40 km/h threshold, send a cancellation packet
    if (!prev_state && g_vehicle.speed_over) {
       send_dynamic_pdc_clear();
    }
}



/**
 * @brief Safe addition of road sign packet to the send queue for Canbox
 * 
 * @param speed_limit SpeedSign (50, 60, 80  etc). 0 — remove sign.
 * @return true if packet successfully added to the circular buffer
 */
bool inject_road_sign_to_queue(uint8_t speed_limit) {
    if (ringbuf_hu_to_can == NULL) {
        return false;
    }


    // Structure: [0x2E] [CMD] [LEN] [DATA0] [DATA1] [DATA2] [DATA3] [CS]
    uint8_t pkt[8];
    
    pkt[0] = CANBOX_HEADER; // 0x2E
    pkt[1] = 0x32;          // CMD (Навигация / Дорожные знаки для VAG BAP)
    pkt[2] = 0x04;          // LEN (4 байта данных)
    
    pkt[3] = (speed_limit > 0) ? 0x01 : 0x00; // State: 0x01 = Актвен, 0x00 = Выключен
    pkt[4] = speed_limit;                     // Значение знака (км/ч)
    pkt[5] = 0x00;                            // Доп. знаки (дождь, время и т.д.)
    pkt[6] = 0x00;                            // Резерв

    // Calc CS ( inversion sum of all bytes in DATA)
    uint32_t sum = 0;
    for (size_t i = 1; i < 7; i++) {
        sum += pkt[i];
    }
    pkt[7] = (uint8_t)(~sum);

    //  Push bytes of the packet directly into the circular buffer of the queue HU -> CAN
    BaseType_t res = xRingbufferSend(
        ringbuf_hu_to_can, 
        pkt, 
        sizeof(pkt), 
        pdMS_TO_TICKS(10) // Timeout for waiting for space in the buffer
    );

    return (res == pdTRUE);
}

/**
 * @brief Периодическая проверка и инжекция знака ограничения скорости
 */
static void process_road_sign_injection(void) {
    static TickType_t last_send_time = 0;
    TickType_t now = xTaskGetTickCount();

    // 1. Определяем желаемый знак согласно скорости (исправлен порядок условий)
    uint8_t target_sign = 30;
    
    if (g_vehicle.current_speed_kmh > 70) {
        target_sign = 100;
    } else if (g_vehicle.current_speed_kmh > 50) {
        target_sign = 70;
    }

    // 2. Отправляем если:
    //    а) Знак изменился (мгновенная реакция)
    //    б) Прошло более 2000 мс (поддержание знака на приборке VAG BAP)
    bool sign_changed = (target_sign != g_vehicle.current_speed_limit);
    bool timeout_expired = ((now - last_send_time) >= pdMS_TO_TICKS(2000));

    if (sign_changed || timeout_expired) {

        if (inject_road_sign_to_queue(target_sign)) {
            g_vehicle.current_speed_limit = target_sign;
            last_send_time = now;
        }
    }
}

/**
 * @brief NULLing PLA/PDC (0x26) and CS
 */

static void clear_pdc_payload(uint8_t *pkt, size_t len) {
    if (len < 4 || pkt[1] != 0x26) return;

    if (len > 64) return;

    // Очищаем байты дистанций
    memset(&pkt[3], 0x00, len - 4); // Clear DATA bytes (from index 3 to len-2)

    // Пересчитываем CS (инверсия суммы байт от CMD до конца DATA)
    uint32_t sum = 0;
    for (size_t i = 1; i < len - 1; i++) {
        sum += pkt[i];
    }
    pkt[len - 1] = (uint8_t)(~sum);
}

/**
 * @brief Stream parser and filter for CANBOX -> HU packets
 */
static void parser_and_forward_can_to_hu(const uint8_t *in_buf, size_t in_len) {
    static uint8_t buf[512];
    static size_t buf_len = 0;

    // 1. Добавляем входящие байты в кольцевой/накопительный буфер задачи
    if (buf_len + in_len > sizeof(buf)) {
        // Переполнение — защита от зацикливания, сбрасываем половинку буфера
        memmove(buf, buf + (buf_len / 2), buf_len / 2);
        buf_len /= 2;
    }
    memcpy(buf + buf_len, in_buf, in_len);
    buf_len += in_len;

    // 2. Разбираем накопитель
    size_t pos = 0;
    while (pos < buf_len) {
        // Ищем заголовок 0x2E
        if (buf[pos] != CANBOX_HEADER) {
            pos++; // Пропускаем мусор (например 0xFF 0xFF)
            continue;
        }

        // Нам нужно минимум 3 байта для определения длины (Header, CMD, LEN)
        if (buf_len - pos < 3) {
            break; // Ждем дочитывания из UART
        }

        uint8_t cmd = buf[pos + 1];
        uint8_t data_len = buf[pos + 2];
        size_t total_pkt_len = 3 + data_len + 1; // Header(1) + CMD(1) + LEN(1) + Data(N) + CS(1)

        // Аномальный размер кадра (валидный пакет CANBOX обычно от 4 до 64 байт)
        if (data_len == 0 || total_pkt_len > 128) {
            pos++; // Неверная длина -> 0x2E был ложным (внутри данных), ищем следующий 0x2E
            continue;
        }

        // Ждем пока весь пакет долетит полностью
        if (buf_len - pos < total_pkt_len) {
            break; 
        }

        // --- ПАКЕТ ПОЛНОСТЬЮ СОБРАН В buf[pos ... pos + total_pkt_len - 1] ---
        
        
        g_pkt_count_can_to_hu++;
        send_brief_to_usb("CAN->HU", g_pkt_count_can_to_hu, total_pkt_len, cmd);

        // Обновляем текущее состояние скорости
        parse_vehicle_speed(&buf[pos], total_pkt_len);

        // Если это кадр визуализации PLA/PDC (0x26) и скорость > 40 км/ч — мутируем его
        if (cmd == 0x26 && g_vehicle.speed_over) {
            clear_pdc_payload(&buf[pos], total_pkt_len);
        }

        

        // Единая точка отправки абсолютно для ВСЕХ пакетов в ГУ
        uart_write_bytes(HU_UART_PORT, (const char *)&buf[pos], total_pkt_len);

        // Сдвигаемся на длину успешного пакета
        pos += total_pkt_len;
    }

    // 3. Удаляем из накопительного буфера обработанные байты
    if (pos > 0) {
        memmove(buf, buf + pos, buf_len - pos);
        buf_len -= pos;
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

/**
 * @brief Stream parser and forwarder for the reverse stream from Head Unit to CANBOX for brief logs
 */
static void parse_and_forward_hu_to_can(const uint8_t *in_buf, size_t in_len) {
    static uint8_t pkt_buf[256];
    static size_t pkt_idx = 0;
    static size_t expected_len = 0;

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
            g_pkt_count_hu_to_can++;

            // Логируем обратный поток от ГУ в USB CDC
            send_brief_to_usb("HU->CAN", g_pkt_count_hu_to_can, pkt_idx, cmd);

            uart_write_bytes(CANBOX_UART_PORT, (const char *)pkt_buf, pkt_idx);

            pkt_idx = 0;
            expected_len = 0;
        }
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
            parse_and_forward_hu_to_can(data, size);
            vRingbufferReturnItem(ringbuf_hu_to_can, (void *)data);
        }

        process_road_sign_injection();

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