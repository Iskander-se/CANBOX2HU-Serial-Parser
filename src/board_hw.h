#ifndef BOARD_HW_H
#define BOARD_HW_H

#include "driver/uart.h"

#define CANBOX_UART_PORT   UART_NUM_1
#define CANBOX_TX_PIN      3
#define CANBOX_RX_PIN      2

#define HU_UART_PORT       UART_NUM_2
#define HU_TX_PIN          4
#define HU_RX_PIN          5

#define BAUDRATE           115200
#define BUF_SIZE           2048

void init_usb_serial(void);
void init_uarts(void);

#endif // BOARD_HW_H