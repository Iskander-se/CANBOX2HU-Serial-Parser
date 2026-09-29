#include "board_hw.h"
#include "driver/usb_serial_jtag.h"

void init_usb_serial(void) {
    usb_serial_jtag_driver_config_t jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    jtag_config.rx_buffer_size = BUF_SIZE;
    jtag_config.tx_buffer_size = BUF_SIZE;
    usb_serial_jtag_driver_install(&jtag_config);
}

void init_uarts(void) {
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