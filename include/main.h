//
// Created by Lars Schwarz on 21.11.2023.
//

#ifndef ESP32_ROBOCAR_FW_MAIN_H
#define ESP32_ROBOCAR_FW_MAIN_H

#include "FastLED.h"
#include "Color.h"

#define LED_BUILTIN 21
#define BTN_BUILTIN 0

#define HSPI_CS 5
#define HSPI_CLK 6
#define HSPI_MISO 7
#define HSPI_MOSI 8

#define PIN_ESP32_OK 1
#define PIN_ESP32_ERROR 2
#define PIN_ESP32_COMM_START 3
#define PIN_ESP32_RES2 4
#define PIN_ESP32_RES3 8
#define PIN_ESP32_STATE_LED 10

extern "C" {
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
}

/// @brief Interrupt to detect a reset request from the builtin button
void IRAM_ATTR ISR();

/// @brief Reset this microcontroller
void reset();

/// @brief Error handler, called when the system is in unstable state
void error_handler();

/// @brief Set the color of the Neopixel
/// @param[in,out] led The Object of the Neopixel which needs to be changed.
/// @param[in] color The desired color in RGBA space.
void set_led(CRGB &led, const RGBA color);

/// @brief Handler to handle serial receive commands
void serial_rx_handler();

/// @brief Handler to write to the serial port
/// @param[in] data The data which needs to be sent
/// @param[in] len The length of the data
void serial_tx_putc(uint8_t *data, uint16_t length);

#endif //ESP32_ROBOCAR_FW_MAIN_H
