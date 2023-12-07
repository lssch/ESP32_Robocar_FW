#include <Esp.h>
#include "main.h"
#include "ESP32DMASPIMaster.h"
#include <comms.h>
#include <iostream>
#include "types.h"
#include <electricui.h>
#include "FastLED.h"

CRGB state_led;

// EUI packages
char nickname[15] = "ESP32 UART/USB";
typedef struct {
  uint8_t mac[6];
  uint8_t ip[4];
} boardinformation_t;

boardinformation_t boardinformation;

robocar_data_t data;
// SPI instance for stm32 communication
ESP32DMASPI::Master spi_master;
// allocate memory for dma buffer and start dma slave with custom configuration
Comms::Comms::comms_packet_t *rx_packet_dma = reinterpret_cast<Comms::Comms::comms_packet_t *>(spi_master.allocDMABuffer(sizeof(Comms::Comms::comms_packet_t) + sizeof(Comms::Comms::comms_packet_t) % 4));
Comms::Comms::comms_packet_t *tx_packet_dma = reinterpret_cast<Comms::Comms::comms_packet_t *>(spi_master.allocDMABuffer(sizeof(Comms::Comms::comms_packet_t) + sizeof(Comms::Comms::comms_packet_t) % 4));
Comms::CommsMaster master{rx_packet_dma, tx_packet_dma, &data, &spi_master};

// Instantiate the communication interface's management object
eui_interface_t comm_links[] = {
        EUI_INTERFACE(&serial_tx_putc),
//        EUI_INTERFACE(&ws_tx_putc)
};

// Electric UI manages variables referenced in this array
eui_message_t tracked_variables[] = {
        EUI_CHAR_ARRAY_RO("name", nickname),
        EUI_CUSTOM("request", data.request),
        EUI_CUSTOM_RO("sensor", data.sensor),
        EUI_CUSTOM_RO("data", data.data),
        EUI_CUSTOM("parameter", data.parameter),
        EUI_CUSTOM_RO("state", data.state),
        EUI_CUSTOM("board", boardinformation),
};

void setup() {
  // Initialise builtin led and early life sign for debugging purpose
  CFastLED::addLeds<WS2812, PIN_ESP32_STATE_LED, GRB>(&state_led, 1);
  set_led(&state_led, {.red = 0, .green = 0, .blue = 255, .brightness = 5});

  delay(100);
  // Initialise Serialport for debugging
  USBSerial.begin(115200);
  std::cout << "Device is initialising..." << std::endl;

  // Initialising GPIO pins
  pinMode(PIN_ESP32_OK, OUTPUT);
  pinMode(PIN_ESP32_ERROR, OUTPUT);
  pinMode(PIN_ESP32_COMM_START, INPUT);
  pinMode(PIN_ESP32_RES2, INPUT);
  pinMode(PIN_ESP32_RES3, INPUT);
  digitalWrite(PIN_ESP32_OK, LOW);

  // Initialise spi port to communicate with stm32 main processor
  std::cout << "Initialise spi port..." << std::endl;
  spi_master.setSpiMode(SPI_MODE0);
  spi_master.setFrequency(50000000);
  spi_master.setMaxTransferSize(sizeof(Comms::Comms::comms_packet_t));
  spi_master.begin(HSPI, HSPI_CLK, HSPI_MISO, HSPI_MOSI, HSPI_CS);
  // clear the dma buffer
  for (int i = 0; i < sizeof(Comms::Comms::comms_packet_t) + sizeof(Comms::Comms::comms_packet_t) % 4; ++i) {
    rx_packet_dma->buffer[i] = 0;
    tx_packet_dma->buffer[i] = 0;
  }

  // Initialise Serialport over USB to communicate with eui
  Serial.begin(1500000);
  Serial.flush();
  // eUI setup to use both interfaces and provide packages to track and send a unique identifier for easy identification
  std::cout << "Initialise eUI..." << std::endl;
  eui_setup_interfaces(comm_links, sizeof(comm_links));
  EUI_TRACK(tracked_variables);
  eui_setup_identifier("esp32", 6);

  std::cout << "Attaching interrupt" << std::endl;
  attachInterrupt(BTN_BUILTIN, ISR, RISING);

  set_led(&state_led, {.red = 0, .green = 255, .blue = 0, .brightness = 5});
  digitalWrite(PIN_ESP32_OK, HIGH);
  std::cout << "Initialisation done." << std::endl;
}

void loop() {
  //check for new inbound data
  serial_rx_handler();

  // send realtime data to the parser for use on the interface
  eui_send_tracked("sensor");
  eui_send_tracked("data");
  eui_send_tracked("board");

  master.exchange({.state = Comms::AccessRequestTypes::IGNORE,
                   .sensor = Comms::AccessRequestTypes::GET,
                   .data = Comms::AccessRequestTypes::GET,
                   .parameter = Comms::AccessRequestTypes::GET,
                   .request = Comms::AccessRequestTypes::SET});

  // Send this data is only sent when the gui is on the right page

}

void IRAM_ATTR ISR() {
  reset();
}

void inline reset() {
  set_led(&state_led, {.red = 255, .green = 0, .blue = 0, .brightness = 5});
  std::cout << "Device is resetting..." << std::endl;
  delay(100);
  ESP.restart();
}

void error_handler() {
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  std::cout << "ESP32 is in ERROR-state..." << std::endl;
  set_led(&state_led, {.red = 255, .green = 0, .blue = 255, .brightness = 5});
  digitalWrite(PIN_ESP32_ERROR, HIGH);
  while (true) {
    std::cout << "Waiting for reset..." << std::endl;
    sleep(1);
  }
#pragma clang diagnostic pop
}

extern "C" {
PUTCHAR_PROTOTYPE {
  USBSerial.print(ch);
  return ch;
}
}

void inline set_led(CRGB* led, color_t color) {
  led->red = color.red;
  led->green = color.green;
  led->blue = color.blue;
  led->maximizeBrightness(ceil(2.5f*color.brightness));
  FastLED.show();
}

void inline serial_rx_handler() {
  // While we have data, we will pass those bytes to the ElectricUI parser
  while(Serial.available() > 0) {
    eui_parse(Serial.read(), &comm_links[0]);  // Ingest a byte
  }
}

void inline serial_tx_putc(uint8_t *data, uint16_t len) {
  Serial.write(data, len);
}