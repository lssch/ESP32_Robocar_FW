#include <Esp.h>
#include "main.h"
#include "ESP32DMASPIMaster.h"
#include <comms.h>
#include <iostream>
#include "types.h"
#include <electricui.h>

CRGB state_led;

// EUI packages
char nickname[8] = "Robocar";

enum class Path: uint8_t {
  HOME = 0,
  SENSORS,
  SETTINGS,
  INFO,
};

class Gui {
public:
  Path path{Path::HOME};
  Path path_old{Path::HOME};
};

robocar_data_t data;
Gui gui;
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
        EUI_CUSTOM("gui", gui),
};

unsigned long timer_board_msg;

void setup() {
  // Initialise builtin led and early life sign for debugging purpose
  CFastLED::addLeds<WS2812, PIN_ESP32_STATE_LED, GRB>(&state_led, 1);
  set_led(state_led, {0,0,255,5});

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
  spi_master.setFrequency(45000000);
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

  set_led(state_led, {0,255,0,5});
  digitalWrite(PIN_ESP32_OK, HIGH);
  std::cout << "Initialisation done." << std::endl;
  timer_board_msg = millis();
}

void loop() {
  //check for new inbound data
  serial_rx_handler();

  switch (gui.path) {
    case Path::HOME:
      master.exchange({.state = Comms::AccessRequestType::GET,
                       .sensor = Comms::AccessRequestType::GET,
                       .data = Comms::AccessRequestType::GET,
                       .parameter = Comms::AccessRequestType::GET,
                       .request = Comms::AccessRequestType::SET});
      break;

    case Path::SENSORS:
      master.exchange({.state = Comms::AccessRequestType::GET,
                              .sensor = Comms::AccessRequestType::GET,
                              .data = Comms::AccessRequestType::GET,
                              .parameter = Comms::AccessRequestType::IGNORE,
                              .request = Comms::AccessRequestType::SET});
      break;

    case Path::SETTINGS:
      if (gui.path_old != Path::SETTINGS) {
        master.exchange({.state = Comms::AccessRequestType::GET,
                                .sensor = Comms::AccessRequestType::GET,
                                .data = Comms::AccessRequestType::GET,
                                .parameter = Comms::AccessRequestType::GET,
                                .request = Comms::AccessRequestType::SET});
        eui_send_tracked("parameter");
      } else if (data.request.safe_parameter)
        master.exchange({.state = Comms::AccessRequestType::GET,
                                .sensor = Comms::AccessRequestType::GET,
                                .data = Comms::AccessRequestType::GET,
                                .parameter = Comms::AccessRequestType::SET,
                                .request = Comms::AccessRequestType::SET});
      else
        master.exchange({.state = Comms::AccessRequestType::GET,
                                .sensor = Comms::AccessRequestType::GET,
                                .data = Comms::AccessRequestType::GET,
                                .parameter = Comms::AccessRequestType::IGNORE,
                                .request = Comms::AccessRequestType::SET});
      break;

    case Path::INFO:
      master.exchange({.state = Comms::AccessRequestType::GET,
                              .sensor = Comms::AccessRequestType::GET,
                              .data = Comms::AccessRequestType::GET,
                              .parameter = Comms::AccessRequestType::IGNORE,
                              .request = Comms::AccessRequestType::SET});
      break;
  }

  // send realtime data to the parser for use on the interface
  eui_send_tracked("sensor");
  eui_send_tracked("data");
  eui_send_tracked("state");

  if (data.request.reset_odomety and data.data.position.x == 0 and data.data.position.y == 0) {
    data.request.reset_odomety = 0;
    eui_send_tracked("request");
  }

  if (data.request.calibrate_imu and data.state.imu == State::Imu::CALIBRATING) {
    data.request.calibrate_imu = 0;
    eui_send_tracked("request");
  }

  // Send this data is only sent when the gui is on the right page
  if (millis() - timer_board_msg > 500) {
    //eui_send_tracked("board");
    if (state_led.r > 0)
      set_led(state_led, {0,255,0,5});
    else
      set_led(state_led, {255,255,0,5});
    timer_board_msg = millis();
  }

  gui.path_old = gui.path;
}

void inline reset() {
  set_led(state_led, {255,0,0,5});
  std::cout << "Device is resetting..." << std::endl;
  delay(100);
  ESP.restart();
}

void error_handler() {
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
  std::cout << "ESP32 is in ERROR-state..." << std::endl;
  set_led(state_led, {255,0,255,5});
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

void inline set_led(CRGB &led, const RGBA color) {
  led.red = color.red;
  led.green = color.green;
  led.blue = color.blue;
  led.maximizeBrightness(color.alpha);
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