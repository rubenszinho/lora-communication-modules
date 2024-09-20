# LoRa Communication Modules for IoT River Monitoring

This repository contains the code for the LoRa sender and receiver modules used in our [IoT River Monitoring Project](https://github.com/rubenszinho/lora-water-level-monitoring). The project utilizes LoRa technology for effective monitoring of rivers in urban areas, focusing on prevention and response to natural disasters during periods of heavy rainfall.

Both modules are developed using the PlatformIO IDE and are designed to work with ESP32 microcontrollers.

## Overview

The LoRa communication system consists of two main components:

1. **Sender Module**: Reads data from a sensor and transmits it via LoRa.
2. **Receiver Module**: Receives the LoRa packets and publishes the data to an MQTT broker.

## Hardware Requirements

### Common Components

- **ESP32 Development Board**
- **LoRa Transceiver Module** (e.g., SX1276)
- **OLED Display** (SSD1306)
- **Antenna** for LoRa module
- **Connecting Wires**
- **Breadboard** or **PCB**

### Additional for Sender Module

- **Analog Sensor** (e.g., water level sensor)

## Software Requirements

- **[PlatformIO IDE](https://platformio.org/)**
- **Arduino Core for ESP32**
- **Libraries**:
  - SPI
  - LoRa
  - Wire
  - Adafruit_GFX
  - Adafruit_SSD1306
  - WiFi (Receiver Module)
  - PubSubClient (Receiver Module)

## Installation

### Clone the Repository

```bash
git clone https://github.com/rubenszinho/lora-communication-modules.git
```

### Set Up PlatformIO

1. Install PlatformIO IDE.
2. Open the cloned project in PlatformIO.
3. Ensure all required libraries are installed via the PlatformIO Library Manager.

### Hardware Setup

#### Pin Configuration

##### LoRa Module Pins

| LoRa Module Pin | ESP32 Pin |
| --------------- | --------- |
| SCK             | GPIO 5    |
| MISO            | GPIO 19   |
| MOSI            | GPIO 27   |
| NSS (SS)        | GPIO 18   |
| RST             | GPIO 14   |
| DIO0            | GPIO 26   |

##### OLED Display Pins

| OLED Pin | ESP32 Pin |
| -------- | --------- |
| SDA      | GPIO 4    |
| SCL      | GPIO 15   |
| RES      | GPIO 16   |

### Configure Wi-Fi and MQTT (Receiver Module)

Update the following placeholders in `receiver.cpp`:

```cpp
// Wi-Fi credentials
const char* ssid = "your_wifi_ssid";
const char* password = "your_wifi_password";

// MQTT broker settings
const char* mqttServer = "your_mqtt_broker_address";
const int mqttPort = your_mqtt_broker_port;
const char* vmuser = "your_mqtt_username";
const char* vmpassword = "your_mqtt_password";
```

## Usage

### Sender Module

1. **Connect the Hardware**: Set up the ESP32, LoRa module, OLED display, and sensor according to the pin configurations.
2. **Upload the Code**: Use PlatformIO to upload `sender.cpp` to the ESP32.
3. **Power On**: The sender will read data from the analog sensor (GPIO 12) and send it via LoRa every 5 seconds.

### Receiver Module

1. **Connect the Hardware**: Set up the ESP32, LoRa module, and OLED display.
2. **Upload the Code**: Use PlatformIO to upload `receiver.cpp` to the ESP32.
3. **Ensure Network Connectivity**: The ESP32 must be connected to Wi-Fi to publish data to the MQTT broker.
4. **Power On**: The receiver will listen for LoRa packets and publish the received data to the MQTT topic `enoe/nivelrio`.

## Code Explanation

### Sender Module (`sender.cpp`)

- **Initialization**: Sets up the LoRa module and OLED display.
- **Data Reading**: Reads analog data from GPIO 12 (simulate sensor input).
- **Data Transmission**: Sends the data as a JSON string via LoRa.
- **Display Update**: Shows the packet number and status on the OLED display.
- **Loop Interval**: Sends data every 5 seconds.

### Receiver Module (`receiver.cpp`)

- **Initialization**: Sets up the LoRa module, Wi-Fi connection, MQTT client, and OLED display.
- **Packet Reception**: Listens for incoming LoRa packets.
- **Data Handling**: Parses the received data and displays it.
- **MQTT Publishing**: Publishes the data to the specified MQTT topic with QoS level 1.
- **Reconnection Logic**: Handles reconnection to the MQTT broker if the connection is lost.

## License

This project is licensed under the GPL License. See the [LICENSE](LICENSE) file for details.