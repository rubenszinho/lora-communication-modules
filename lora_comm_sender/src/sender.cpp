//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//Libraries for Serialization
#include <ArduinoJson.h>
#include <cbor.h>
#include "sensor_data.pb-c.h"

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//433E6 for Asia
//866E6 for Europe  
//915E6 for North America
#define BAND 915E6

//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Protocol timing configuration (in seconds)
#define JSON_DURATION_SEC 10        // 1 minute
#define MESSAGEPACK_DURATION_SEC 10 // 2 minutes  
#define CBOR_DURATION_SEC 10       // 3 minutes
#define PROTOBUF_DURATION_SEC 240   // 4 minutes

// Send interval
#define SEND_INTERVAL_MS 5000       // 5 seconds between packets

// Array size for sensor data
#define ARRAY_SIZE 250
#define DEVICE_ID "ESP32_LORA_PROTOCOLS_001"

// Protocol enumeration
enum ProtocolType {
  PROTOCOL_JSON = 0,
  PROTOCOL_MESSAGEPACK,
  PROTOCOL_CBOR, 
  PROTOCOL_PROTOBUF
};

// Current protocol state
ProtocolType current_protocol = PROTOCOL_JSON;
unsigned long protocol_start_time = 0;
int packet_counter = 0;
uint32_t sequence_number = 0;

// Protocol names for display
const char* protocol_names[] = {"JSON", "MSGPACK", "CBOR", "PROTOBUF"};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// Sensor data structure
struct SensorData {
  float temperature[ARRAY_SIZE];
  float precipitation[ARRAY_SIZE]; 
  float soil_moisture[ARRAY_SIZE];
  float wind[ARRAY_SIZE];
  float pressure;
  uint32_t light_level;
  bool raining;
  uint64_t timestamp;
  char device_id[32];
  uint32_t battery_level;
  uint32_t sequence_number;
};

SensorData sensor_data;

// Performance metrics
struct PerformanceMetrics {
  uint64_t serialization_time_us;
  size_t message_size_bytes;
  uint32_t heap_before_kb;
  uint32_t heap_after_kb;
  const char* protocol_name;
};

PerformanceMetrics current_metrics;

// Function prototypes - IMPORTANT: Declare all functions before use
void checkProtocolSwitch();
void readSensorData();
void sendLoRaData();
void updateDisplay();
size_t serializeJSON(char* buffer, size_t buffer_size);
size_t serializeMessagePack(char* buffer, size_t buffer_size);
size_t serializeCBOR(char* buffer, size_t buffer_size);
size_t serializeProtobuf(char* buffer, size_t buffer_size);

void setup() {
  //initialize Serial Monitor
  Serial.begin(9600);
  Serial.println("=== ESP32 LoRa Protocol Rotation Test ===");
  Serial.printf("Device ID: %s\n", DEVICE_ID);
  
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA PROTOCOLS");
  display.display();
  
  Serial.println("LoRa Protocol Rotation Init");

  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    display.setCursor(0,20);
    display.print("LoRa FAILED!");
    display.display();
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0,10);
  display.print("LoRa OK!");
  display.display();
  
  // Initialize protocol timing
  protocol_start_time = millis() / 1000; // Convert to seconds
  
  // Initialize sensor data structure
  strcpy(sensor_data.device_id, DEVICE_ID);
  
  Serial.println("Protocol rotation schedule:");
  Serial.printf("  - JSON: %d seconds\n", JSON_DURATION_SEC);
  Serial.printf("  - MessagePack: %d seconds\n", MESSAGEPACK_DURATION_SEC);  
  Serial.printf("  - CBOR: %d seconds\n", CBOR_DURATION_SEC);
  Serial.printf("  - Protobuf: %d seconds\n", PROTOBUF_DURATION_SEC);
  Serial.printf("Starting with protocol: %s\n", protocol_names[current_protocol]);
  
  delay(2000);
  display.clearDisplay();
}

void loop() {
  static unsigned long lastSend = 0;
  unsigned long currentTime = millis();
  
  // Check if we need to switch protocols
  checkProtocolSwitch();
  
  // Send data every SEND_INTERVAL_MS
  if (currentTime - lastSend >= SEND_INTERVAL_MS) {
    sendLoRaData();
    lastSend = currentTime;
  }
  
  // Update display
  updateDisplay();
  
  delay(100);
}

void checkProtocolSwitch() {
  unsigned long current_time_sec = millis() / 1000;
  unsigned long elapsed_time = current_time_sec - protocol_start_time;
  
  bool should_switch = false;
  ProtocolType next_protocol = current_protocol;
  
  switch (current_protocol) {
    case PROTOCOL_JSON:
      if (elapsed_time >= JSON_DURATION_SEC) {
        next_protocol = PROTOCOL_MESSAGEPACK;
        should_switch = true;
      }
      break;
      
    case PROTOCOL_MESSAGEPACK:
      if (elapsed_time >= MESSAGEPACK_DURATION_SEC) {
        next_protocol = PROTOCOL_CBOR;
        should_switch = true;
      }
      break;
      
    case PROTOCOL_CBOR:
      if (elapsed_time >= CBOR_DURATION_SEC) {
        next_protocol = PROTOCOL_PROTOBUF;
        should_switch = true;
      }
      break;
      
    case PROTOCOL_PROTOBUF:
      if (elapsed_time >= PROTOBUF_DURATION_SEC) {
        next_protocol = PROTOCOL_JSON; // Loop back to beginning
        should_switch = true;
      }
      break;
  }
  
  if (should_switch) {
    current_protocol = next_protocol;
    protocol_start_time = current_time_sec;
    Serial.printf("Switching to protocol: %s\n", protocol_names[current_protocol]);
  }
}

void readSensorData() {
  // Fill arrays with predictable values for testing
  for (int i = 0; i < ARRAY_SIZE; i++) {
    float value = i * 0.25f;
    sensor_data.temperature[i] = value;
    sensor_data.precipitation[i] = value;
    sensor_data.soil_moisture[i] = value;
    sensor_data.wind[i] = value;
  }
  
  // Fill scalar fields
  sensor_data.pressure = 1013.25f;
  sensor_data.light_level = 512;
  sensor_data.raining = false;
  sensor_data.timestamp = millis();
  sensor_data.battery_level = 100;
  sensor_data.sequence_number = ++sequence_number;
}

void sendLoRaData() {
  // Collect metrics before serialization
  current_metrics.heap_before_kb = ESP.getFreeHeap() / 1024;
  current_metrics.protocol_name = protocol_names[current_protocol];
  
  // Read fresh sensor data
  readSensorData();
  
  // Allocate buffer for serialized data
  const size_t BUFFER_SIZE = 2 * 8192; // 16KB buffer
  char* buffer = (char*)malloc(BUFFER_SIZE);
  if (!buffer) {
    Serial.println("Failed to allocate buffer");
    return;
  }
  
  // Measure serialization time
  unsigned long start_time = micros();
  size_t serialized_size = 0;
  
  switch (current_protocol) {
    case PROTOCOL_JSON:
      serialized_size = serializeJSON(buffer, BUFFER_SIZE);
      break;
    case PROTOCOL_MESSAGEPACK:
      serialized_size = serializeMessagePack(buffer, BUFFER_SIZE);
      break;
    case PROTOCOL_CBOR:
      serialized_size = serializeCBOR(buffer, BUFFER_SIZE);
      break;
    case PROTOCOL_PROTOBUF:
      serialized_size = serializeProtobuf(buffer, BUFFER_SIZE);
      break;
  }
  
  unsigned long end_time = micros();
  current_metrics.serialization_time_us = end_time - start_time;
  current_metrics.message_size_bytes = serialized_size;
  current_metrics.heap_after_kb = ESP.getFreeHeap() / 1024;
  
  // Send via LoRa if serialization was successful
  if (serialized_size > 0) {
    Serial.printf("[%s] Sending LoRa packet: %d\n", 
                  protocol_names[current_protocol], packet_counter);
    
    LoRa.beginPacket();
    LoRa.write((uint8_t*)buffer, serialized_size);
    LoRa.endPacket();
    
    Serial.printf("[%s] Packet sent - Size: %d bytes, Time: %lu μs\n",
                  protocol_names[current_protocol], 
                  serialized_size, 
                  current_metrics.serialization_time_us);
    
    packet_counter++;
  } else {
    Serial.printf("[%s] Failed to serialize data\n", protocol_names[current_protocol]);
  }
  
  free(buffer);
}

size_t serializeJSON(char* buffer, size_t buffer_size) {
  JsonDocument doc;
  
  // Create arrays for vectors - Use full array size
  JsonArray temp_array = doc["temperature"].to<JsonArray>();
  JsonArray precip_array = doc["precipitation"].to<JsonArray>();
  JsonArray soil_array = doc["soil_moisture"].to<JsonArray>();
  JsonArray wind_array = doc["wind"].to<JsonArray>();
  
  for (int i = 0; i < ARRAY_SIZE; i++) {
    temp_array.add(sensor_data.temperature[i]);
    precip_array.add(sensor_data.precipitation[i]);
    soil_array.add(sensor_data.soil_moisture[i]);
    wind_array.add(sensor_data.wind[i]);
  }
  
  // Add scalar fields
  doc["pressure"] = sensor_data.pressure;
  doc["light_level"] = sensor_data.light_level;
  doc["raining"] = sensor_data.raining;
  doc["timestamp"] = (double)sensor_data.timestamp;
  doc["device_id"] = sensor_data.device_id;
  doc["battery_level"] = sensor_data.battery_level;
  doc["sequence_number"] = sensor_data.sequence_number;
  doc["protocol"] = "JSON";
  doc["array_size"] = ARRAY_SIZE;
  
  // Serialize to buffer
  return serializeJson(doc, buffer, buffer_size);
}

size_t serializeMessagePack(char* buffer, size_t buffer_size) {
  JsonDocument doc;
  
  // Create arrays for vectors - Use full array size
  JsonArray temp_array = doc["temperature"].to<JsonArray>();
  JsonArray precip_array = doc["precipitation"].to<JsonArray>();
  JsonArray soil_array = doc["soil_moisture"].to<JsonArray>();
  JsonArray wind_array = doc["wind"].to<JsonArray>();
  
  for (int i = 0; i < ARRAY_SIZE; i++) {
    temp_array.add(sensor_data.temperature[i]);
    precip_array.add(sensor_data.precipitation[i]);
    soil_array.add(sensor_data.soil_moisture[i]);
    wind_array.add(sensor_data.wind[i]);
  }
  
  // Add scalar fields
  doc["pressure"] = sensor_data.pressure;
  doc["light_level"] = sensor_data.light_level;
  doc["raining"] = sensor_data.raining;
  doc["timestamp"] = (double)sensor_data.timestamp;
  doc["device_id"] = sensor_data.device_id;
  doc["battery_level"] = sensor_data.battery_level;
  doc["sequence_number"] = sensor_data.sequence_number;
  doc["protocol"] = "MESSAGEPACK";
  doc["array_size"] = ARRAY_SIZE;
  
  // Serialize to MessagePack format
  return serializeMsgPack(doc, buffer, buffer_size);
}

size_t serializeCBOR(char* buffer, size_t buffer_size) {
  // Create root map
  cbor_item_t *root = cbor_new_definite_map(12);
  
  // Add device_id
  (void)cbor_map_add(root, (struct cbor_pair) {
    .key = cbor_move(cbor_build_string("device_id")),
    .value = cbor_move(cbor_build_string(sensor_data.device_id))
  });
  
  // Add sequence_number
  (void)cbor_map_add(root, (struct cbor_pair) {
    .key = cbor_move(cbor_build_string("sequence_number")),
    .value = cbor_move(cbor_build_uint32(sensor_data.sequence_number))
  });
  
  // Add pressure
  (void)cbor_map_add(root, (struct cbor_pair) {
    .key = cbor_move(cbor_build_string("pressure")),
    .value = cbor_move(cbor_build_float4(sensor_data.pressure))
  });
  
  // Add light_level
  (void)cbor_map_add(root, (struct cbor_pair) {
    .key = cbor_move(cbor_build_string("light_level")),
    .value = cbor_move(cbor_build_uint32(sensor_data.light_level))
  });
  
  // Add timestamp
  (void)cbor_map_add(root, (struct cbor_pair) {
    .key = cbor_move(cbor_build_string("timestamp")),
    .value = cbor_move(cbor_build_uint64(sensor_data.timestamp))
  });
  
  // Add protocol
  (void)cbor_map_add(root, (struct cbor_pair) {
    .key = cbor_move(cbor_build_string("protocol")),
    .value = cbor_move(cbor_build_string("CBOR"))
  });
  
  // Add battery_level
  (void)cbor_map_add(root, (struct cbor_pair) {
    .key = cbor_move(cbor_build_string("battery_level")),
    .value = cbor_move(cbor_build_uint32(sensor_data.battery_level))
  });
  
  // Add raining
  (void)cbor_map_add(root, (struct cbor_pair) {
    .key = cbor_move(cbor_build_string("raining")),
    .value = cbor_move(cbor_build_bool(sensor_data.raining))
  });
  
  // Add array_size
  (void)cbor_map_add(root, (struct cbor_pair) {
    .key = cbor_move(cbor_build_string("array_size")),
    .value = cbor_move(cbor_build_uint16(ARRAY_SIZE))
  });
  
  // Add temperature array - Full size
  cbor_item_t *temp_array = cbor_new_definite_array(ARRAY_SIZE);
  for (int i = 0; i < ARRAY_SIZE; i++) {
    (void)cbor_array_push(temp_array, cbor_move(cbor_build_float4(sensor_data.temperature[i])));
  }
  (void)cbor_map_add(root, (struct cbor_pair) {
    .key = cbor_move(cbor_build_string("temperature")),
    .value = cbor_move(temp_array)
  });
  
  // Add precipitation array - Full size
  cbor_item_t *precip_array = cbor_new_definite_array(ARRAY_SIZE);
  for (int i = 0; i < ARRAY_SIZE; i++) {
    (void)cbor_array_push(precip_array, cbor_move(cbor_build_float4(sensor_data.precipitation[i])));
  }
  (void)cbor_map_add(root, (struct cbor_pair) {
    .key = cbor_move(cbor_build_string("precipitation")),
    .value = cbor_move(precip_array)
  });
  
  // Add soil_moisture array - Full size
  cbor_item_t *soil_array = cbor_new_definite_array(ARRAY_SIZE);
  for (int i = 0; i < ARRAY_SIZE; i++) {
    (void)cbor_array_push(soil_array, cbor_move(cbor_build_float4(sensor_data.soil_moisture[i])));
  }
  (void)cbor_map_add(root, (struct cbor_pair) {
    .key = cbor_move(cbor_build_string("soil_moisture")),
    .value = cbor_move(soil_array)
  });
  
  // Add wind array - Full size
  cbor_item_t *wind_array = cbor_new_definite_array(ARRAY_SIZE);
  for (int i = 0; i < ARRAY_SIZE; i++) {
    (void)cbor_array_push(wind_array, cbor_move(cbor_build_float4(sensor_data.wind[i])));
  }
  (void)cbor_map_add(root, (struct cbor_pair) {
    .key = cbor_move(cbor_build_string("wind")),
    .value = cbor_move(wind_array)
  });
  
  // Serialize to buffer
  size_t serialized_size = cbor_serialize(root, (unsigned char*)buffer, buffer_size);
  
  // Clean up
  cbor_decref(&root);
  
  return serialized_size;
}

size_t serializeProtobuf(char* buffer, size_t buffer_size) {
  Sensor__SensorData pb_msg = SENSOR__SENSOR_DATA__INIT;
  
  // Allocate arrays for sensor data - Full size
  float temp_array[ARRAY_SIZE];
  float precip_array[ARRAY_SIZE];
  float soil_array[ARRAY_SIZE];
  float wind_array[ARRAY_SIZE];
  
  // Copy data to arrays
  for (int i = 0; i < ARRAY_SIZE; i++) {
    temp_array[i] = sensor_data.temperature[i];
    precip_array[i] = sensor_data.precipitation[i];
    soil_array[i] = sensor_data.soil_moisture[i];
    wind_array[i] = sensor_data.wind[i];
  }
  
  // Set array fields
  pb_msg.n_temperature = ARRAY_SIZE;
  pb_msg.temperature = temp_array;
  
  pb_msg.n_precipitation = ARRAY_SIZE;
  pb_msg.precipitation = precip_array;
  
  pb_msg.n_soil_moisture = ARRAY_SIZE;
  pb_msg.soil_moisture = soil_array;
  
  pb_msg.n_wind = ARRAY_SIZE;
  pb_msg.wind = wind_array;
  
  // Set scalar fields
  pb_msg.pressure = sensor_data.pressure;
  pb_msg.light_level = sensor_data.light_level;
  pb_msg.raining = sensor_data.raining;
  pb_msg.timestamp = sensor_data.timestamp;
  pb_msg.device_id = sensor_data.device_id;
  pb_msg.battery_level = sensor_data.battery_level;
  pb_msg.sequence_number = sensor_data.sequence_number;
  
  // Pack the message
  size_t packed_size = sensor__sensor_data__pack(&pb_msg, (uint8_t*)buffer);
  
  return packed_size;
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  display.setCursor(0,0);
  display.printf("LORA PROTOCOLS");
  
  display.setCursor(0,10);
  display.printf("Current: %s", protocol_names[current_protocol]);
  
  display.setCursor(0,20);
  unsigned long elapsed = (millis() / 1000) - protocol_start_time;
  display.printf("Time: %lu sec", elapsed);
  
  display.setCursor(0,30);
  display.printf("Packets: %d", packet_counter);
  
  display.setCursor(0,40);
  display.printf("Size: %u bytes", current_metrics.message_size_bytes);
  
  display.setCursor(0,50);
  display.printf("Heap: %u KB", ESP.getFreeHeap() / 1024);
  
  display.display();
}