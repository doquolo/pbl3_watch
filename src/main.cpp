#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <WiFi.h>
#include <Wire.h>
#include <time.h>
#include <PubSubClient.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// WiFi credentials
const char* ssid = "owo";
const char* password = "123456789";

// NTP Server settings
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600;  // GMT+7 for Vietnam
const int daylightOffset_sec = 0;

// MQTT Configuration
const char* mqtt_broker = "broker.emqx.io";
const char* mqtt_topic = "thongtinbenhnhan/f2bSibFN4iNORrveDvPaPsAnWrr2";
const char* mqtt_username = "Bao";
const char* mqtt_password = "123123123";
const int mqtt_port = 1883;

// MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

uint32_t irBuffer[100];
uint32_t redBuffer[100];

int32_t bufferLength;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

byte pulseLED = 11;
byte readLED = 13;

int battVoltage = 0;
bool wifiConnected = false;
unsigned long lastMqttPublish = 0;
const unsigned long mqttPublishInterval = 250; // Publish every 250 ms

static const uint16_t screenWidth = 240;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight);

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

void initWiFi()
{
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    wifiConnected = true;
    
    // Configure time with NTP
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("NTP time configured");
    
    // Wait a bit for time to sync
    delay(2000);
  } else {
    Serial.println("\nWiFi connection failed");
    wifiConnected = false;
  }
}

void connectMQTT()
{
  mqttClient.setServer(mqtt_broker, mqtt_port);
  
  while (!mqttClient.connected()) {
    String client_id = "esp32-watch-" + String(WiFi.macAddress());
    Serial.printf("Connecting to MQTT broker as %s...\n", client_id.c_str());
    
    if (mqttClient.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed with state ");
      Serial.println(mqttClient.state());
      delay(2000);
    }
  }
}

void publishHealthData()
{
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  
  // Read temperature
  float temperatureC = particleSensor.readTemperature() - 7.0;
  
  // Get current IR value
  uint32_t currentIR = particleSensor.getIR();
  
  // Create JSON payload
  char payload[200];
  snprintf(payload, sizeof(payload),
           "{\"BPM\": %d, \"SpO2\": %d, \"TempC\": %.2f, \"IR\": %lu}",
           (validHeartRate) ? heartRate : 0,
           (validSPO2) ? spo2 : 0,
           temperatureC,
           currentIR,
           map(constrain(battVoltage, 3300, 4000), 3300, 4000, 0, 100));
  
  mqttClient.publish(mqtt_topic, payload);
  Serial.print("Published: ");
  Serial.println(payload);
}

void getFormattedTime(char* timeStr, char* dateStr)
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    sprintf(timeStr, "--:--");
    sprintf(dateStr, "No Time");
    return;
  }
  
  // Format time as HH:MM
  strftime(timeStr, 10, "%H:%M", &timeinfo);
  
  // Format date as "Day, DD Mon"
  strftime(dateStr, 20, "%a, %d %b", &timeinfo);
}

uint64_t prev_loop_time = 0;

void setup()
{
  Serial.begin(115200);
  
  // Initialize WiFi and NTP
  initWiFi();
  
  // Connect to MQTT broker
  if (wifiConnected) {
    connectMQTT();
  }

  // Initialize sensor
  Wire.setPins(3, 10);
  particleSensor.begin(Wire, I2C_SPEED_FAST);

  byte ledBrightness = 60;
  byte sampleAverage = 4;
  byte ledMode = 2;
  byte sampleRate = 100;
  int pulseWidth = 411;
  int adcRange = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableDIETEMPRDY(); // Enable temperature reading

  bufferLength = 100;

  // Read first 100 samples
  for (byte i = 0; i < bufferLength; i++)
  {
    while (particleSensor.available() == false)
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
  lv_init();

  tft.begin();
  tft.setRotation(0);

  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  ui_init();

  Serial.println("Setup done");
}

void loop()
{
  // Maintain MQTT connection
  if (wifiConnected) {
    if (!mqttClient.connected()) {
      connectMQTT();
    }
    mqttClient.loop();
  }

  // Reconnect WiFi if disconnected
  if (WiFi.status() != WL_CONNECTED && wifiConnected) {
    Serial.println("WiFi disconnected, attempting reconnect...");
    wifiConnected = false;
  }

  // Dumping the first 25 sets of samples
  for (byte i = 25; i < 100; i++)
  {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  // Take 25 new samples
  for (byte i = 75; i < 100; i++)
  {
    while (particleSensor.available() == false)
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.print(irBuffer[i], DEC);
    Serial.print(F(", HR="));
    Serial.print(heartRate, DEC);
    Serial.print(F(", HRvalid="));
    Serial.print(validHeartRate, DEC);
    Serial.print(F(", SPO2="));
    Serial.print(spo2, DEC);
    Serial.print(F(", SPO2Valid="));
    Serial.println(validSPO2, DEC);
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Calculate battery percentage
  battVoltage = analogReadMilliVolts(0) * 2;

  // Update UI with sensor data
  char buffer[15];
  sprintf(buffer, "%d\nbpm", (validHeartRate) ? heartRate : 0);
  lv_label_set_text(ui_bpm, buffer);
  
  sprintf(buffer, "%d\n%%", (validSPO2) ? spo2 : 0);
  lv_label_set_text(ui_spo2, buffer);
  
  sprintf(buffer, "%.02f\noC", particleSensor.readTemperature()-7.0);
  lv_label_set_text(ui_temp, buffer);

  // Get and display current time from NTP
  char timeStr[10];
  char dateStr[20];
  getFormattedTime(timeStr, dateStr);
  lv_label_set_text(ui_timestring, timeStr);
  lv_label_set_text(ui_datestring, dateStr);

  // Update battery display
  int battPercent = map(battVoltage, 3300, 4000, 0, 100);
  battPercent = constrain(battPercent, 0, 100);
  sprintf(buffer, "%d%% (%d mV)", battPercent, battVoltage);
  lv_label_set_text(ui_battpercent, buffer);
  lv_bar_set_value(ui_Bar1, battPercent, LV_ANIM_OFF);

  // Publish health data to MQTT periodically
  unsigned long currentMillis = millis();
  if (wifiConnected && (currentMillis - lastMqttPublish >= mqttPublishInterval)) {
    publishHealthData();
    lastMqttPublish = currentMillis;
  }

  // uint64_t elapesed_time = millis() - prev_loop_time;
  // prev_loop_time = millis();
  lv_tick_inc(25);
  lv_timer_handler();
  delay(25);
}