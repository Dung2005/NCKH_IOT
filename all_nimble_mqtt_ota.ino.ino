#include <WiFi.h>
#include <NimBLEDevice.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <DHT.h>
#include "MQ135.h"
#include <Update.h>
#include <ArduinoJson.h>
#include "SPIFFS.h"
#include <SPI.h>
#include <MFRC522.h>

#define EEPROM_SIZE 512
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define RESET_BUTTON_PIN 27
#define WARN_LED_DO 33    // hoat dong
#define WARN_LED_VANG 25  // khong ket noi
#define WARN_LED_XANH 26  // chua cai dat

/*
DTH: 4
LED: 2
CO2: 35
BUTTON: 34
LIGHSENSOR: 32
HUMITYSOIL: 33
RAINSENSOR: 25
*/

#define DHTPIN 23
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
#define LED_CONTROLL 2
#define PIN_MQ135 22
MQ135 mq135_sensor = MQ135(PIN_MQ135);
#define LIGH_SENSOR 34
#define SOIL_SENSOR 33
#define RAIN_SENSOR 35
#define SS_PIN 5      // Chân GPIO5 (D1 trên ESP8266) -> GPIO5 trên ESP32
#define RST_PIN 16    // Chân GPIO16 (D0 trên ESP8266) -> GPIO16 trên ESP32
#define SERVO_PIN 17  // Chân GPIO17 (tùy chỉnh theo ESP32)

String rfid_uids[10];  // Giả sử tối đa 10 UID

MFRC522 rfid(SS_PIN, RST_PIN);


/*
Status device:
Không load được dữ liệu từ EEPROM: "no_data"
Không kết nối được WiFi: "wifi_error"
Không kết nối được MQTT: "mqtt_error"
Kết nối WiFi và MQTT thành công: "connected"
*/
String statusDevice = "";

// Khai báo các biến toàn cục
bool resetButtonPressed = false;         // Biến lưu trạng thái nút reset
unsigned long resetButtonPressTime = 0;  // Biến lưu thời gian nhấn nút reset

WiFiClient espClient;
PubSubClient client(espClient);

char wifiSSID[32];
char wifiPass[64];
char mqttUser[32];
char mqttServer[32];
char mqttPass[64];
char *mqttTopic = "v1/devices/me/telemetry";
int mqttPort = 1883;

bool wifiConnected = false;
bool mqttConnected = false;
bool isChangedData = false;

NimBLEServer *pServer;
NimBLECharacteristic *pCharacteristic;
String rxValue = "";

String fw_title = "";
String fw_version = "v0.0.2";
String fw_tag = "";
int fw_size = 0;
String fw_checksum_algorithm = "";
String fw_checksum = "";

void setWarnLed(const int type);
void setupBLE();
bool connectToWiFi(const char *ssid, const char *password);
bool connectToMQTT();
void sendDataMqtt();
void sendDHT11Data();
void onBLEReceive(String jsonData);
void saveCredentialsToEEPROM();
void loadCredentialsFromEEPROM();
void handleResetButton();
void resetDevice();
void callback(char *topic, byte *payload, unsigned int length);
void readRFID();
bool downloadFirmware(const char *deviceToken, const char *fwTitle, const char *fwVersion, const char *savePath = NULL);
void checkAndUpdateFirmware(const char *deviceToken, const char *fwTitle, const char *fwVersion);
void performUpdate(const char *filePath);

class MyBLECallbacks : public BLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pCharacteristic) {
    String rxData = pCharacteristic->getValue().c_str();
    if (rxData.length() > 0) {
      Serial.println("Received credentials over BLE");
      rxValue += rxData;
      Serial.println("Received Data: " + rxValue);
      if (rxValue.indexOf(';') != -1) {
        onBLEReceive(rxValue);
        Serial.println("Received Data" + rxValue);
        rxValue = "";
      }
    } else {
      Serial.println("Received empty data");
    }
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_CONTROLL, OUTPUT);
  pinMode(WARN_LED_DO, OUTPUT);
  pinMode(WARN_LED_VANG, OUTPUT);
  pinMode(WARN_LED_XANH, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  SPI.begin();
  rfid.PCD_Init();
  client.setCallback(callback);
  dht.begin();
  EEPROM.begin(EEPROM_SIZE);
  loadCredentialsFromEEPROM();
  setWarnLed(0);

  if (statusDevice == "no_data") {
    setWarnLed(2);
    Serial.println("No data found in EEPROM");
    setupBLE();
    while (statusDevice != "connected") {
      if (isChangedData) {
        connectToWiFi(wifiSSID, wifiPass);
        if (wifiConnected) {
          connectToMQTT();
        }
        if (mqttConnected) {
          String message = "SUCCESS";
          pCharacteristic->setValue(message.c_str());
          pCharacteristic->notify();
          saveCredentialsToEEPROM();
          statusDevice = "connected";
          isChangedData = false;
        }
      }
      delay(2000);
    }
    BLEDevice::deinit();          // Tắt BLE stack
    esp_bt_controller_disable();  // Tắt Bluetooth controller
    setWarnLed(1);
  }
  if (!SPIFFS.begin(true))  // ESP32 cần tham số true để format nếu chưa có hệ thống file
  {
    Serial.println("Failed to mount file system");
    return;
  }

  connectToWiFi(wifiSSID, wifiPass);
}

int timeSend = 60 * 1000;
unsigned long lastTime = 0;

void loop() {
  unsigned long currentTime = millis();

  handleResetButton();

  // Kiểm tra WiFi
  if (WiFi.status() != WL_CONNECTED) {
    setWarnLed(3);
    WiFi.disconnect();
    WiFi.reconnect();
    delay(1000);  // Cho phép kết nối lại
    return;
  }

  // Kiểm tra MQTT
  if (!mqttConnected || !client.connected()) {
    setWarnLed(3);
    connectToMQTT();
    if (!mqttConnected || !client.connected()) {
      delay(1000);  // Chờ trước khi thử lại
      return;
    }
  }

  // WiFi & MQTT đều ổn
  setWarnLed(1);
  client.loop();
  // Gửi dữ liệu mỗi timeSend ms
  if (currentTime - lastTime >= timeSend || currentTime < lastTime) {
    sendDataMqtt();
    lastTime = currentTime;
  }

  // Đọc RFID (nếu cần)
  readRFID();

  delay(100);  // Delay nhỏ để không quá tải CPU
}
void sendDataMqtt() {
  // // air
  // float humidity = dht.readHumidity();
  // float temperature = dht.readTemperature();

  // // ppm
  // float correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);

  // light
  int lightPercentage = map(analogRead(LIGH_SENSOR), 0, 1023, 0, 100);

  // // soil
  // int moisturePercent = map(analogRead(SOIL_SENSOR), 1023, 0, 0, 100);

  // rain
  int rainPercentage = map(analogRead(RAIN_SENSOR), 0, 1023, 0, 100);

  // total
  String payload = "{";
  // payload += "\"humidity\":" + String(humidity, 1) + ",";
  // payload += "\"temperature\":" + String(temperature, 1) + ",";
  // payload += "\"air_quality\":" + String(correctedPPM, 1);
  payload += "\"light\":" + String(lightPercentage) + ",";
  // payload += "\"soil_moisture\":" + String(moisturePercent) + ",";
  payload += "\"rain\":" + String(rainPercentage);
  payload += "}";
  client.publish(mqttTopic, payload.c_str());
  Serial.println("Data sent to MQTT: " + payload);
}

void setWarnLed(const int type) {
  bool red = false, green = false, blue = false;
  if (type == 1) {
    red = true;
  }
  if (type == 2) {
    green = true;
  }
  if (type == 3) {
    blue = true;
  }
  digitalWrite(WARN_LED_DO, red);
  digitalWrite(WARN_LED_VANG, green);
  digitalWrite(WARN_LED_XANH, blue);
}
bool connectToWiFi(const char *ssid, const char *password) {
  Serial.println("Connecting to WiFi...");
  Serial.println("CONNECT WITH " + String(ssid) + " | " + String(password));
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - startTime > 30000) {
      Serial.println("\nFailed to connect to WiFi");
      wifiConnected = false;
      statusDevice = "no_connected";
      return false;
    }
  }
  Serial.println("\nWiFi connected");
  statusDevice = "wifi_connected";
  wifiConnected = true;
  return true;
}

bool connectToMQTT() {
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  Serial.print("Connecting to MQTT...");
  if (client.connect("ESP32Client", mqttUser, mqttPass)) {
    Serial.println("MQTT connected");
    mqttConnected = true;
    // Đăng ký nhận thông tin Shared Attributes khi có thay đổi
    client.subscribe("v1/devices/me/attributes");
    // Gửi yêu cầu lấy giá trị Shared Attributes
    String request = "{\"sharedKeys\":\"lightState\"}";
    client.publish("v1/devices/me/attributes/request/1", request.c_str());
    if (statusDevice == "wifi_connected") {
      statusDevice = "connected";
      setWarnLed(1);
    }
    mqttConnected = true;
    return true;
  } else {
    Serial.print("Failed to connect to MQTT, rc=");
    Serial.println(client.state());
    mqttConnected = false;
    return false;
  }
}
void sendDHT11Data() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if (isnan(humidity)) {
    Serial.println("Failed to read from DHT11 sensor!");
    return;
  }
  String payload = "{\"humidity\": " + String(humidity) + ", \"temperature\":" + String(temperature) + "}";
  client.publish(mqttTopic, payload.c_str());
  Serial.println("Data sent to MQTT: " + payload);
}

void setupBLE() {
  NimBLEDevice::init("ESP32_NimBLE");
  NimBLEServer *pServer = NimBLEDevice::createServer();
  NimBLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pCharacteristic->setCallbacks(new MyBLECallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("BLE setup complete, waiting for credentials...");
}

void onBLEReceive(String jsonData) {
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, jsonData);
  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    return;
  }
  strcpy(wifiSSID, doc["ssid"]);
  strcpy(wifiPass, doc["password"]);
  strcpy(mqttUser, doc["mqttUser"]);
  strcpy(mqttServer, doc["mqttServer"]);
  mqttPort = doc["mqttPort"];
  Serial.printf("onBLEReceive.LOG -> SSI: %s\n", wifiSSID);
  Serial.printf("onBLEReceive.LOG -> PASSWORD: %s\n", wifiPass);
  Serial.printf("onBLEReceive.LOG -> MQTTUSER: %s\n", mqttUser);
  Serial.printf("onBLEReceive.LOG -> MQTTSERVER: %s\n", mqttServer);
  Serial.printf("onBLERecevie.LOG -> MQTTPORT: $d\n", mqttPort);
  isChangedData = true;
}

void saveCredentialsToEEPROM() {
  EEPROM.writeString(0, wifiSSID);
  EEPROM.writeString(32, wifiPass);
  EEPROM.writeString(96, mqttUser);
  EEPROM.writeString(128, mqttPass);
  EEPROM.writeString(160, mqttServer);
  EEPROM.write(192, (uint8_t)(mqttPort >> 8));
  EEPROM.write(193, (uint8_t)(mqttPort & 0xFF));
  EEPROM.commit();
}

void loadCredentialsFromEEPROM() {
  EEPROM.readString(0).toCharArray(wifiSSID, sizeof(wifiSSID));
  EEPROM.readString(32).toCharArray(wifiPass, sizeof(wifiPass));
  EEPROM.readString(96).toCharArray(mqttUser, sizeof(mqttUser));
  EEPROM.readString(128).toCharArray(mqttPass, sizeof(mqttPass));
  EEPROM.readString(160).toCharArray(mqttServer, sizeof(mqttServer));
  mqttPort = (EEPROM.read(192) << 8) | EEPROM.read(193);

  if (strlen(wifiSSID) == 0 || strlen(wifiPass) == 0 || strlen(mqttUser) == 0 || strlen(mqttServer) == 0) {
    statusDevice = "no_data";
    Serial.println("No data found in EEPROM");
  } else {
    statusDevice = "data_found";
    Serial.println("Credentials loaded from EEPROM.");
  }
}

#define DEBOUNCE_DELAY 50  // 50ms để lọc nhiễu

void handleResetButton() {
  static unsigned long lastDebounceTime = 0;
  int buttonState = digitalRead(RESET_BUTTON_PIN);

  if (buttonState == HIGH) {
    if (!resetButtonPressed && millis() - lastDebounceTime > DEBOUNCE_DELAY) {
      resetButtonPressTime = millis();
      resetButtonPressed = true;
      lastDebounceTime = millis();
    } else if (resetButtonPressed && millis() - resetButtonPressTime > 3000) {
      Serial.println("Reset button held for 10 seconds. Resetting device...");
      resetDevice();
      resetButtonPressed = false;
    }
  } else {
    resetButtonPressed = false;
  }
}
//xoa tất cả thông tin cấu hình khi esp khởi động lại nó sẽ ycau nhập lại
void resetDevice() {
  memset(wifiSSID, 0, sizeof(wifiSSID));  //xoá ssid wifi
  memset(wifiPass, 0, sizeof(wifiPass));
  memset(mqttUser, 0, sizeof(mqttUser));
  memset(mqttServer, 0, sizeof(mqttServer));
  mqttPort = 0;               //
  saveCredentialsToEEPROM();  //lưu trạng thái đã xoá vào eeprom để tránh xử dụng dlieu cũ
  ESP.restart();
}

bool otaRequested = false;

void callback(char *topic, byte *payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message received: ");
  Serial.println(message);

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, message);
  if (!error) {
    //=== Light Control ===
    bool lightState = doc["lightState"];
    if (doc["shared"]["lightState"]) lightState = doc["shared"]["lightState"];
    if (lightState) {
      digitalWrite(LED_CONTROLL, HIGH);
    } else {
      digitalWrite(LED_CONTROLL, LOW);
    }

    //== RFID UID update ==
    if (doc["shared"]["RFID_UIDs"]) {
      JsonArray array = doc["shared"]["RFID_UIDs"].as<JsonArray>();
      int i = 0;
      for (String uid : array) {
        if (i < 10)  // Giới hạn 10 UID
        {
          rfid_uids[i] = uid;
          i++;
        }
      }
    }

    //  servo
    if (doc["shared"]["servoState"]) {
      int servoState = doc["shared"]["servoState"];
      if (servoState == 1) {
        client.publish(mqttTopic, String("{\"RFID\":\"APP\", \"open\": true}").c_str());
        digitalWrite(SERVO_PIN, HIGH);
      }
    }

    // neu nhan duoc message moi servoState
    if (doc.containsKey("servoState")) {
      int servoState = doc["servoState"];
      if (servoState == 1) {
        digitalWrite(SERVO_PIN, HIGH);
        client.publish(mqttTopic, String("{\"RFID\":\"APP\", \"open\": true}").c_str());
      } else {
        digitalWrite(SERVO_PIN, LOW);
        client.publish(mqttTopic, String("{\"RFID\":\"APP\", \"open\": false}").c_str());
      }
    }

    // neu nhan duoc message moi rfid_uids
    if (doc["RFID_UIDs"]) {
      JsonArray array = doc["RFID_UIDs"].as<JsonArray>();
      int i = 0;
      for (String uid : array) {
        if (i < 10)  // Giới hạn 10 UID
        {
          rfid_uids[i] = uid;
          i++;
        }
      }
    }

    if (doc["helloWorld"] == "ota") {
      Serial.println("TEST OTA");
    }

    // if(doc["shared"].containsKey("fw_version")){
    //   Serial.println("Phiên bản MQTT gửi về: " + String(doc["shared"]["fw_version"]));
    //   Serial.println("Phiên bản HIỆN TẠI : " + String(fw_version));
    //   if(!strcmp(fw_version, doc["shared"]["fw_version"])){
    //     Serial.println("BẮT ĐẦU UPDATE");
    //     checkAndUpdateFirmware(mqttUser, doc["shared"]["fw_title"].as<const char*>(), doc["shared"]["fw_version"].as<const char*>());
    //     fw_version = strdup(doc["shared"]["fw_version"].as<const char*>());
    //   }
    // }

    // Then modify your version check logic:
    if (doc["shared"].containsKey("fw_version")) {
      String new_version = doc["shared"]["fw_version"].as<String>();
      Serial.println("Phiên bản MQTT gửi về: " + new_version);
      Serial.println("Phiên bản HIỆN TẠI : " + fw_version);

      if (fw_version != new_version) {
        Serial.println("BẮT ĐẦU UPDATE");
        String new_title = doc["shared"]["fw_title"].as<String>();
        checkAndUpdateFirmware(mqttUser, new_title.c_str(), new_version.c_str());
        fw_version = new_version;
      }
    }

    if (doc.containsKey("fw_version")) {
      String new_version = doc["fw_version"].as<String>();
      Serial.println("Phiên bản MQTT gửi về: " + new_version);
      Serial.println("Phiên bản HIỆN TẠI : " + fw_version);

      if (fw_version != new_version) {
        Serial.println("BẮT ĐẦU UPDATE");
        String new_title = doc["fw_title"].as<String>();
        checkAndUpdateFirmware(mqttUser, new_title.c_str(), new_version.c_str());
        fw_version = new_version;
      }
    }
  }
}
void readRFID() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial())
    return;

  String uid = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    uid += String(rfid.uid.uidByte[i], HEX);
  }

  Serial.print("RFID UID detected: ");
  Serial.println(uid);

  for (int i = 0; i < 10; i++) {
    client.loop();  // Đảm bảo MQTT luôn hoạt động
    Serial.println(rfid_uids[i]);

    if (rfid_uids[i] == uid) {
      Serial.println("Access granted! Moving servo...");
      digitalWrite(SERVO_PIN, HIGH);
      client.publish(mqttTopic, String("{\"RFID\":\"" + uid + "\", \"open\": true}").c_str());

      // Sử dụng while thay vì delay
      unsigned long startTime = millis();
      while (millis() - startTime < 3000) {
        client.loop();  // Đảm bảo MQTT vẫn chạy khi chờ

        yield();  // Cho phép ESP8266 xử lý tác vụ nền (WiFi, MQTT, watchdog)
      }

      digitalWrite(SERVO_PIN, LOW);
      client.publish(mqttTopic, String("{\"RFID\":\"" + uid + "\", \"open\": false}").c_str());
      return;
    }
  }
  Serial.println("Access denied!");
}
//Hàm tải firmware từ ThingsBoard
bool downloadFirmware(const char *deviceToken, const char *fwTitle, const char *fwVersion, const char *savePath) {
  HTTPClient http;

  // Xây dựng URL request
  String url = "http://" + String(mqttServer) + ":8080/api/v1/";
  url += String(deviceToken);
  url += "/firmware?title=";
  url += String(fwTitle);
  url += "&version=";
  url += String(fwVersion);

  // if (chunkSize > 0) {
  //   url += "&size=";
  //   url += chunkSize;
  //   url += "&chunk=";
  //   url += chunk;
  // }

  Serial.print("Requesting firmware from: ");
  Serial.println(url);

  if (!http.begin(espClient, url)) {
    Serial.println("Failed to begin HTTP connection");
    return false;
  }

  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    Serial.println("Firmware download started");

    // Nếu không chỉ định savePath, trả về true nếu request thành công
    if (savePath == NULL) {
      http.end();
      return true;
    }

    // Mở file để ghi firmware
    File file = SPIFFS.open(savePath, "w");
    if (!file) {
      Serial.println("Failed to open file for writing");
      http.end();
      return false;
    }

    // Nhận dữ liệu và ghi vào file
    int len = http.getSize();
    uint8_t buff[128] = { 0 };
    WiFiClient *stream = http.getStreamPtr();

    while (http.connected() && (len > 0 || len == -1)) {
      size_t size = stream->available();
      if (size) {
        int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
        file.write(buff, c);
        if (len > 0) {
          len -= c;
        }
        Serial.print(".");
      }
      delay(1);
    }

    file.close();
    Serial.println("\nFirmware download complete");
    http.end();
    return true;
  } else {
    Serial.printf("HTTP request failed, error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return false;
  }
}

// Hàm kiểm tra và cập nhật firmware
void checkAndUpdateFirmware(const char *deviceToken, const char *fwTitle, const char *fwVersion) {
  // Tải firmware
  const char *binFile = "/firmware.bin";
  if (downloadFirmware(deviceToken, fwTitle, fwVersion, binFile)) {
    // Tiến hành cập nhật firmware
    performUpdate(binFile);
  }
}

// Hàm cập nhật firmware
void performUpdate(const char *filePath) {
  File updateFile = SPIFFS.open(filePath, "r");
  if (!updateFile) {
    Serial.println("Failed to open firmware file");
    return;
  }

  size_t updateSize = updateFile.size();
  if (updateSize == 0) {
    Serial.println("Firmware file is empty");
    updateFile.close();
    return;
  }

  Serial.println("Starting firmware update...");
  if (Update.begin(updateSize)) {
    size_t written = Update.writeStream(updateFile);
    if (written == updateSize) {
      Serial.println("Firmware written successfully");
    } else {
      Serial.printf("Firmware written %d/%d bytes\n", written, updateSize);
    }

    if (Update.end()) {
      Serial.println("OTA done!");
      if (Update.isFinished()) {
        Serial.println("Update successfully completed. Rebooting...");
        updateFile.close();
        ESP.restart();
      } else {
        Serial.println("Update not finished? Something went wrong!");
      }
    } else {
      Serial.println("Error Occurred. Error #: " + String(Update.getError()));
    }
  } else {
    Serial.println("Not enough space to begin OTA");
  }

  updateFile.close();
}
