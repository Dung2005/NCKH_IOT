#include <WiFi.h>    
#include <PubSubClient.h>  
#include <ArduinoJson.h>  


const char *wifi_ssid = "Wifi_tầng 2_Dũng";
const char *wifi_pass = "2334445555";


#define TOKEN               "M8G4RcEkDmwYtxQf3gwW" // accesstokenn của device
#define THINGSBOARD_SERVER  "demo.thingsboard.io" // tùy địa chỉ server

#define SERIAL_DEBUG_BAUD    115200


WiFiClient espClient;
PubSubClient client(espClient);


#define LED_PIN 16
#define AO_PIN 26

bool subscribed = false;
bool WifiConnected = false;
// callback khi nhận được message mới
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    return;
  }
  //xu ly trang thai led tu Server
   if (doc.containsKey("state")) {
    bool ledState = doc["state"];
    ChangeLedState(ledState);
  }
  // bool state = doc["state"].as<bool>();
  
  // if (state) {
  //   Serial.println("LED ON");
  //   digitalWrite(LED_PIN, HIGH);
  // } else {
  //   Serial.println("LED OFF");
  //   digitalWrite(LED_PIN, LOW);  
  // }
}

void connecto_wifi(const char *wifi_ssid ,const char *wifi_pass){
  Serial.print("\nconnecting to wifi: ");
  Serial.println(wifi_ssid);
  WiFi.begin(wifi_ssid , wifi_pass);
  unsigned long starttime = millis();
  const unsigned long tgian_cho = 30000;
  while(WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.print(".");
    if(millis() - starttime > tgian_cho){
      Serial.println("\nFailed to connect to WiFi ");
      WifiConnected = false;
      return;
    }
  }
  Serial.println("\nWfi connected");
  Serial.print("WiFi IP Address: ");
  Serial.print(WiFi.localIP());
  WifiConnected = true;
}
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    if (client.connect("ESP8266Client", TOKEN, "")) {
      Serial.println("connected");
      //dky chu de để nhận các thay đổi liên quan đến thuộc tính của thiết bị(ngưỡng, tt led , thong tin user)
      client.subscribe("v1/devices/me/attributes");
      client.subscribe("v1/devices/me/telemetry");//Nhận phản hồi hoặc thông báo từ server dựa trên dữ liệu đã gửi.
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
//
void ChangeLedState(bool state) {
  digitalWrite(LED_PIN, state ? HIGH : LOW);
  String payload = "{\"state\":" + String(state) + "}";
  Serial.print("\nPublishing state: ");
  Serial.println(payload);
  client.publish("v1/devices/me/attributes", payload.c_str());
}

bool rainDetection = false;
void SendRainSensorData(){
  int analogValue = analogRead(AO_PIN);
  // int lightState = digitalRead(DO_PIN);
  int rainIntensity = map(analogValue , 0 , 4095 , 0 , 100);//chuyển đổi gtri sang phần trăm
  //gui du lieu cuong do mua len thingsboard
  String payload = "{\rainIntensity\":" + String(rainIntensity) + "}";
  Serial.print("\nPublishing rain intensity: ");
  Serial.println(payload);
  client.publish("v1/devices/me/telemetry", payload.c_str());

  if(analogValue < 1500){
    Serial.println("troi mua bat dau bat den: ");// co the thay den = may bom, khi dat nguong mua se tu dong dung bom nuoc
    ChangeLedState(true);
  }
  else{
    Serial.println("troi ko mua ");
    ChangeLedState(false);
  }
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  connecto_wifi(wifi_ssid , wifi_pass);
  
  client.setServer(THINGSBOARD_SERVER, 1883);
  client.setCallback(callback);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN , LOW);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  
  client.loop();
  SendRainSensorData();
  delay(5000);
}
