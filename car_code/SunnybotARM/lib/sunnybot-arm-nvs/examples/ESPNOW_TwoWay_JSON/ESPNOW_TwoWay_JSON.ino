//双向传递JSON ESP32 传递电压值 ESP32 S3传递返回值
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <ESPAsyncWebServer.h>
//0x30,0xC6,0xF7,0x23,0xDD,0xE8
//0x7C,0xDF,0xA1,0xE8,0xC6,0x98
uint8_t broadcastAddress[] = {0x68,0xB6,0xB3,0x21,0xEA,0xF0}; // add peer address esp32 S3
int action = 0;
float voltage_k = 0.99;

unsigned long previousMillis = 0; //

String recv_jsondata;
String send_jsondata;

StaticJsonDocument<128> doc;     // for data < 1KB
StaticJsonDocument<128> docRecv; // for data < 1KB
// DynamicJsonDocument doc(1024);  // for data > 1KB

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// recieved data will process here
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{

  char *buff = (char *)incomingData; // char buffer
  recv_jsondata = String(buff);      // converting into STRING
  Serial.print("Recieved ");
  Serial.println(recv_jsondata); // Complete JSON data will be printed here
  DeserializationError error = deserializeJson(docRecv, recv_jsondata);

  if (!error)
  {

    // RECIEVED DATA
    action = docRecv["action"]; //要执行的动作
    Serial.println(action);     // values of a
  }

  else
  {
    Serial.print(F("JSON解码失败: "));
    Serial.println(error.f_str());
    return;
  }
}

void setup()
{

  //    ONBOARD LED WILL GLOW IN CASE OF RESET
  //    {Remove if you want}
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  delay(2000);
  digitalWrite(2, LOW);
  delay(2000);

  //

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  previousMillis = millis();
}

void loop()
{
  float analogA0Volts = float(analogReadMilliVolts(A0)) * 11.0 * voltage_k / 1000.0;
  float analogA3Volts = float(analogReadMilliVolts(A3)) * 11.0 * voltage_k / 1000.0;
  //定时发送
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 1000)
  {
    previousMillis = currentMillis;

    send_jsondata = "";        // Clearing  JSON STRING
    doc["BV"] = analogA0Volts; // Creating JSON
    doc["qrCode"] = "123-321";
    doc["color"] = "123";
    serializeJson(doc, send_jsondata); // DATA TO BE SENT 单次发送的数据不能超过 250字节
    //Serial.println(sizeof(send_jsondata) );
    //Serial.println( send_jsondata.length());
    esp_now_send(broadcastAddress, (uint8_t *)send_jsondata.c_str(), send_jsondata.length());
    Serial.println(send_jsondata);
  }
  //按收到数据执行动作
  if (action)
  {
    digitalWrite(2, HIGH); //灯灭
  }
  else
  {
    digitalWrite(2, LOW); //灯亮
  }
}
