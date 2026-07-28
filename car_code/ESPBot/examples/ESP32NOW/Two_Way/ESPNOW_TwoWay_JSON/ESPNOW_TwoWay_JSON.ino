//双向传递JSON ESP32 传递电压值 ESP32 S3传递返回值 esp s3侧
#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>//点击这里会自动打开管理库页面: http://librarymanager/All#ArduinoJson

////0x30,0xC6,0xF7,0x23,0xDD,0xE8
uint8_t broadcastAddress[] = {0xA0,0xB7,0x65,0x58,0xE7,0x90}; // add peer address esp32
int action = 0;


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
    float batteryVoltage = docRecv["BV"]; 
    //要执行的动作
    Serial.println(batteryVoltage); // values of batteryVoltage
  }

  else
  {
    Serial.print(F("JSON解码失败:"));
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
  
  //定时发送
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 3000)
  {
    previousMillis = currentMillis;

    send_jsondata = ""; // Clearing  JSON STRING
    //切换信号
    if (action)
    {
      action = 0;
      doc["action"] = action; // Creating JSON
    }
    else
    {
      action = 1;
      doc["action"] = action; // Creating JSON
    }
    serializeJson(doc, send_jsondata); // DATA TO BE SENT
    esp_now_send(broadcastAddress, (uint8_t *)send_jsondata.c_str(),send_jsondata.length());
    Serial.println(send_jsondata);
  }

}
