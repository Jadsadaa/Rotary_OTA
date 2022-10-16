#include <ModbusMaster.h>
#include <HardwareSerial.h>
#include "REG_CONFIG.h"
#include <Wire.h>
#include <MCP342x.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#define A 4
#define B 2
#define Z 14

const char* ssid     = "greenio";
const char* password = "green7650";




int status = WL_IDLE_STATUS;
int PORT = 8883;

HardwareSerial modbus(2);
ModbusMaster node;

uint8_t address = 0x68;
MCP342x adc = MCP342x(address);

WiFiClientSecure wifiClient;
PubSubClient client(wifiClient);


unsigned long Time, Time2,  Time_moter[3];
unsigned long previousMillis = 0;
unsigned long interval = 30000;

long adc0, counter = 0, counter_moter[3], LOOP = 0 , newcounter = 0;
int Step = 5, Start = 0;

//WiFi&OTA 参数
String HOSTNAME = "EGATSet2";
#define PASSWORD "green7650" //the password for OTA upgrade, can set it in any char you want

//MQTT
String deviceToken = "kBrF4vFO8a0yG4ysXaP5"; //TV4em3w1NA21n6AyOWkO
char Server[] = "mqtt.thingcontrol.io";


// Modbus
struct Meter
{
  String temp;
  String hum;
};

Meter meter[10] ;

void setup() {
  Serial.begin(115200);
  modbus.begin(9600, SERIAL_8N1, 16, 17);
  node.begin(ID_meter, modbus);
  Wire.begin();

  WiFi.begin(ssid, password);
  reconnectWIFI();
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());



  client.setServer( Server, PORT );
  reconnectMqtt();

  uint64_t chipider;

  uint64_t chipid = ESP.getEfuseMac();
  unsigned long long1 = (unsigned long)((chipid & 0xFFFF0000) >> 16 );
  unsigned long long2 = (unsigned long)((chipid & 0x0000FFFF));

  String hex = String(long1, HEX) + String(long2, HEX);
  HOSTNAME.concat(hex);
  setupOTA();


  pinMode(A, INPUT_PULLUP); // internal pullup input pin 25
  pinMode(B, INPUT_PULLUP); // internalเป็น pullup input pin 25
  pinMode(Z, INPUT_PULLUP); // internalเป็น pullup input pin 27

  attachInterrupt(A, ai0, RISING);
  attachInterrupt(B, ai1, RISING);

  // Reset devices
  MCP342x::generalCallReset();
  delay(1); // MC342x needs 300us to settle, wait 1ms

  // Check device present
  Wire.requestFrom(address, (uint16_t)1);
  if (!Wire.available()) {
    Serial.print("No device found at address ");
    Serial.println(address, HEX);
    while (1);
  }

}
void loop() {
  // put your main code here, to run repeatedly:

  if (millis() - Time >= 300000)  //test whether the period has elapsed
  {
    read_Modbus(data_) ;
    Read_ADC();
    Cal_Rotery();
    sendDATA();
    //    Serial.print("counter0  :  ");
    //    Serial.println(counter_moter[0]);
    //    Serial.print("Time_moter0 :  ");
    //    Serial.println(Time_moter[0]);
    Time = millis();  //IMPORTANT to save the start time of the current LED state.
  }
  //  if ( counter != newcounter ) {
  //    Serial.println ("Counter:" + String (counter) + "    Loop:" + String (LOOP));
  //    newcounter = counter;
  //  }


  if (millis() - Time2 >= 3000)  //test whether the period has elapsed
  {
    setupOTA();
    reconnectWIFI();
    Time2 = millis();  //IMPORTANT to save the start time of the current LED state.
  }


  status = WiFi.status();
  if ( status == WL_CONNECTED)
  {
    if ( !client.connected() )
    {
      reconnectMqtt();
    }
  }

  ArduinoOTA.handle();
  reconnectWIFI();
  client.loop();

}


void read_Modbus(uint16_t  REG)
{
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[2];
  uint16_t dat[2];
  uint32_t value = 0;
  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readInputRegisters(REG, 2);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
  }
  for (int a = 0; a < 2; a++)
  {
    //Serial.print(data[a]);
    //Serial.print("\t");
  }
  //Serial.println("");
  meter[0].temp = data[0];
  meter[0].hum =  data[1];
  //Serial.println("----------------------");
}

void Read_ADC() {
  /// 6568 to 31561    0-10 BAR
  MCP342x::Config status;
  uint16_t err = adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
                                    MCP342x::resolution16, MCP342x::gain1,
                                    1000000, adc0, status);


  if (err) {
    Serial.print("Convert error: ");
    Serial.println(err);
  }
  else {
    //    Serial.print("adc0: ");
    //    Serial.println(adc0);
  }

  delay(1000);
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(B) == LOW) {
    if (Start == 0) {
      counter_moter[0] = counter  ;
      Time_moter[0] = millis();
      Start = 1;
      Step = 1;
    } else if (Start == 1 && Step == 2) {
      counter_moter[1] = counter  ;
      Time_moter[1] = millis();
      Step = 3;
    }
    counter++;

  } else if ( digitalRead(Z) == LOW ) {
    LOOP--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(A) == LOW) {
    if (Start == 0) {
      counter_moter[0] = counter  ;
      Time_moter[0] = millis();
      Start = 1;
      Step = 2;
    } else if (Start == 1 && Step == 1) {
      counter_moter[1] = counter  ;
      Time_moter[1] = millis();
      Step = 4;
    }
    counter --;
  } else if ( digitalRead(Z) == LOW ) {
    LOOP++;
  }
}


void Cal_Rotery() {
  if (Step == 3)
  {
    counter_moter[3] = counter_moter[1] - counter_moter[0];
    Time_moter[3] = Time_moter[1] - Time_moter[0];
    Serial.print("counter : ");
    Serial.println(counter_moter[3]);
    Serial.print("Time_moter : ");
    Serial.println(Time_moter[3]);
    String json = "";
    json.concat("{\"C_M_U\":");
    json.concat(counter_moter[3]);
    json.concat(",\"T_M_U\":");
    json.concat(Time_moter[3]);
    json.concat("}");
    Serial.println(json);
    client.publish( "v1/devices/me/telemetry",  json.c_str());

    Step = 5;
    Start = 0;
  } else if (Step == 4)
  {
    counter_moter[3] = counter_moter[1] - counter_moter[0];
    Time_moter[3] = Time_moter[1] - Time_moter[0];
    Serial.print("counter : ");
    Serial.println(counter_moter[3]);
    Serial.print("Time_moter : ");
    Serial.println(Time_moter[3]);
    String json = "";
    json.concat("{\"C_M_D\":");
    json.concat(counter_moter[3]);
    json.concat(",\"T_M_D\":");
    json.concat(Time_moter[3]);
    json.concat("}");
    Serial.println(json);
    client.publish( "v1/devices/me/telemetry",  json.c_str());

    Step = 5;
    Start = 0;
  }
}



void setupOTA()
{
  ArduinoOTA.setHostname(HOSTNAME.c_str());
  ArduinoOTA.setPassword(PASSWORD);
  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
}


void reconnectMqtt()
{
  if ( client.connect("", deviceToken.c_str(), deviceToken.c_str()) )
  {
    Serial.println( F("Connect MQTT Success."));
    client.subscribe("v1/devices/me/rpc/request/+");
  } else {
    Serial.println( F("Connect MQTT fail."));
    delay(1000);
  }
}

void reconnectWIFI()
{
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

void sendDATA() {
  String json = "";
  json.concat("{\"T\":");
  json.concat(meter[0].temp);
  json.concat(",\"H\":");
  json.concat(meter[0].hum);
  json.concat(",\"P\":");
  json.concat(adc0);
  json.concat("}");
  Serial.println(json);
  client.publish( "v1/devices/me/telemetry",  json.c_str());
}
