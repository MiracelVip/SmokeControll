
/********************************************************************************
 *****  Import required libraries
 *******************************************************************************/
#include <AutoPID.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "FS.h"
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/********************************************************************************
 *****  Define Pins
 *******************************************************************************/
#define OUTPUT_SSR 16        //D0
#define OUTPUT_RELAY 13      //D7
#define TEMP_PROBE_PIN 14    //D5
#define OLED_SDA 0           //D2
#define OLED_SCL 4           //D1
/********************************************************************************
 *****  Config JSON
 *******************************************************************************/
bool loadConfig() {
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return false;
  }

  size_t size = configFile.size();
  if (size > 1024) {
    Serial.println("Config file size is too large");
    return false;
  }

  // Allocate a buffer to store contents of the file.
  std::unique_ptr<char[]> buf(new char[size]);

  // We don't use String here because ArduinoJson library requires the input
  // buffer to be mutable. If you don't use ArduinoJson, you may as well
  // use configFile.readString instead.
  configFile.readBytes(buf.get(), size);

  StaticJsonDocument<1000> doc;
  auto error = deserializeJson(doc, buf.get());
  if (error) {
    Serial.println("Failed to parse config file");
    return false;
  }

  const char* pid_p = doc["pid_p"];
  const char* pid_i = doc["pid_i"];
  const char* pid_d = doc["pid_d"];
  const char* pid_outputMin = doc["pid_outputMin"];
  const char* pid_outputMax = doc["pid_outputMax"];
  const char* pid_setpoint = doc["pid_setpoint"];
  const char* pid_automatic = doc["pid_automatic"];

  // Real world application would store these values in some variables for
  // later use.

  Serial.print("Loaded pid_p: ");Serial.println(pid_p);
  Serial.print("Loaded pid_i: ");Serial.println(pid_i);
  Serial.print("Loaded pid_d: ");Serial.println(pid_d);
  Serial.print("Loaded pid_outputMin: ");Serial.println(pid_outputMin);
  Serial.print("Loaded pid_outputMax: ");Serial.println(pid_outputMax);
  Serial.print("Loaded pid_setpoint: ");Serial.println(pid_setpoint);
  Serial.print("Loaded pid_automatic: ");Serial.println(pid_automatic);
  return true;
}
 
/********************************************************************************
 *****  PID Vars
 *******************************************************************************/
double pid_outputMin = 0;
double pid_outputMax = 0;
double pid_p_2 = pid_p.toDouble();
double pid_i = 0.0003;
double pid_d = 0;
double pid_temperature = 0;
double pid_setPoint = 0;
double pid_outputVal = 0;
AutoPID myPID(&pid_temperature, &pid_setPoint, &pid_outputVal, pid_outputMin, pid_outputMax, pid_p_2, pid_i, pid_d);

/********************************************************************************
 *****  DS18b20
 *******************************************************************************/
#define TEMP_READ_DELAY 2000        //can only read digital temp sensor every ~750ms
unsigned long lastTempUpdate;       //tracks clock time of last temp update
OneWire oneWire(TEMP_PROBE_PIN);
DallasTemperature temperatureSensors(&oneWire);

/********************************************************************************
 *****  Read the Temperature
 *      call repeatedly in loop, only updates after a certain time interval
 *      returns true if update happened
 *******************************************************************************/
bool updateTemperature() {
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    pid_temperature = temperatureSensors.getTempFByIndex(0);    //get temp reading
    temperatureSensors.requestTemperatures();               //request reading for next time
    lastTempUpdate = millis();
    return true;
  }
  return false;
}

/********************************************************************************
 *****  Webserver & WiFi
 *******************************************************************************/
const char* ssid = "BA-ES";
const char* password = "barth1414ivb";
AsyncWebServer server(80);

void setup(){
   Serial.begin(74880);                       // Serial port for debugging purposes
   while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  /***********************************************
   ****  SetUp
   ***********************************************/
  pinMode(OUTPUT_SSR, OUTPUT);
  pinMode(OUTPUT_RELAY, OUTPUT);
  temperatureSensors.begin();
  temperatureSensors.requestTemperatures();
  while (!updateTemperature()) {}             //wait until temp sensor updated
  myPID.setTimeStep(4000);                    //set PID update interval to 4000ms
  

  /***********************************************
   ****  Read the last Config
   ***********************************************/
    if (!SPIFFS.begin()) {
      Serial.println("Failed to mount file system");
      return;
  }
    if (!loadConfig()) {
      Serial.println("Failed to load config");
  } else {
      Serial.println("Config loaded");
  }

//  
//
//  // Connect to Wi-Fi
//  WiFi.begin(ssid, password);
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(1000);
//    Serial.println("Connecting to WiFi..");
//  }
//
//  // Print ESP32 Local IP Address
//  Serial.println(WiFi.localIP());
//
//  // Route for root / web page
//  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(SPIFFS, "/index.html", String(), false);
//  });
// server.on("/bootstrap.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(SPIFFS, "/bootstrap.min.js", "application/javascript");
//  });
// server.on("/bootstrap4-toggle.min.css", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(SPIFFS, "/bootstrap4-toggle.min.css", "text/css");
//  });
// server.on("/bootstrap4-toggle.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(SPIFFS, "/bootstrap4-toggle.min.js", "application/javascript");
//  });
// server.on("/bootstrap-input-spinner.js", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(SPIFFS, "/bootstrap-input-spinner.js", "application/javascript");
//  });
// server.on("/jquery.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(SPIFFS, "/jquery.min.js", "application/javascript");
//  });
// server.on("/sb-admin.min.css", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(SPIFFS, "/sb-admin.min.css", "text/css");
//  });
// server.on("/sb-admin.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(SPIFFS, "/sb-admin.min.js", "text/css");
//  });
//
//// Send a GET request to <ESP_IP>/get?input1=<inputMessage>
//  server.on("/setTarget", HTTP_GET, [] (AsyncWebServerRequest *request) {
//    String inputMessage;
//    String inputParam;
//    // GET input1 value on <ESP_IP>/get?input1=<inputMessage>
//    if (request->hasParam("target")) {
//      inputMessage = request->getParam("target")->value();
//      inputParam = "target";
//      Serial.print("msg");
//      Serial.println(inputMessage);
//      Serial.print("param");
//      Serial.println(inputParam);
////      setpoint = inputParam;
//    }
//   
//    else {
//      inputMessage = "No message sent";
//      inputParam = "none";
//    }
//    Serial.println(inputMessage);
//    request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" 
//                                     + inputParam + ") with value: " + inputMessage +
//                                     "<br><a href=\"/\">Return to Home Page</a>");
//  });
// 
//
//  // Route to set GPIO to HIGH
//
//  
//  // Route to set GPIO to LOW
// 
//  server.on("/getAct", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send_P(200, "text/plain", temperature.c_str());
//  });
//  server.on("/getOutput", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send_P(200, "text/plain", output.c_str());
//  });
//
//  // Start server
//  server.begin();
}
 
void loop(){
///////////////////////////////////////////////////////////////////////
///// PID
///////////////////////////////////////////////////////////////////////
  updateTemperature();
  //setPoint = analogRead(POT_PIN);
  //myPID.run(); //call every loop, updates automatically at certain time interval
  //analogWrite(OUTPUT_PIN, outputVal);
  //output = outputVal;
///////////////////////////////////////////////////////////////////////
///// END PID
/////////////////////////////////////////////////////////////////////// 
}
