#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

SFE_UBLOX_GNSS HAM_GNSS; // ZED-F9P
SFE_UBLOX_GNSS HAM_GNSS_L_Band; // NEO-D9S

const uint32_t LBand_frequency = 1556290000; //L-Band Frequency in Hz. get updated frequency from u-blox mqtt protocal eventually

// OLED Display Setup
#define SCREEN_WIDTH 128 // OLED Width in pixels
#define SCREEN_HEIGHT 32 // OLED Height in pixels

#define OLED_RESET -1 // reset pin for display
#define SCREEN_ADDRESS 0x3c
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// display functions
void display_info(String message);
void display_add_info(String message);
void display_info_lg(String message);

const char* ssid = "HAM_GNSS";
const char* password = "pass1234";

WebServer server(80); // Create server on port 80
WebSocketsServer webSocket = WebSocketsServer(81);

/*IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

void handle_NotFound();
void handle_OnConnect();
void handle_start_survey();
void handle_view_survey_log();
void handle_gnss_info();

// HTML Pages Functions
String Send_Index_HTML();
String Send_Start_Survey_HTML();
String Send_Survey_Log_HTML();
String Send_GNSS_Info_HTML();

// Web Script Functions
String Client_WebSocketJS();

//Web Socket Functions
void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length);

// surveying functions

StaticJsonDocument<200> json_doc_tx;
StaticJsonDocument<200> json_doc_rx;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // OLED Display
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();

  digitalWrite(LED_BUILTIN, HIGH);

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  display_info("Creating WiFi Access Point");
  delay(100);
  display_info("Connect to WiFi '");
  display_add_info(ssid);
  display_add_info("' pass: ");
  display_add_info(password);
  display_add_info(" Then go to -> 192.168.1.1 in browser");

  // Define routes
  server.on("/", handle_OnConnect);
  server.onNotFound(handle_NotFound);
  server.on("/start_survey", handle_start_survey);
  server.on("/view_survey_log", handle_view_survey_log);
  server.on("/gnss_info", handle_gnss_info);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("HTTP server started");

  //GNSS - ZED-F9P
  //Wire.begin();

  //HAM_GNSS.enableDebugging(Serial);
  if (HAM_GNSS.begin() == false) // Connect to the u-blox ZED-F9P module using Wire port
  {
    Serial.println("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing.");
    delay(2000);
  }

  Serial.print("u-blox GNSS module connected");

  // NEO-D9S
  while (HAM_GNSS_L_Band.begin(0x43) == false) {
    Serial.println("u=blpx NEO-D9S not detected at default I2C address. Please check wiring.");
  }

  Serial.println("u-blox NEO-D9S connected");

  //NEO-D9S Setup
  uint8_t ok = HAM_GNSS_L_Band.addCfgValset32(UBLOX_CFG_PMP_CENTER_FREQUENCY, LBand_frequency); // Default 1539812500 Hz  32bit val
  if (ok) ok = HAM_GNSS_L_Band.addCfgValset16(UBLOX_CFG_PMP_SEARCH_WINDOW, 2200); // Default 2200 Hz
  if (ok) ok = HAM_GNSS_L_Band.addCfgValset8(UBLOX_CFG_PMP_USE_SERVICE_ID, 0); // Default 1
  if (ok) ok = HAM_GNSS_L_Band.addCfgValset16(UBLOX_CFG_PMP_SERVICE_ID, 21845); //50821
  if (ok) ok = HAM_GNSS_L_Band.addCfgValset16(UBLOX_CFG_PMP_DATA_RATE, 2400); // Default 2400 bps
  if (ok) ok = HAM_GNSS_L_Band.addCfgValset8(UBLOX_CFG_PMP_USE_DESCRAMBLER, 1); // Default 1
  if (ok) ok = HAM_GNSS_L_Band.addCfgValset16(UBLOX_CFG_PMP_DESCRAMBLER_INIT, 26969); // Default 23560
  if (ok) ok = HAM_GNSS_L_Band.addCfgValset8(UBLOX_CFG_PMP_USE_PRESCRAMBLING, 0); // Default 0
  if (ok) ok = HAM_GNSS_L_Band.addCfgValset(UBLOX_CFG_PMP_UNIQUE_WORD, 16238547128276412563ull); // 0xE15AE893E15AE893

  // Configure NEO-D9S Baud rate to match ZED-F9P's baud rate
  if(ok) ok = HAM_GNSS_L_Band.addCfgValset(UBLOX_CFG_UART2_BAUDRATE, 38400); // match baudrate with ZED default
  if(ok) ok = HAM_GNSS_L_Band.addCfgValset(UBLOX_CFG_UART2OUTPROT_UBX, 1); // Enable UBX output on UART2
  if(ok) ok = HAM_GNSS_L_Band.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART2, 1); // Output UBX-RXM-PMP on UART2

  Serial.print("L-Band configuration: ");
  if (ok) {
    Serial.println("OK");
    HAM_GNSS_L_Band.sendCfgValset();
  } else {
    Serial.println("NOT OK!");
  }

  HAM_GNSS_L_Band.softwareResetGNSSOnly();

  // ZED-F9P Setup
  ok = HAM_GNSS.setI2CInput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_SPARTN); //Be sure SPARTN input is enabled
  if(ok) ok = HAM_GNSS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED); // set the differential mode - ambiguties
  if(ok) ok = HAM_GNSS.addCfgValset8(UBLOX_CFG_SPARTN_USE_SOURCE, 1); // use LBAND PMP message
  if(ok) ok = HAM_GNSS.addCfgValset8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_I2C, 1); // enable message output via i2c every second
  if(ok) ok = HAM_GNSS.addCfgValset8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_I2C, 1);
  if(ok) ok = HAM_GNSS.addCfgValset8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_I2C, 1);
  if(ok) ok = HAM_GNSS.addCfgValset8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_I2C, 1);
  if(ok) ok = HAM_GNSS.addCfgValset8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_I2C, 10); // enable message 1230 every 10 seconds

  if (ok) ok = HAM_GNSS.setDynamicSPARTNKeys(16,2294, 0, "d8f33f27fc2afd1db1624d5a45817d71", 16, 2297, 0, "9a5899dc0b6313245219d303f281db77"); // add encryption keys to decript NEO-DS9 messages.
  

  Serial.print("GNSS: configuration ");
  if (ok) {
    Serial.println("OK");
    HAM_GNSS.sendCfgValset();
  } else {
    Serial.println("NOT OK!");
  }
    

  HAM_GNSS.softwareResetGNSSOnly();
}

// testing
int interval = 1000; // 1 sec
unsigned long previousMillis = 0;

void loop() {
  // put your main code here, to run repeatedly:
  server.handleClient();
  webSocket.loop();

  /*unsigned long now = millis();
  if(now - previousMillis > interval) {
    String jsonString ="";
    JsonObject object = json_doc_tx.to<JsonObject>();
    object["message"] = "Hello from web socket. " + String(random(100));
    serializeJson(json_doc_tx, jsonString);
    // Testing websockets
    webSocket.broadcastTXT(jsonString); // can only be an array of chars
    previousMillis = now;
  }*/

  unsigned long now = millis();
  if(now - previousMillis > interval) {
    // Testing Request Poll Position
    if (HAM_GNSS.getPVT() == true && webSocket.connectedClients() != 0) 
    {
      String JsonString ="";
      JsonObject object = json_doc_tx.to<JsonObject>(); 

      int32_t latitude = HAM_GNSS.getLatitude() / int(1000000);
      int32_t longitude = HAM_GNSS.getLongitude() / int(10000000);
      int32_t altitude = HAM_GNSS.getAltitude();
      int32_t altitude_msl = HAM_GNSS.getAltitudeMSL();

      object["latitude"] = String(latitude);
      object["longitude"] = String(longitude);
      object["altitude"] = String(altitude);
      object["altitude_msl"] = String(altitude_msl);

      serializeJson(object, JsonString);
      Serial.println(JsonString);
      webSocket.broadcastTXT(JsonString);

    }
  }
}

// Routes
void handle_OnConnect() {
  Serial.println("Client Connected to ESP32 Web Server.");
  display_info_lg("Stand By Mode");
  server.send(200, "text/html",  Send_Index_HTML());
}

void handle_NotFound() {
  server.send(404, "text/plain", "URL End Point Not found");
}

void handle_start_survey() {
  display_info_lg("Starting Survey..");
  server.send(200, "text/html", Send_Start_Survey_HTML());
  // Testing surveying functionality
  bool response = HAM_GNSS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (request can take a long time)
}

void handle_view_survey_log() {
  display_info_lg("Survey log displaying on Web dashboard");
  server.send(200, "text/html", Send_Survey_Log_HTML());
}

void handle_gnss_info() {
  display_info_lg("GNSS Info displaying on Web dashboaed");
  server.send(200, "text/html", Send_GNSS_Info_HTML());
}



// Website HTML
String Send_Index_HTML() {
  String page = "<!DOCTYPE html> <html>\n";
  page += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  page += "<title>HAM-GNSS-Reciever</title>\n";
  page += "</head>\n";
  page += "<body>\n";
  page += "<h1 style=\"text-align: center; font-weight: bold;\">High Altitude Media GNSS Reciver</h1>\n";
  page += "<a href=\"start_survey\">Start Survey</a>\n";
  page += "<a href=\"view_survey_log\">View Survey Log</a>\n";
  page += "<a href=\"gnss_info\">GNSS Info</a>";
  page += "<h4 style=\"text-align: center;\">Latitude & Longitude Cordinates: </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS_L_Band.getLatitude();
  page += ", ";
  page += HAM_GNSS_L_Band.getLongitude();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">Fix Type : </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS_L_Band.getFixType();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">Heading : </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS_L_Band.getHeading();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">Mean Sea Level : </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS_L_Band.getMeanSeaLevel();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">Measurement rate : </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS_L_Band.getMeasurementRate();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">Elipsoid : </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS_L_Band.getElipsoid();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">Altitude : </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += "Altitude: ";
  page += HAM_GNSS_L_Band.getAltitude();
  page += ", Mean Sea Level Alitutude: ";
  page += HAM_GNSS_L_Band.getAltitudeMSL();
  page += "</p>\n";
  page += "<p style=\"margin-top:10px;\">Created By Zion Johnson </p> <a href=\"https://github.com/zion379\">Github Portfolio</a>\n";
  page += "</body>\n";
  page += "</html>\n";

  return page;
}

String Send_Start_Survey_HTML() {
  String page = "<!DOCTYPE html> <html>\n";
  page += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  page += "<title>HAM-GNSS-Reciever</title>\n";
  page += "</head>\n";
  page += "<body>\n";
  page += "<h1 style=\"text-align: center; font-weight: bold\">High Altitude Media GNSS Reciver</h1>\n";
  page += "<h3 style=\"text-align: center;\"> Start Survey </h3>\n";
  page += "<p style=\"text-align: center;\">This Functionality is still being created will be avaible soon.</p>\n";
  page += "<a href=\"/\">Home</a>\n";
  page += "<p>Latitude: <span id='latitude'> lat value </span> , Longitude: <span id='longitude'> long val</span></p>\n";
  page += "<p>Altitude : <span id='altitude'> altitude value </span></p>\n";
  page += "<p>Altitude above MSL : <span id='altitude_msl'> MSL Altitude Val</p>\n";
  page += "<button type='button' id='BTN_SEND_BACK'> Send info to esp32 </button>\n";
  page += Client_WebSocketJS(); // returns string that is valid HTML
  page += "</body>\n";
  page += "</html>\n";

  return page;
}

String Send_Survey_Log_HTML() {
  String page = "<!DOCTYPE html> <html>\n";
  page += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  page += "<title>HAM-GNSS-Reciever</title>\n";
  page += "</head>\n";
  page += "<body>\n";
  page += "<h1 style=\"text-align: center; font-weight: bold\">High Altitude Media GNSS Reciver</h1>\n";
  page += "<h3 style=\"text-align: center;\"> Survey Log </h3>\n";
  page += "<p style=\"text-align: center;\">This Functionality is still being created will be avaible soon.</p>\n";
  page += "<a href=\"/\">Home<\a>\n";
  page += "</body>\n";
  page += "</html>\n";

  return page;
}

String Send_GNSS_Info_HTML() {
  String page = "<!DOCTYPE html> <html>\n";
  page += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  page += "<title>HAM-GNSS-Reciever</title>\n";
  page += "</head>\n";
  page += "<body>\n";
  page += "<h1 style=\"text-align: center; font-weight: bold\">High Altitude Media GNSS Reciver</h1>\n";
  page += "<h3 style=\"text-align: center;\"> GNSS INFO </h3>\n";
  page += "<h4 style=\"text-align: center;\">GNSS Reciever Module: </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS.getModuleName();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">GNSS Reciever Firmware Version: </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS.getFirmwareVersionHigh();
  page += ".";
  page += HAM_GNSS.getFirmwareVersionLow();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">GNSS Reciever Protocal version : </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS.getProtocolVersionHigh();
  page += ".";
  page += HAM_GNSS.getProtocolVersionLow();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">Firmware Type: </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS.getFirmwareType();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">GNSS Reciever Module 2: </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS_L_Band.getModuleName();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">GNSS Reciever Firmware Version: </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS_L_Band.getFirmwareVersionHigh();
  page += ".";
  page += HAM_GNSS_L_Band.getFirmwareVersionLow();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">GNSS Reciever Protocal version : </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS_L_Band.getProtocolVersionHigh();
  page += ".";
  page += HAM_GNSS_L_Band.getProtocolVersionLow();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">Firmware Type: </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS_L_Band.getFirmwareType();
  page += "</p>\n";
  page += "<h4 style=\"text-align: center;\">Antenna Status: </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS_L_Band.getAntennaStatus();
  page += "</p>\n";
  page += "<a href=\"/\">Home<\a>\n";
  page += "</body>\n";
  page += "</html>\n";
  return page;
}
// Web scripts
String Client_WebSocketJS() {
  String script = "<script>\n";
  script += "var Socket;\n";
  script += "document.getElementById('BTN_SEND_BACK').addEventListener('click', button_send_back);\n";
  script += "function button_send_back() {\n";
  script += " var message = {message: 'wassup zion u created a web socket ', date: '10-22-2000'}; \n";
  script += " Socket.send(JSON.stringify(message));\n";
  script += "}\n";
  script += "function init() {\n";
  script += " Socket = new WebSocket('ws://' + window.location.hostname + ':81/');\n";
  script += " Socket.onmessage = function(event) { //callback func\n";
  script += " processCommand(event);\n";
  script += " };\n";
  script += "}\n";
  script += "function processCommand(event) {\n"; // update elements with data
  script += " var obj = JSON.parse(event.data)\n";
  script += " document.getElementById('latitude').innerHTML = obj.latitude;\n";
  script += " document.getElementById('longitude').innerHTML = obj.longitude;\n";
  script += " document.getElementById('altitude').innerHTML = obj.altitude;\n";
  script += " document.getElementById('altitude_msl').innerHTML = obj.altitude_msl;\n";
  script += "}\n";
  script += "window.onload = function(event) {\n";
  script += " init();\n";
  script += "}\n";
  script += "</script>\n";
  return script;
}


// Display functions
void display_info(String message) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(1);
  display.setTextWrap(true);
  display.print(message);
  display.display();
}
void display_add_info(String message) {
  display.print(message);
  display.display();
}

void display_info_lg(String message) {
  display.clearDisplay();
  display.setCursor(0,16);
  display.setTextSize(1);
  display.setTextColor(1);
  display.setTextWrap(true);
  display.print(message);
  display.display();
}

// Web socket functions
void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length) { // client number, type of event, payload pointer to array of unsigned integers, length of payload array
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.println("Client Disconnected");
    break;
  case WStype_CONNECTED:
    Serial.println("Client Connected"); // broadcast message to client via json object.
    break;
  case WStype_TEXT:
    DeserializationError error =  deserializeJson(json_doc_rx, payload);
    if(error) {
      Serial.println("deserializeJson() failed");
      return;
    }
    else {
      const char* message = json_doc_rx["message"];
      const char* date = json_doc_rx["date"];

      Serial.println("Message from client : " + String(message) + String(date));
    }
    break;
  }
}

// Surveying Functions

