#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <SD.h>

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

// SD Card Setup
void listFiles(const char *dirName);
String listFiles_HTML(const char *dirName);

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
void handle_device_files();

// HTML Pages Functions
String Send_Index_HTML();
String Send_Start_Survey_HTML();
String Send_Survey_Log_HTML();
String Send_GNSS_Info_HTML();
String Send_Device_Files_HTML();

// Web Script functions
String Client_WebSocketJS();

// CSS functions
String WebsiteBaseCSS();

//Web Socket Functions
void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length);

// surveying functions
void start_survey_observation();
void handle_survey_observation_in_progress();
void stop_survey_observation();

// surveying vars
bool survey_in_progress = false;
float survey_desired_accuracy = 6.00; // value is in meters

// surveying callback functions
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct);
void newRAWX(UBX_RXM_RAWX_data_t *ubxDataStruct);

// Web Communication json vars
StaticJsonDocument<400> json_doc_tx;
StaticJsonDocument<400> json_doc_rx;

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
  server.on("/device_files", handle_device_files);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  Serial.println("HTTP server started");

  //GNSS - ZED-F9P
  //Wire.begin();

  //HAM_GNSS.enableDebugging(Serial);
  while (HAM_GNSS.begin(0x42) == false) // Connect to the u-blox ZED-F9P module using Wire port
  {
    Serial.println("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing.");
    delay(500);
  }

  Serial.print("u-blox GNSS module connected");

  // NEO-D9S
  while (HAM_GNSS_L_Band.begin(0x43) == false) {
    Serial.println("u-blpx NEO-D9S not detected at default I2C address. Please check wiring.");
    delay(500);
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
  ok = HAM_GNSS.setI2CInput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_SPARTN | COM_TYPE_RTCM3); //Be sure SPARTN input is enabled
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
    
  //HAM_GNSS.setAutoPVTcallbackPtr(&printPVTdata); // Enable automativ NAV PVT messages with callback to printPVTdata
  //HAM_GNSS.setAutoRXMRAWXcallbackPtr(&newRAWX);

  HAM_GNSS.softwareResetGNSSOnly();

  delay(1000); // wait before intializing sd card.

  //SD Card
  Serial.print("Initializing SD Card...");
  if(!SD.begin(0, SPI, 10000000)) {
    Serial.println("SD Card initialization failed!");
    return;
  }
  else {
    Serial.println("SD Card initialized :)");
    listFiles("/");
  }
}

// testing
int interval = 1000; // 1 sec
unsigned long previousMillis = 0;

int ublox_msg_check_interval = 250; // 250ms
unsigned long ublox_previousMillis = 0;

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

  if ((now - ublox_previousMillis > ublox_msg_check_interval)) {
    HAM_GNSS.checkUblox();
    HAM_GNSS.checkCallbacks();
    ublox_previousMillis = now;
  }

  handle_survey_observation_in_progress();

  
  if((now - previousMillis > interval) && !survey_in_progress) { // print values while not in active survey
    // Testing Request Poll Position
    if (HAM_GNSS.getPVT() == true && webSocket.connectedClients() != 0) 
    {
      String JsonString ="";
      JsonObject object = json_doc_tx.to<JsonObject>(); 

      float_t latitude = HAM_GNSS.getLatitude() / float_t(10000000);
      float_t longitude = HAM_GNSS.getLongitude() / float_t(10000000);
      int32_t altitude = HAM_GNSS.getAltitude();
      int32_t altitude_msl = HAM_GNSS.getAltitudeMSL();

      object["latitude"] = String(latitude);
      object["longitude"] = String(longitude);
      object["altitude"] = String(altitude);
      object["altitude_msl"] = String(altitude_msl);

      serializeJson(object, JsonString);
      //Serial.println(JsonString);
      webSocket.broadcastTXT(JsonString);
      previousMillis = now;

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
  display_info_lg("Survey log displaying on web dash");
  server.send(200, "text/html", Send_Survey_Log_HTML());
}

void handle_gnss_info() {
  display_info_lg("GNSS Info displaying on web dash");
  server.send(200, "text/html", Send_GNSS_Info_HTML());
}

void handle_device_files() {
  display_info_lg("Device files displaying on web dash");
  server.send(200, "text/html", Send_Device_Files_HTML());
}



// Website HTML
String Send_Index_HTML() {
  String page = "<!DOCTYPE html> <html>\n";
  page += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  page += "<title>HAM-GNSS-Reciever</title>\n";
  page += "</head>\n";
  page += "<body>\n";
  page += WebsiteBaseCSS();
  page += "<h1 style=\"text-align: center; font-weight: bold;\">High Altitude Media GNSS Reciver</h1>\n";
  page += "<a href=\"start_survey\" class=\"button-link\">Start Survey</a>\n";
  page += "<a href=\"view_survey_log\" class=\"button-link\">View Survey Log</a>\n";
  page += "<a href=\"gnss_info\" class=\"button-link\">GNSS Info</a>";
  page += "<a href=\"device_files\" class=\"button-link\">Device Files</a>\n";
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
  page += "<p style=\"margin-top:10px;\">Created By Zion Johnson </p> <a href=\"https://github.com/zion379\" class=\"button-link\">Github Portfolio</a>\n";
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
  page += WebsiteBaseCSS();
  page += "<h1 style=\"text-align: center; font-weight: bold\">High Altitude Media GNSS Reciver</h1>\n";
  page += "<h3 style=\"text-align: center;\"> Start Survey </h3>\n";
  page += "<h4 style=\"text-align: center;\"> Survey Status: <span id='survey_status'>inactive</span> </h4>\n";
  page += "<h4 id='survey_msg' style='display: none; text-align: center;'></h4>\n";
  page += "<h4 id='survey_time' style='display: none; text-align: center;'></h4>\n";
  page += "<h4 id='survey_accuracy' style='display: none; text-align: center;'></h4>\n";
  page += "<h5 id='survey_desired_accuracy' style='display: none; text-align: center; font-weight: bold;'></h5>\n";
  page += "<label id='accuracy_input_label' for='target_accuracy_input'>Enter Target Accuracy:</label>\n";
  page += "<input type='number' id='target_accuracy_input' name='target_accuracy_input' min='0.08' max='100'>\n";
  page += "<button type='button' id='set_target_accuracy'> Set Target Accuracy </button>\n";
  page += "<h5 id='pos_accuracy' style='display: none; text-align: center; font-weight: bold;'></h5>\n";
  page += "<h5 id='vert_accuracy' style='display: none; text-align: center; font-weight: bold;'></h5>\n";
  page += "<h5 id='horz_accuracy' style='display: none; text-align: center; font-weight: bold;'></h5>\n";
  page += "<p style=\"text-align: center;\">Latitude: <span id='latitude'> lat value </span> , Longitude: <span id='longitude'> long val</span></p>\n";
  page += "<p style=\"text-align: center;\">Altitude : <span id='altitude'> altitude value </span></p>\n";
  page += "<p style=\"text-align: center;\">Altitude above MSL : <span id='altitude_msl'> MSL Altitude Val</p>\n";
  page += "<h5 id='mean_sea_lvl' style='display: none; text-align: center; font-weight: bold;'></h5>\n";
  page += "<button type='button' id='BTN_SEND_BACK' disabled> Send info to esp32 </button>\n";
  page += "<button type='button' id='start_survey' disabled> Start Survey </button>\n";
  page += "<button type='button' id='stop_survey' disabled> Stop Survey </button>\n";
  page += Client_WebSocketJS(); // returns string that is valid HTML
  page += "<a href=\"/\" class=\"button-link\">Home</a>\n";
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
  page += WebsiteBaseCSS();
  page += "<h1 style=\"text-align: center; font-weight: bold\">High Altitude Media GNSS Reciver</h1>\n";
  page += "<h3 style=\"text-align: center;\"> Survey Log </h3>\n";
  page += "<p style=\"text-align: center;\">This Functionality is still being created will be avaible soon.</p>\n";
  page += "<a href=\"/\" class=\"button-link\">Home<\a>\n";
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
  page += WebsiteBaseCSS();
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
  page += "<h4 style=\"text-align: center;\">Antenna Heading: </h4>\n";
  page += "<p style=\"text-align: center;\">";
  page += HAM_GNSS_L_Band.getHeading();
  page += "</p>\n";
  page += "<a href=\"/\" class=\"button-link\" >Home</a>\n";
  page += "</body>\n";
  page += "</html>\n";
  return page;
}

String Send_Device_Files_HTML() {
  String page = "<!DOCTYPE html> <html>\n";
  page += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  page += "<title>HAM-GNSS-Reciever</title>\n";
  page += "</head>\n";
  page += "<body>\n";
  page += WebsiteBaseCSS();
  page += "<h1 style=\"text-align: center; font-weight: bold\">High Altitude Media GNSS Reciver</h1>\n";
  page += "<h3 style=\"text-align: center;\"> Device Files </h3>\n";
  page += "<h4 style=\"text-align: center;\">Folders on GNSS Reciever:</h4>\n";
  page += listFiles_HTML("/");
  page += "<a href=\"/\" class=\"button-link\">Home</a>\n";
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
  script += "\n";
  script += "document.getElementById('start_survey').addEventListener('click', start_survey);\n";
  script += "function start_survey() {\n";
  script += " var message = {survey: 'START'};\n";
  script += " Socket.send(JSON.stringify(message));\n";
  script += "}\n";
  script += "\n";
  script += "document.getElementById('stop_survey').addEventListener('click', stop_survey);\n";
  script += "function stop_survey() {\n";
  script += " var message = {survey: 'STOP'};\n";
  script += " Socket.send(JSON.stringify(message));\n";
  script += "}\n";
  script += "document.getElementById('set_target_accuracy').addEventListener('click', set_target_accuracy);\n";
  script += "function set_target_accuracy() {\n";
  script += " var target_accuracy_input = document.getElementById('target_accuracy_input')\n";
  script += " var message = {set_target_accuracy: target_accuracy_input.value};\n";
  script += " Socket.send(JSON.stringify(message));\n";
  script += "}\n";
  script += "\n";
  script += "function init() {\n";
  script += " Socket = new WebSocket('ws://' + window.location.hostname + ':81/');\n";
  script += " Socket.addEventListener('open', (event) => {\n";
  script += "   var sendMsg_btn = document.getElementById('BTN_SEND_BACK');\n";
  script += "   var start_survey_btn = document.getElementById('start_survey');\n";
  script += "   var stop_survey_btn = document.getElementById('stop_survey');\n";
  script += "   sendMsg_btn.disabled = false;\n";
  script += "   start_survey_btn.disabled = false;\n";
  script += "   stop_survey_btn.disabled = false;\n";
  script += " });\n";
  script += " Socket.addEventListener('close', (event) => {\n";
  script += "   var sendMsg_btn = document.getElementById('BTN_SEND_BACK');\n";
  script += "   var start_survey_btn = document.getElementById('start_survey');\n";
  script += "   var stop_survey_btn = document.getElementById('stop_survey');\n";
  script += "   sendMsg_btn.disabled = true;\n";
  script += "   start_survey_btn.disabled = true;\n";
  script += "   stop_survey_btn.disabled = true;\n";
  script += " });\n";
  script += " Socket.onmessage = function(event) { //callback func\n";
  script += " processCommand(event);\n";
  script += " };\n";
  script += "}\n";
  script += "function processCommand(event) {\n"; // update elements with data
  script += " var obj = JSON.parse(event.data)\n";
  script += " if (obj.latitude && obj.longitude && obj.altitude && obj.altitude_msl) {\n";
  script += "    document.getElementById('latitude').innerHTML = obj.latitude;\n";
  script += "    document.getElementById('longitude').innerHTML = obj.longitude;\n";
  script += "    document.getElementById('altitude').innerHTML = obj.altitude;\n";
  script += "    document.getElementById('altitude_msl').innerHTML = obj.altitude_msl;\n";
  script += "  }\n";
  script += "\n";
  script += " if (obj.survey_status) {\n";
  script += "   switch(obj.survey_status) {\n";
  script += "     case 'failed_to_get_status':\n";
  script += "       document.getElementById('survey_status').innerHTML = 'Failed to get survey status.';\n";
  script += "       break;\n";
  script += "     case 'in_progress':\n";
  script += "       document.getElementById('survey_status').innerHTML = 'Survey currently in progress.';\n";
  script += "       break;\n";
  script += "     case 'started':\n";
  script += "       document.getElementById('survey_status').innerHTML = 'Survey Started!';\n";
  script += "       document.getElementById('target_accuracy_input').style.display = 'none';\n";
  script += "       document.getElementById('set_target_accuracy').style.display = 'none';\n";
  script += "       document.getElementById('accuracy_input_label').style.display = 'none';\n";
  script += "       break;\n";
  script += "     case 'survey_start_failed':\n";
  script += "       document.getElementById('survey_status').innerHTML = 'Survey Start Failed.';\n";
  script += "       break;\n";
  script += "     case 'stopped':\n";
  script += "       document.getElementById('survey_status').innerHTML = 'Survey Stopped';\n";
  script += "       document.getElementById('target_accuracy_input').style.display = 'block';\n";
  script += "       document.getElementById('set_target_accuracy').style.display = 'block';\n";
  script += "       document.getElementById('accuracy_input_label').style.display = 'block';\n";
  script += "       break;\n";
  script += "   }\n";
  script += "  }\n";
  script += " if (obj.survey_msg) {\n";
  script += "   var survey_msg_el = document.getElementById('survey_msg');\n";
  script += "   survey_msg_el.style.display = 'block';\n";
  script += "   survey_msg_el.textContent = obj.survey_msg;\n";
  script += " } else {\n";
  script += "   var survey_msg_el = document.getElementById('survey_msg');\n";
  script += "   survey_msg_el.style.display = 'none';\n";
  script += " }\n";
  script += " if (obj.survey_time_elapsed) {\n";
  script += "   var survey_time_el = document.getElementById('survey_time');\n";
  script += "   survey_time_el.style.display = 'block';\n";
  script += "   survey_time_el.textContent = 'Time elapsed: ' + obj.survey_time_elapsed;\n";
  script += " } else {\n";
  script += "   var survey_time_el = document.getElementById('survey_time');\n";
  script += "   survey_time_el.style.display = 'none';\n";
  script += " }\n";
  script += " if (obj.survey_accuracy) {\n";
  script += "   var survey_accuracy_el = document.getElementById('survey_accuracy');\n";
  script += "   survey_accuracy_el.style.display = 'block';\n";
  script += "   survey_accuracy_el.textContent = 'Mean Accuracy: ' + obj.survey_accuracy;\n";
  script += " } else {\n";
  script += "   var survey_accuracy_el = document.getElementById('survey_accuracy');\n";
  script += "   survey_accuracy_el.style.display = 'none';\n";
  script += " }\n";
  script += " if (obj.survey_desired_accuracy) {\n";
  script += "   var desired_accuracy_el = document.getElementById('survey_desired_accuracy');\n";
  script += "   desired_accuracy_el.style.display = 'block';\n";
  script += "   desired_accuracy_el.textContent = 'Target Accuracy: ' + obj.survey_desired_accuracy;\n";
  script += " } else {\n";
  script += "   var desired_accuracy_el = document.getElementById('survey_desired_accuracy');\n";
  script += "   desired_accuracy_el.style.display = 'block';\n";
  script += " }\n";
  script += " if (obj.survey_lat && obj.survey_long) {\n";
  script += "   var survey_lat_el = document.getElementById('latitude');\n";
  script += "   var survey_long_el = document.getElementById('longitude');\n";
  script += "   survey_lat_el.innerHTML = obj.survey_lat;\n";
  script += "   survey_long_el.innerHTML = obj.survey_long;\n";
  script += " }\n";
  script += " if (obj.survey_altitude && obj.survey_altitude_msl && obj.survey_msl) {\n";
  script += "   var survey_altitude_el = document.getElementById('altitude');\n";
  script += "   var survey_altitude_msl_el = document.getElementById('altitude_msl');\n";
  script += "   var survey_msl_el = document.getElementById('mean_sea_lvl');\n"; // create this el
  script += "   survey_altitude_el.innerHTML = obj.survey_altitude;\n";
  script += "   survey_altitude_msl_el.innerHTML = obj.survey_altitude_msl;\n";
  script += "   survey_msl_el.textContent = 'Mean Sea Level ' + obj.survey_msl;\n";
  script += " }\n";
  script += " if (obj.survey_pos_accuracy && obj.survey_vertical_accuracy && obj.survey_horizontal_accuracy) {\n";
  script += "   var survey_pos_accuracy_el = document.getElementById('pos_accuracy');\n";
  script += "   var survey_vert_accuracy_el = document.getElementById('vert_accuracy');\n";
  script += "   var survey_horz_accuracy_el = document.getElementById('horz_accuracy');\n";
  script += "   survey_pos_accuracy_el.style.display = 'block';\n";
  script += "   survey_vert_accuracy_el.style.display = 'block';\n";
  script += "   survey_horz_accuracy_el.style.display = 'block';\n";
  script += "   survey_pos_accuracy_el.textContent = 'position accuracy: ' + obj.survey_pos_accuracy;\n";
  script += "   survey_vert_accuracy_el.textContent = 'vertical accuracy: ' + obj.survey_vertical_accuracy + '(mm)';\n";
  script += "   survey_horz_accuracy_el.textContent = 'horizontal accuracy: ' + obj.survey_horizontal_accuracy + '(mm)';\n";
  script += " } else {\n";
  script += "   var survey_pos_accuracy_el = document.getElementById('pos_accuracy');\n";
  script += "   var survey_vert_accuracy_el = document.getElementById('vert_accuracy');\n";
  script += "   var survey_horz_accuracy_el = document.getElementById('horz_accuracy');\n";
  script += "   survey_pos_accuracy_el.style.display = 'none';\n";
  script += "   survey_vert_accuracy_el.style.display = 'none';\n";
  script += "   survey_horz_accuracy_el.style.display = 'none';\n";
  script += " }\n";
  script += "}\n";
  script += "window.onload = function(event) {\n";
  script += " init();\n";
  script += "}\n";
  script += "</script>\n";
  return script;
}

// Web CSS
String WebsiteBaseCSS() {
  String CSS = "<style>\n";
  CSS += ".button-link {\n";
  CSS += "  display: inline-block;\n";
  CSS += "  padding: 10px 20px;\n";
  CSS += "  background-color: #4CAF50;\n";
  CSS += "  color: white;\n";
  CSS += "  text-align: center;\n";
  CSS += "  text-decoration: none;\n";
  CSS += "  font-size: 16px;\n";
  CSS += "  border-radius: 5px;\n";
  CSS += "  border: none;\n";
  CSS += "  cursor: pointer;\n";
  CSS += "}\n";
  CSS += "\n";
  CSS += ".button-link:hover {\n";
  CSS += "  background-color: #45A049;\n";
  CSS += "}\n";
  CSS += "</style>\n";

  return CSS;
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
    if (HAM_GNSS.getSurveyInValid() == false) {
      String jsonString = "";
      JsonObject object = json_doc_tx.to<JsonObject>();
      object["survey_status"] = "in_progress";
      serializeJson(object, jsonString);
      webSocket.broadcastTXT(jsonString);
    }
    break;
  case WStype_TEXT:
    DeserializationError error =  deserializeJson(json_doc_rx, payload);
    if(error) {
      Serial.println("deserializeJson() failed");
      return;
    }
    else {
      // functionality for test button
      if(json_doc_rx["message"] && json_doc_rx["date"]) {
        const char* message = json_doc_rx["message"];
        const char* date = json_doc_rx["date"];

        Serial.println("Message from client : " + String(message) + String(date));
      }

      if(json_doc_rx["survey"]) {
        if (json_doc_rx["survey"] == "START") {
          Serial.println("Start survey called");
          start_survey_observation();
        }
        else if (json_doc_rx["survey"] == "STOP") {
          // Call stop survey
          Serial.print("Stopping Survey");
          stop_survey_observation();
        }
      }

      if(json_doc_rx["set_target_accuracy"]) {
        Serial.print("Set new target accuracy to: ");
        float new_accuracy_val = json_doc_rx["set_target_accuracy"];
        Serial.println(new_accuracy_val);
        survey_desired_accuracy = new_accuracy_val;
      }
    }
    break;
  }
}

// SD Card Functions
void listFiles(const char *dirName) {
  File root = SD.open(dirName);

  if(!root) {
    Serial.println("Failed to open directory");
    return;
  }

  Serial.println("Files in root directory:");

  while (File entry = root.openNextFile()) {
    Serial.println(entry.name());
    entry.close();
  }

  root.close();
}

String listFiles_HTML(const char *dirName) {
  File root = SD.open(dirName);
  String directory_files = "";

  if(!root) {
    Serial.println("Failed to open directory");
    display_info("Attempted to open file directory but failed.");
  }

  Serial.println("Files in root directory:");
  display_info("Displaying Files on Device in Web browser.");


  while (File entry = root.openNextFile()) {
    Serial.println(entry.name());
    directory_files += "<p style=\"text-align: center;\">";
    directory_files += entry.name();
    directory_files += "</p>\n";
    entry.close();
  }

  root.close();
  return directory_files;
}

// Surveying Functions
void start_survey_observation() {
  // dashboard send data json object
  String jsonString = "";
  JsonObject object = json_doc_tx.to<JsonObject>();

  display_info("starting survey observation.");
  // check if there already is a survey in progress
  bool response = HAM_GNSS.getSurveyStatus(2000);
  
  if (response ==  false) {
    Serial.println("Failed to get Survey In status.");
    display_info("Failed to get Survey In status.");
    // Send Survey Status
    object["survey_status"] = "failed_to_get_status";
    serializeJson(object, jsonString);
    webSocket.broadcastTXT(jsonString);
    while(1); // freeze
  }

  if (HAM_GNSS.getSurveyInActive() == false && survey_in_progress) {
    Serial.print("Survey already in progress.");
    display_info("Survey already in progress");
    // Send Survey Status
    object["survey_status"] = "in_progress";
    serializeJson(object, jsonString);
    webSocket.broadcastTXT(jsonString);
  }
  else {
    // Start Survey
    Serial.println("Initiating Survey");
    display_info("Initiating Survey");

    // Send Survey Status
    object["survey_status"] = "started";
    object["survey_desired_accuracy"] = survey_desired_accuracy;
    serializeJson(object, jsonString);
    webSocket.broadcastTXT(jsonString);

    // Start Survey
    response = HAM_GNSS.enableSurveyMode(60, survey_desired_accuracy, VAL_LAYER_RAM); // Enable Survey in, 60 secoonds 5m of accuracy Save setting in RAM layer only (not BBR)
    if(response == false) {
      display_info("Survey Start Failed");
      // Send Survey Status
      object["survey_status"] = "survey_start_failed";
      serializeJson(object, jsonString);
      webSocket.broadcastTXT(jsonString);
      while (1);
    }
    else {
      survey_in_progress = true; // set global survey variable
    }
  }
}

void handle_survey_observation_in_progress() {
  if(HAM_GNSS.getSurveyInValid() == false  && survey_in_progress) { // check if survey is in progress

    // dashboard send data json object
    String jsonString = "";
    JsonObject object = json_doc_tx.to<JsonObject>();

    bool response = HAM_GNSS.getSurveyStatus(2000); // Query module for SVIN status with 2000ms timeout (req can take a long time)
    delay(100); // give time for survey to start
    if(response == true) {
      object["survey_status"] = "in_progress";
      object["survey_time_elapsed"] = (String)HAM_GNSS.getSurveyInObservationTime();
      object["survey_accuracy"] = (String)HAM_GNSS.getSurveyInMeanAccuracy();
      object["survey_lat"] = (String)HAM_GNSS.getHighResLatitude();
      object["survey_long"] = (String)HAM_GNSS.getHighResLongitude();
      object["survey_altitude"] = (String)HAM_GNSS.getAltitude(); // create js func from here down
      object["survey_altitude_msl"] = (String)HAM_GNSS.getAltitudeMSL();
      object["survey_msl"] = (String)HAM_GNSS.getMeanSeaLevel();
      object["survey_pos_accuracy"] = (String)HAM_GNSS.getPositionAccuracy();
      object["survey_vertical_accuracy"] = (String)HAM_GNSS.getVerticalAccuracy();
      object["survey_horizontal_accuracy"] = (String)HAM_GNSS.getHorizontalAccuracy();
      serializeJson(object, jsonString);
      webSocket.broadcastTXT(jsonString);

      display_info("Survey in Progress");
      display_add_info(" time: ");
      display_add_info((String)HAM_GNSS.getSurveyInObservationTime());
      display_add_info(" accuracy: " );
      display_add_info((String)HAM_GNSS.getSurveyInMeanAccuracy());
    } else {
      object["survey_status"] = "in_progress";
      object["survey_msg"] = "SVIN request failed";
      serializeJson(object, jsonString);
      webSocket.broadcastTXT(jsonString);

      display_info("SVIN request failed");
    }

  } else if(HAM_GNSS.getSurveyInValid() && survey_in_progress) { // survey is invalid

      // dashboard send data json object
      String jsonString = "";
      JsonObject object = json_doc_tx.to<JsonObject>();

      object["survey_status"] = "finished";
      object["survey_msg"] = "Transmitting RTCM";
      serializeJson(object, jsonString);
      webSocket.broadcastTXT(jsonString);

      display_info("Survey Finished Transmitting RTCM");
      stop_survey_observation();
    }
}

void stop_survey_observation() {
  // dashboard send data json object
    String jsonString = "";
    JsonObject object = json_doc_tx.to<JsonObject>();

  // check if survey is already going
  if(!HAM_GNSS.getSurveyInValid() || survey_in_progress) {
    bool response = HAM_GNSS.disableSurveyMode(); // disable survey mode.
    if(response) {
      display_info("Survey Observation Stopped.");
      object["survey_status"] = "stopped";
      object["survey_msg"] = "Survey Manually stopped";
      serializeJson(object, jsonString);
      webSocket.broadcastTXT(jsonString);
      survey_in_progress = false;
    } else {
      display_info("Attempted to stop survey. but failed");

      object["survey_status"] = "stop_failed";
      object["survey_msg"] = "Attempted to stop survey. but failded.";
      serializeJson(object, jsonString);
      webSocket.broadcastTXT(jsonString);
      survey_in_progress = false;
    }
  } else {
    display_info("Attempted to stop survey but there are no survey in progress.");

    object["survey_status"] = "stopped";
    object["survey_msg"] = "Attempted  to stop survey but there was no survey in progress.";
    serializeJson(object, jsonString);
    webSocket.broadcastTXT(jsonString);
    survey_in_progress = false;
  }
}

void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
    Serial.println();

    Serial.print(F("Time: ")); // Print the time
    uint8_t hms = ubxDataStruct->hour; // Print the hours
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = ubxDataStruct->min; // Print the minutes
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = ubxDataStruct->sec; // Print the seconds
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F("."));
    unsigned long millisecs = ubxDataStruct->iTOW % 1000; // Print the milliseconds
    if (millisecs < 100) Serial.print(F("0")); // Print the trailing zeros correctly
    if (millisecs < 10) Serial.print(F("0"));
    Serial.print(millisecs);

    long latitude = ubxDataStruct->lat; // Print the latitude
    Serial.print(F(" Lat: "));
    Serial.print(latitude);

    long longitude = ubxDataStruct->lon; // Print the longitude
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
    Serial.print(F(" Height above MSL: "));
    Serial.print(altitude);
    Serial.println(F(" (mm)"));
}

void newRAWX(UBX_RXM_RAWX_data_t *ubxDataStruct)
{
  Serial.println();

  Serial.print(F("New RAWX data received. It contains "));
  Serial.print(ubxDataStruct->header.numMeas); // Print numMeas (Number of measurements / blocks)
  Serial.println(F(" data blocks:"));

  for (uint8_t block = 0; block < ubxDataStruct->header.numMeas; block++) // For each block
  {
    Serial.print(F("GNSS ID: "));
    if (ubxDataStruct->blocks[block].gnssId < 100) Serial.print(F(" ")); // Align the gnssId
    if (ubxDataStruct->blocks[block].gnssId < 10) Serial.print(F(" ")); // Align the gnssId
    Serial.print(ubxDataStruct->blocks[block].gnssId);
    Serial.print(F("  SV ID: "));
    if (ubxDataStruct->blocks[block].svId < 100) Serial.print(F(" ")); // Align the svId
    if (ubxDataStruct->blocks[block].svId < 10) Serial.print(F(" ")); // Align the svId
    Serial.print(ubxDataStruct->blocks[block].svId);

    if (sizeof(double) == 8) // Check if our processor supports 64-bit double
    {
      // Convert prMes from uint8_t[8] to 64-bit double
      // prMes is little-endian
      double pseudorange;
      memcpy(&pseudorange, &ubxDataStruct->blocks[block].prMes, 8);
      Serial.print(F("  PR: "));
      Serial.print(pseudorange, 3);

      // Convert cpMes from uint8_t[8] to 64-bit double
      // cpMes is little-endian
      double carrierPhase;
      memcpy(&carrierPhase, &ubxDataStruct->blocks[block].cpMes, 8);
      Serial.print(F(" m  CP: "));
      Serial.print(carrierPhase, 3);
      Serial.print(F(" cycles"));
    }
    Serial.println();
  }
}

