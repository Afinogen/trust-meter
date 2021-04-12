#include <Arduino.h>
#include "HX711.h"
#include <EEPROM.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>

#include <Ticker.h>

#define VERSION "0.0.1"
#define EEPROM_START 0
#define STA_SSID_DEFAULT "CLIENTSSID"
#define STA_PASSWORD_DEFAULT "WiFinetKEY"
#define STRING_DELIMITER F(";")
#define MAX_REQUEST_LEN 20
#define DEFAULT_CALIBRATE_FACTOR 1

Stream *responseTo = &Serial;

// Scale Settings
#define MEASURE_START 1
#define MEASURE_WAIT 0
const int LOADCELL_DOUT_PIN = D3;
const int LOADCELL_SCK_PIN = D4;
float     conversion_rate    = 0.035274;                      // указываем коэффициент для перевода из унций в граммы
HX711 scale;

//ESC settings
#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000
#define SPEED_PIN D1
const int ESC_PIN = D2;
Ticker measureTick;
byte measureState = MEASURE_WAIT;
uint8_t throttle = 0;
Servo ESC;

//RPM settings
unsigned long rmpTimerStart = 0;
volatile uint16_t countTicks = 0;
uint16_t rpm = 0;

//WIFI settings
WiFiServer wifiServer(80);


//EEPROM settings
uint32_t memcrc; 
uint8_t *p_memcrc = (uint8_t*)&memcrc;

struct eeprom_data_t {
  float calibrationFactor;
  int minThrottle;
  int maxThrottle;
  char STAssid[17];
  char STApass[17];
} eeprom_data;

static PROGMEM uint32_t crc_table[16] = {
// static  prog_uint32_t crc_table[16] = {
  0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
  0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};


// ----------------------------------- crc_update -----------------------------------
unsigned long crc_update(unsigned long crc, byte data)
{
  byte tbl_idx;
  tbl_idx = crc ^ (data >> (0 * 4));
  crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
  tbl_idx = crc ^ (data >> (1 * 4));
  crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
  return crc;
}
// ----------------------------------- crc_byte -----------------------------------
unsigned long crc_byte(byte *b, int len)
{
  unsigned long crc = ~0L;
  uint16_t i;

  for (i = 0 ; i < len ; i++)
  {
    crc = crc_update(crc, *b++);
  }
  crc = ~crc;
  return crc;
}
// ----------------------------------- readSettingsESP -----------------------------------
void readSettingsESP()
{
  int i;
  uint32_t datacrc;
  byte eeprom_data_tmp[sizeof(eeprom_data)];

  EEPROM.begin(sizeof(eeprom_data) + sizeof(memcrc));

  for (i = EEPROM_START; i < EEPROM_START+sizeof(eeprom_data); i++)
  {
    eeprom_data_tmp[i] = EEPROM.read(i);
  }

  p_memcrc[0] = EEPROM.read(i++);
  p_memcrc[1] = EEPROM.read(i++);
  p_memcrc[2] = EEPROM.read(i++);
  p_memcrc[3] = EEPROM.read(i++);

  datacrc = crc_byte(eeprom_data_tmp, sizeof(eeprom_data_tmp));
  responseTo->println(memcrc);
  responseTo->println(datacrc);
  if (memcrc == datacrc)
  {
    memcpy(&eeprom_data, eeprom_data_tmp,  sizeof(eeprom_data));
  }
  else
  {
    eeprom_data.calibrationFactor = DEFAULT_CALIBRATE_FACTOR;
    eeprom_data.minThrottle = MIN_THROTTLE;
    eeprom_data.maxThrottle =  MAX_THROTTLE;
    strncpy(eeprom_data.STAssid, STA_SSID_DEFAULT, sizeof(STA_SSID_DEFAULT));
    strncpy(eeprom_data.STApass, STA_PASSWORD_DEFAULT, sizeof(STA_PASSWORD_DEFAULT));
  }
}
// ----------------------------------- writeSettingsESP -----------------------------------
void writeSettingsESP()
{
  int i;
  byte eeprom_data_tmp[sizeof(eeprom_data)];

  EEPROM.begin(sizeof(eeprom_data) + sizeof(memcrc));

  memcpy(eeprom_data_tmp, &eeprom_data, sizeof(eeprom_data));

  for (i = EEPROM_START; i < EEPROM_START+sizeof(eeprom_data); i++)
  {
    EEPROM.write(i, eeprom_data_tmp[i]);
  }
  memcrc = crc_byte(eeprom_data_tmp, sizeof(eeprom_data_tmp));
  responseTo->println(memcrc);

  EEPROM.write(i++, p_memcrc[0]);
  EEPROM.write(i++, p_memcrc[1]);
  EEPROM.write(i++, p_memcrc[2]);
  EEPROM.write(i++, p_memcrc[3]);

  EEPROM.commit();
}

char *getRequestPayload(char *data)
{
  char *value = new char[MAX_REQUEST_LEN]{'\0'};
  uint8_t pos = 0;
  for (size_t i = 0; i < MAX_REQUEST_LEN; i++)
  {
    if (data[i] == ';') {
      pos = i+1;
      break;
    }
  }
  if (pos == 0) {
    return value;
  }
  
  for (size_t i = pos; i < MAX_REQUEST_LEN; i++)
  {
    value[i-pos] = data[i];
  }

  return value;
}

void clearScale()
{
    scale.set_scale();                                                                    
    scale.tare(); 
}

void sendGramms()
{
  float units = scale.get_units();       
  float grams = units * conversion_rate;

  String response = "$17;";
  response += grams;
  responseTo->println(response); 
}

void calibrate(char *data) 
{
  char *value = getRequestPayload(data);
  float controlWeight = atof(value);
  delete value;
  
  scale.set_scale();
  float calibration_factor = scale.get_units(1) / (controlWeight / conversion_rate);
  scale.set_scale(calibration_factor);
  
  String response = "$16;";
  response += calibration_factor;
  responseTo->println(response); 
  sendGramms();  
}

void sendVersion()
{
  String response = F("$1;");
  response += VERSION;
  responseTo->println(response);
}

void sendSettings()
{
  String response = F("$2;");
  response += eeprom_data.calibrationFactor;
  response += STRING_DELIMITER;
  response += eeprom_data.STAssid;
  response += STRING_DELIMITER;
  response += eeprom_data.STApass;
  response += STRING_DELIMITER;
  response += eeprom_data.minThrottle;
  response += STRING_DELIMITER;
  response += eeprom_data.maxThrottle;
  
  responseTo->println(response);
}

void sendIp()
{
  String response = F("$8;");
    
  if (WiFi.status() == WL_CONNECTED) {
    response += WiFi.localIP().toString();
  }

  responseTo->println(response);
}

void getMeasure()
{
  float units = scale.get_units();       
  float grams = units * conversion_rate;   
  String response = F("$3;");                            
  response += grams;
  response += STRING_DELIMITER;
  response += throttle;
  response += STRING_DELIMITER;
  response += rpm;
  responseTo->println(response);
}

void firmwareUpdate() 
{
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

bool checkCommand(char *data, char *command)
{
  if (data[0] != '$') {
    return false;
  }

  char requestCommand[MAX_REQUEST_LEN]{'\0'};

  for (size_t i = 1; i < MAX_REQUEST_LEN; i++)
  {
    if (data[i] == ';') {
      break;
    }
    requestCommand[i-1] = data[i];
  }
  
  return strcmp(requestCommand,command) == 0;
}

void updateWifiSSId(char *data)
{
  char *value = getRequestPayload(data);
  strncpy(eeprom_data.STAssid, value, strlen(value));
  delete value;
}

void updateWifiPassword(char *data)
{
  char *value = getRequestPayload(data);
  strncpy(eeprom_data.STApass, value, strlen(value));
  delete value;
}

void setThrottle(char *data)
{
  char *value = getRequestPayload(data);
  throttle = atoi(value);
  ESC.write(map(throttle, 0, 100, 0, 180));
  String response = F("$9;");
  response += value;
  responseTo->println(response);
  delete value;
}

void setMinThrottle(char *data)
{
  char *value = getRequestPayload(data);
  eeprom_data.minThrottle = atoi(value);
  String response = F("$11;");
  response += value;
  responseTo->println(response);
  delete value;
}

void setMaxThrottle(char *data)
{
  char *value = getRequestPayload(data);
  eeprom_data.maxThrottle = atoi(value);
  String response = F("$12;");
  response += value;
  responseTo->println(response);
  delete value;
}

void setMeasureCorrection(char *data)
{
  char *value = getRequestPayload(data);
  eeprom_data.calibrationFactor = atof(value);
  String response = F("$13;");
  response += value;
  responseTo->println(response);
  delete value;
}

IRAM_ATTR void rpmInerrupt(void)
{
  countTicks++;
}

void sendRpm()
{
  if (measureState == MEASURE_WAIT) {
    return;
  }

  if (millis() - rmpTimerStart >= 1000) {
    rpm = (countTicks/7*60);
    rmpTimerStart = millis();
    countTicks = 0;
  }
}

void parseRequest(char *inData) {
  if (checkCommand(inData, (char *)"1")) {
    sendVersion();
  } else if (checkCommand(inData, (char *)"2")) {
    sendSettings();
  } else if (checkCommand(inData, (char *)"3")) {
    measureState = MEASURE_START;
    scale.set_scale();                                          // выполняем измерение значения без калибровочного коэффициента
    scale.tare();                                               // сбрасываем значения веса на датчике в 0
    scale.set_scale(eeprom_data.calibrationFactor);                        // устанавливаем калибровочный коэффициент
    attachInterrupt(digitalPinToInterrupt(SPEED_PIN), rpmInerrupt, RISING);
    measureTick.attach(0.5, getMeasure);
  } else if (checkCommand(inData, (char *)"4")) {
    detachInterrupt(SPEED_PIN);
    measureState = MEASURE_WAIT;
    measureTick.detach();
    responseTo->println(F("$4"));
  } else if (checkCommand(inData, (char *)"5")) {
    updateWifiSSId(inData);
  } else if (checkCommand(inData, (char *)"6")) {
    updateWifiPassword(inData);
  } else if (checkCommand(inData, (char *)"7")) {
    ESP.restart();
  } else if (checkCommand(inData, (char *)"8")) {
    sendIp();
  } else if (checkCommand(inData, (char *)"9")) {
    setThrottle(inData);
  } else if (checkCommand(inData, (char *)"10")) {
    writeSettingsESP();
  } else if (checkCommand(inData, (char *)"11")) {
    setMinThrottle(inData);
  } else if (checkCommand(inData, (char *)"12")) {
    setMaxThrottle(inData);
  } else if (checkCommand(inData, (char *)"13")) {
    setMeasureCorrection(inData);
  } else if (checkCommand(inData, (char *)"14")) {
    String response = "$14;";
    response += (countTicks/7*60);
    responseTo->println(response);
  } else if (checkCommand(inData, (char *)"15")) {
    clearScale();
  } else if (checkCommand(inData, (char *)"16")) {
    calibrate(inData);
  } else if (checkCommand(inData, (char *)"17")) {
    sendGramms();
  } else {
    responseTo->println(F("хз"));
    Serial.println(inData);
  }
}

void setup() {
  Serial.begin(115200);

  readSettingsESP();

  Serial.print("Scale: ");
  Serial.println(eeprom_data.calibrationFactor);
  Serial.print("Wifi SSID: ");
  Serial.println(eeprom_data.STAssid);
  Serial.print("Wifi password: ");
  Serial.println(eeprom_data.STApass);
  
  // Begin WiFi
  WiFi.mode(WIFI_STA);
  WiFi.hostname("Trust meter");
  WiFi.begin(eeprom_data.STAssid, eeprom_data.STApass);
 
  // Connecting to WiFi...
  Serial.print("Connecting to ");
  Serial.print(eeprom_data.STAssid);
  uint8_t countProbe = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
    Serial.print(".");
    if (countProbe == 60) {
      WiFi.disconnect();
      break;
    }
    countProbe++;
  }
 
  firmwareUpdate();
  // Connected to WiFi
  Serial.println();
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  if (WiFi.status() == WL_CONNECTED) {
    sendIp();
  }
  wifiServer.begin();

  ESC.attach(ESC_PIN, eeprom_data.minThrottle, eeprom_data.maxThrottle);
  ESC.write(throttle);

  pinMode(SPEED_PIN, INPUT_PULLUP);

  Serial.println("Initializing the scale");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale();                                          // выполняем измерение значения без калибровочного коэффициента
  scale.tare();                                               // сбрасываем значения веса на датчике в 0
  scale.set_scale(eeprom_data.calibrationFactor);                        // устанавливаем калибровочный коэффициент
  Serial.println(F("Setup finish"));
}

void loop() {
  ArduinoOTA.handle();
  WiFiClient client = wifiServer.available();
 
  if (client) {
    while (client.connected()) {
      sendRpm();
      while (client.available()>0) {
        responseTo = &client;
        char inData[20]{'\0'};
        client.readBytesUntil('\n', inData, 20);
        parseRequest(inData);
      }
      delay(10);
    }
    responseTo = &Serial;
    client.stop();
    // Serial.println("Client disconnected");
  }

  if (Serial.available() > 0) {
      responseTo = &Serial;
      char inData[20]{'\0'};
      Serial.readBytesUntil('\n', inData, 20);
      parseRequest(inData);
  }
  sendRpm();
}