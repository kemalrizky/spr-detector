#include <Arduino.h>

#include <WiFi.h>
#include <WiFiAP.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"

// OV2640 camera module pins (CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// A4988 Motor Driver Pins
#define DIR 15
#define STEP 13
#define EN 12
// Infrared Receiver Pin
#define IR 14
// Laser Module Pin
#define LASER 2
// Built-in Flash Pin
#define FLASHLIGHT 4

// ESP32-CAM Access Point Credentials
#define AP_SSID "SPR Sensor"
#define AP_PASS "12345678"

WiFiClient client;
WiFiServer server(80);

uint8_t enhanceVal = 30;
uint16_t boundUp = 0;
uint16_t boundDown = 320;
uint32_t totalPixelVal = 0;
uint32_t brightData[6370];


String command = "";
bool commandReceived = false;

int calculateStep(int _stepMode, int _angle) {
  int _totalStep = round((_stepMode*64/11.25)*(_angle-30));
  return _totalStep;
}

void moveStepper(int _dir,int _step) {
  if(_dir == 1) {                     // clockwise (new)
    digitalWrite(DIR,HIGH);
  }
  else {                              // counter clockwise (new)
    digitalWrite(DIR,LOW);
  }
  delay(1);
  for(int j=0; j < _step; j++) {
    digitalWrite(STEP,HIGH);
    delayMicroseconds(5000);
    digitalWrite(STEP,LOW);
    delayMicroseconds(5000);
  }
}

void setRef() {
  while(!digitalRead(IR)) {
    // moveStepper(1,1);   // old
    moveStepper(0,1);     // new
    delay(10);
  }
}

void OV2640CameraConfig() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }
}

void capturePhoto(bool _isPicture,bool _isFullPicture) {
  camera_fb_t *fb = esp_camera_fb_get();          // access frame buffer
  
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  uint8_t *pixel = fb->buf;
  if(_isPicture){
    if(_isFullPicture){
      client.print("pic:");
      client.write(fb->buf,fb->len);
    }
    else{
      /*220,120,80,70*/
      for(uint32_t j = 0; j < fb->len; j++){
        client.print("picArray:");
        client.write(*(pixel+j));
      }
    }
    client.println(totalPixelVal);
    totalPixelVal = 0;
  }
  else{
    totalPixelVal = 0;
    for(uint32_t j = 0; j < fb->len; j++){
      if(j/320 >= boundUp && j/320 <= boundDown){
        uint32_t _brightVal = *(pixel+j);
        if(_brightVal >= enhanceVal){
          totalPixelVal += _brightVal;
        }
      }
    }
  }
  esp_camera_fb_return(fb);                      // release frame buffer memory
}

void initWiFiAP() {
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(1000);
  IPAddress local_ip = IPAddress (192, 168, 0, 198);
  IPAddress gateway = IPAddress (192, 168, 0, 198);
  IPAddress subnet = IPAddress (255, 255, 255, 0);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  delay(1000);
  // Turn-off the 'brownout detector'
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  // Camera and peripherals config
  OV2640CameraConfig();
  pinMode(EN,OUTPUT);
  pinMode(DIR,OUTPUT);
  pinMode(STEP,OUTPUT);
  pinMode(LASER,OUTPUT);
  pinMode(IR,INPUT);
  pinMode(FLASHLIGHT,OUTPUT);
  
  // Initialize peripherals
  digitalWrite(DIR,LOW);
  digitalWrite(STEP,LOW);
  digitalWrite(EN,HIGH);
  digitalWrite(LASER,LOW);

  // Initializing ESP32 as Wifi Access Point
  initWiFiAP();
  server.begin();

  for(int j=0; j<6370; j++){
    brightData[j]=0;
  }

  digitalWrite(FLASHLIGHT,HIGH);
  delay(100);
  digitalWrite(FLASHLIGHT,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  client = server.available();
  if(client) {
    String buffer = client.readString();
    command = buffer.substring(5,buffer.indexOf(" HTTP"));
    Serial.println(command);
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    //client.println("Connection: keep-alive");
    client.println("\n\rok");
    commandReceived = true;
  }

  if(commandReceived) {
    commandReceived = false;
    digitalWrite(EN,LOW);
    delay(1);
    
    if(command.substring(0,command.indexOf(':')) == "getPicture") {
      capturePhoto(true,true);
      command = "";
    }
    if(command.substring(0,command.indexOf(':')) == "laserOn") {
      digitalWrite(LASER,HIGH);
      client.println("Laser ON");
      command = "";
    }
    if(command.substring(0,command.indexOf(':')) == "laserOff") {
      digitalWrite(LASER,LOW);
      client.println("Laser OFF");
      command = "";
    }
    if(command.substring(0,command.indexOf(':')) == "moveStepper") {
      int _stepperDir = command.substring(command.indexOf(':')+1,command.indexOf(',')).toInt();
      int _stepperMoveCount = command.substring(command.indexOf(',')+1,command.indexOf(';')).toInt();
      moveStepper(_stepperDir,_stepperMoveCount);
      client.print("Stepper Moved ");client.print(_stepperMoveCount);client.print(" dir ");client.println(_stepperDir);
      command  = "";
    }
    if(command.substring(0,command.indexOf(':')) == "setReference") {
      setRef();
      client.println("Set Ref Done");
      command = "";
    }
    if(command.substring(0,command.indexOf(':')) == "enhance") {
      enhanceVal = command.substring(command.indexOf(':')+1,command.indexOf(';')).toInt();
      client.print("Set enhance value: ");client.println(enhanceVal);
      command = "";
    }
    if(command.substring(0,command.indexOf(':')) == "window") {
      boundUp = command.substring(command.indexOf(':')+1,command.indexOf(',')).toInt();
      boundDown = command.substring(command.indexOf(',')+1,command.indexOf(';')).toInt();
      client.print("Set window: ");client.print(boundUp);client.print(",");client.print(boundDown);
      command = "";
    }
    if(command.substring(0,command.indexOf(':')) == "getSPRData") {
      int stepMode = command.substring(command.indexOf(':')+1,command.indexOf(',')).toInt();
      int maxAngle = command.substring(command.indexOf(',')+1,command.indexOf(';')).toInt();
      client.print("Start procedure. Enhance val: "); client.print(enhanceVal);
      client.print(", bound up: "); client.print(boundUp); client.print(", bound down: "); client.println(boundDown);
      int _totalStep = calculateStep(stepMode, maxAngle);
      for(int j = 0; j<_totalStep; j++){
        capturePhoto(false,false);
        brightData[j] = totalPixelVal;
        // moveStepper(0,1);       // old
        moveStepper(1,1);       // new
      }
      for(int j = 0; j<_totalStep; j++){
        client.print(brightData[j]); client.print(",");
      }
      command = "";
    }

    digitalWrite(EN,HIGH);
    delay(1);
  }
}