/* 'Rain Rain Go Away' Arduino project
    Components:
                  - ESP32-CAM
                  - BMP280 sensor
                  - Rain sensor
                  - Breadboard
                  - Some jumper wires

    Libraries:
                  - ArduinoJson library
                  - Universal Telegram Bot library (https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot)
                  - BMP280_DEV library by Martin Lindupp

    Created on 24 September 2022 by c010blind3ngineer
*/

/*
  Base code by Rui Santos
  His complete project details at https://RandomNerdTutorials.com/esp32-cam-shield-pcb-telegram/

  Project created using Brian Lough's Universal Telegram Bot Library: https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <BMP280_DEV.h>


// Replace with your network credentials
const char* ssid = "**********";
const char* password = "**********";

// Use @myidbot to find out the chat ID of an individual or a group
// Also note that you need to click "start" on a bot before it can
// message you
String chatID = "**********";

// Initialize Telegram BOT
String BOTtoken = "**********:****************-****************";

bool sendPhoto = false;

WiFiClientSecure clientTCP;

UniversalTelegramBot bot(BOTtoken, clientTCP);

//CAMERA_MODEL_AI_THINKER
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

#define FLASH_LED_PIN 4
bool flashState = LOW;

// Rain Sensor
bool rainDetected = false;

// Define I2C Pins for BMP280
#define I2C_SDA 14
#define I2C_SCL 15
BMP280_DEV bmp(I2C_SDA, I2C_SCL);


float temperature, pressure, altitude;

int botRequestDelay = 1000;   // mean time between scan messages
long lastTimeBotRan;     // last time messages' scan has been done

void handleNewMessages(int numNewMessages);
String sendPhotoTelegram();

// Get BMP280 sensor readings and return them as a String variable
String getReadings() {
  String message = "Temperature: " + String(temperature) + " ºC \n";
  message += "Pressure: " + String(pressure) + " Pa \n";
  message += "Altitude: " + String(altitude) + " m \n";
  return message;
}

// Indicates when rain is detected
static void IRAM_ATTR detectsMovement(void * arg) {
  Serial.println("RAIN DETECTED!!!");
  rainDetected = true;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, flashState);

  Wire.begin(I2C_SDA, I2C_SCL);

  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

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
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  // Drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA

  // Rain Sensor mode INPUT_PULLUP
  //err = gpio_install_isr_service(0);
  err = gpio_isr_handler_add(GPIO_NUM_13, &detectsMovement, (void *) 13);
  if (err != ESP_OK) {
    Serial.printf("handler add failed with error 0x%x \r\n", err);
  }
  err = gpio_set_intr_type(GPIO_NUM_13, GPIO_INTR_POSEDGE);
  if (err != ESP_OK) {
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  }

  Serial.println("Testing BMP280...");
  Serial.println("");
  if (!bmp.begin(0x76)) {
    Serial.print("Oops...something went wrong... please check your BMP280 sensor");
    while (1);
  }
  bmp.setTimeStandby(TIME_STANDBY_2000MS);     // Set the standby time to 2 seconds
  bmp.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE

  Serial.print("Bot is awake");
  bot.sendMessage(chatID, "Bot is awake", "");
}

void loop() {
  // BMP280 readings need to be on loop to get accurate readings
  bmp.getMeasurements(temperature, pressure, altitude);

  if (sendPhoto) {
    Serial.println("Preparing photo");
    sendPhotoTelegram();
    sendPhoto = false;
  }

  if (rainDetected) {
    bot.sendMessage(chatID, "It's raining!", "");
    Serial.println("Rain rain go away!!");
    sendPhotoTelegram();
    rainDetected = false;
  }

  if (millis() > lastTimeBotRan + botRequestDelay) {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }
}

String sendPhotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }

  Serial.println("Connect to " + String(myDomain));

  if (clientTCP.connect(myDomain, 443)) {
    Serial.println("Connection successful");

    String head = "--c010blind3ngineer\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + chatID + "\r\n--c010blind3ngineer\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--c010blind3ngineer--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;

    clientTCP.println("POST /bot" + BOTtoken + "/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=c010blind3ngineer");
    clientTCP.println();
    clientTCP.print(head);

    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        clientTCP.write(fbBuf, remainder);
      }
    }

    clientTCP.print(tail);

    esp_camera_fb_return(fb);

    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;

    while ((startTimer + waitTime) > millis()) {
      Serial.print(".");
      delay(100);
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state == true) getBody += String(c);
        if (c == '\n') {
          if (getAll.length() == 0) state = true;
          getAll = "";
        }
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length() > 0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connected to api.telegram.org failed.";
    Serial.println("Connected to api.telegram.org failed.");
  }
  return getBody;
}

void handleNewMessages(int numNewMessages) {
  Serial.print("Handle New Messages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {
    // Chat id of the requester
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != chatID) {
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }

    // Print the received message
    String text = bot.messages[i].text;
    Serial.println(text);

    String fromName = bot.messages[i].from_name;

    if (text == "/flash") {
      flashState = !flashState;
      digitalWrite(FLASH_LED_PIN, flashState);
    }
    if (text == "/photo") {
      sendPhoto = true;
      Serial.println("New photo  request");
    }
    if (text == "/readings") {
      String readings = getReadings();
      bot.sendMessage(chatID, readings, "");
      Serial.print(temperature);
      Serial.print(F("*C    "));
      Serial.print(pressure);
      Serial.print(F("hPa   "));
      Serial.print(altitude);
      Serial.print(F("m"));
    }
    if (text == "/start") {
      String welcome = "Welcome to the ESP32-CAM Telegram bot.\n";
      welcome += "/photo : takes a new photo\n";
      welcome += "/flash : toggle flash LED\n";
      welcome += "/readings : request sensor readings\n\n";
      welcome += "You'll receive a photo whenever it starts to rain. \n";
      bot.sendMessage(chatID, welcome, "Markdown");
    }
  }
}
