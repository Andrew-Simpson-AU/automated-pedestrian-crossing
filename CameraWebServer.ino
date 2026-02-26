/**************************************************
 * Freenove ESP32 WROVER CAM - Camera Web Server *
 **************************************************/

#include "esp_camera.h"
#include <WiFi.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


// ===== Wi-Fi credentials =====
const char* ssid = "Sean’s iPhone";
const char* password = "passsword";

// ===== Camera model =====
#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_AI_THINKER


// ===== Manual pin definitions for Freenove ESP32 WROVER CAM =====
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    21
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      19
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      5
#define Y2_GPIO_NUM      4
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

// ===== Camera config =====
camera_config_t config;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Freenove ESP32-WROVER Camera...");

  // ===== Configure camera =====
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Set frame size and buffers
  config.frame_size = FRAMESIZE_QVGA;  // 320x240
  config.jpeg_quality = 12;
  config.fb_count = 2;

  // ===== Init camera =====
  if(psramFound()){
    Serial.println("PSRAM found and enabled.");
  } else {
    Serial.println("Warning: PSRAM not found! Camera may fail.");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }
  Serial.println("Camera init OK!");

  // ===== Connect to Wi-Fi =====
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.println("");
  Serial.print("Wi-Fi connected! IP Address: ");
  Serial.println(WiFi.localIP());

  // ===== Start camera server =====
  startCameraServer();
  Serial.println("Camera ready! Access stream in your browser.");
}

// ===== Minimal camera web server =====
#include <WebServer.h>
WebServer server(80);

void loop() {
  server.handleClient();  // Add this line
  delay(1);
}

void handle_jpg_stream(void){
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  while (true) {
    camera_fb_t * fb = esp_camera_fb_get();
    if(!fb){
      Serial.println("Camera capture failed");
      continue;
    }
    client.print("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ");
    client.print(fb->len);
    client.print("\r\n\r\n");
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    esp_camera_fb_return(fb);
    if(!client.connected()) break;
  }
}

void startCameraServer(){
  server.on("/", HTTP_GET, [](){
    server.send(200, "text/html",
      "<html><body><h1>ESP32 Camera Stream</h1>"
      "<img src='/stream'>"
      "</body></html>");
  });

  server.on("/stream", HTTP_GET, handle_jpg_stream);

  server.begin();
}