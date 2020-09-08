#include "esp_camera.h"
#include "CommandHandler.h"
#include <WiFi.h>
#include<WiFiClient.h>
//EthernetClient 

#include <Ethernet.h>

//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
#define CAMERA_MODEL_M5STACK_WIDE
//#define CAMERA_MODEL_AI_THINKER
#define WIFI_MODEL_AP
#include "camera_pins.h"

//const char* ssid = "*********";
//const char* password = "*********";

const char* ssid = "cool";
const char* password = "12369874";
WiFiClient client;
//EthernetClient  client;
//char server[] = "127.0.4.1"; 
//char server[] = "192.168.4.1"; 
IPAddress server(127,0,0,1);
//IPAddress server(192,168,4,1);
void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  //s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_framesize(s, FRAMESIZE_XGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(WIFI_MODEL_AP)
  // You can remove the password parameter if you want the AP to be open.
  WiFi.mode(WIFI_AP);
  String Mac = WiFi.macAddress();
  String SSID = "Camera:"+ Mac;
  bool result = WiFi.softAP(SSID.c_str(), "", 0, 0);
  if (!result){
    Serial.println("AP Config failed.");
  } else
  {
    Serial.println("AP Config Success. AP NAME: " + String(SSID));
  }
  //WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  WiFi.begin();

  startCameraServer();
#else
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
#endif
  CommandHandler.begin();

  
  //IPAddress server(106,52,170,30);
 /*
  if (client.connect(server, 80)) {
    //client.println("http://192.168.4.1:80/capture");
    //client.println("Host: 106.52.170.30");
    //client.println("User-Agent: ArduinoWiFi/1.1");
    //client.println("Connection: close");
    //client.println();
    client.println("GET /capture HTTP/1.1");
    client.println("Host: 127.0.0.1");
    client.println("User-Agent: ArduinoWiFi/1.1");
    client.println("Connection: close");
    client.println();

     Serial.println("connection OK");
  } else {
    // if you couldn't make a connection:
    Serial.println("connection failed");
  }
 */
}

void loop() {
  // put your main code here, to run repeatedly:
  //client.println("http://192.168.4.1:80/capture");
  //client.println("http://127.0.0.1:80/capture");
  
  //delay(1000);
  CommandHandler.Serial2Event();
}
