/*
  Board: AI-Thinker ESP32-CAM or ESP-EYE
  Compile as:
   ESP32 Dev Module
   CPU Freq: 240
   Flash Freq: 80
   Flash mode: QIO
   Flash Size: 4Mb
   Partrition: Minimal SPIFFS
   PSRAM: Enabled
*/

// ESP32 has two cores: APPlication core and PROcess core (the one that runs ESP32 SDK stack)
#define APP_CPU 1
#define PRO_CPU 0

#include "esp_camera.h"
#include "ov2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

#include <esp_bt.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

#include <WebSocketsServer.h>
#include <ESP32Servo.h>

#define CAMERA_MODEL_AI_THINKER

#define MAX_CLIENTS 5

#define DUMMY_SERVO1_PIN 12     //We need to create 2 dummy servos.
#define DUMMY_SERVO2_PIN 13     //So that ESP32Servo library does not interfere with pwm channel and timer used by esp32 camera.

#define SERVO1_PIN 14
#define BELL_PIN 15

#define PINTU_PIN 2

#define POS_TUTUP 90
#define POS_BUKA 0

#include "camera_pins.h"

#define SSID1 "ROUTE_545"
#define PWD1 "satudualima"

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
Servo dummyServo1;
Servo dummyServo2;
Servo pintuServo;

unsigned long prevTutup = 0;
const unsigned long delayTutup = 2000;

int prevBell = HIGH;

enum Status {
  Tutup,
  Buka,
  Terbuka
};

Status kondisi = Tutup;

// ===== rtos task handles =========================
// Streaming is implemented with 3 tasks:
TaskHandle_t tMjpeg;   // handles client connections to the webserver
TaskHandle_t tCam;     // handles getting picture frames from the camera and storing them locally

uint8_t       noActiveClients;       // number of active clients

// frameSync semaphore is used to prevent streaming buffer as it is replaced with the next frame
SemaphoreHandle_t frameSync = NULL;

// We will try to achieve 24 FPS frame rate
const int FPS = 15;

// We will handle web client requests every 100 ms (10 Hz)
const int WSINTERVAL = 100;


// ======== Server Connection Handler Task ==========================
void mjpegCB(void* pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(WSINTERVAL);

  // Creating frame synchronization semaphore and initializing it
  frameSync = xSemaphoreCreateBinary();
  xSemaphoreGive( frameSync );

  //=== setup section  ==================

  //  Creating RTOS task for grabbing frames from the camera
  xTaskCreatePinnedToCore(
    camCB,        // callback
    "cam",        // name
    4 * 1024,       // stack size
    NULL,         // parameters
    2,            // priority
    &tCam,        // RTOS task handle
    PRO_CPU);     // core

  //  Registering webserver handling routines
  server.on("/1", HTTP_GET, handleJPGSstream);
  server.onNotFound(handleNotFound);

  //  Starting webserver
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  noActiveClients = 0;

  Serial.printf("\nmjpegCB: free heap (start)  : %d\n", ESP.getFreeHeap());
  //=== loop() section  ===================
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    server.handleClient();
    webSocket.loop();

    //  After every server client handling request, we let other tasks run and then pause
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


// Current frame information
volatile uint32_t frameNumber;
volatile size_t   camSize;    // size of the current frame, byte
volatile char*    camBuf;      // pointer to the current frame


// ==== RTOS task to grab frames from the camera =========================
void camCB(void* pvParameters) {

  TickType_t xLastWakeTime;

  //  A running interval associated with currently desired frame rate
  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / FPS);

  //  Pointers to the 2 frames, their respective sizes and index of the current frame
  char* fbs[2] = { NULL, NULL };
  size_t fSize[2] = { 0, 0 };
  int ifb = 0;
  frameNumber = 0;

  //=== loop() section  ===================
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {

    //  Grab a frame from the camera and query its size
    camera_fb_t* fb = NULL;

    fb = esp_camera_fb_get();
    size_t s = fb->len;

    //  If frame size is more that we have previously allocated - request  125% of the current frame space
    if (s > fSize[ifb]) {
      fSize[ifb] = s + s;
      fbs[ifb] = allocateMemory(fbs[ifb], fSize[ifb]);
    }

    //  Copy current frame into local buffer
    char* b = (char *)fb->buf;
    memcpy(fbs[ifb], b, s);
    esp_camera_fb_return(fb);

    //  Let other tasks run and wait until the end of the current frame rate interval (if any time left)
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    //  Only switch frames around if no frame is currently being streamed to a client
    //  Wait on a semaphore until client operation completes
    //    xSemaphoreTake( frameSync, portMAX_DELAY );

    //  Do not allow frame copying while switching the current frame
    xSemaphoreTake( frameSync, xFrequency );
    camBuf = fbs[ifb];
    camSize = s;
    ifb++;
    ifb &= 1;  // this should produce 1, 0, 1, 0, 1 ... sequence
    frameNumber++;

    //  Let anyone waiting for a frame know that the frame is ready
    xSemaphoreGive( frameSync );

    //  Immediately let other (streaming) tasks run
    taskYIELD();

    //  If streaming task has suspended itself (no active clients to stream to)
    //  there is no need to grab frames from the camera. We can save some juice
    //  by suspedning the tasks
    if ( noActiveClients == 0 ) {
      Serial.printf("mjpegCB: free heap           : %d\n", ESP.getFreeHeap());
      Serial.printf("mjpegCB: min free heap       : %d\n", ESP.getMinFreeHeap());
      Serial.printf("mjpegCB: max alloc free heap : %d\n", ESP.getMaxAllocHeap());
      Serial.printf("mjpegCB: tCam stack wtrmark  : %d\n", uxTaskGetStackHighWaterMark(tCam));
      Serial.flush();
      vTaskSuspend(NULL);  // passing NULL means "suspend yourself"
    }
  }
}


// ==== Memory allocator that takes advantage of PSRAM if present =======================
char* allocateMemory(char* aPtr, size_t aSize) {

  //  Since current buffer is too smal, free it
  if (aPtr != NULL) free(aPtr);

  char* ptr = NULL;
  ptr = (char*) ps_malloc(aSize);

  // If the memory pointer is NULL, we were not able to allocate any memory, and that is a terminal condition.
  if (ptr == NULL) {
    Serial.println("Out of memory!");
    delay(5000);
    ESP.restart();
  }
  return ptr;
}


// ==== STREAMING ======================================================
const char HEADER[] = "HTTP/1.1 200 OK\r\n" \
                      "Access-Control-Allow-Origin: *\r\n" \
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);


struct streamInfo {
  uint32_t        frame;
  WiFiClient      client;
  TaskHandle_t    task;
  char*           buffer;
  size_t          len;
};

// ==== Handle connection request from clients ===============================
void handleJPGSstream(void)
{
  if ( noActiveClients >= MAX_CLIENTS ) return;
  Serial.printf("handleJPGSstream start: free heap  : %d\n", ESP.getFreeHeap());

  streamInfo* info = new streamInfo;

  info->frame = frameNumber - 1;
  info->client = server.client();
  info->buffer = NULL;
  info->len = 0;

  //  Creating task to push the stream to all connected clients
  int rc = xTaskCreatePinnedToCore(
             streamCB,
             "strmCB",
             3 * 1024,
             (void*) info,
             2,
             &info->task,
             APP_CPU);
  if ( rc != pdPASS ) {
    Serial.printf("handleJPGSstream: error creating RTOS task. rc = %d\n", rc);
    Serial.printf("handleJPGSstream: free heap  : %d\n", ESP.getFreeHeap());
    //    Serial.printf("stk high wm: %d\n", uxTaskGetStackHighWaterMark(tSend));
    delete info;
  }

  noActiveClients++;

  // Wake up streaming tasks, if they were previously suspended:
  if ( eTaskGetState( tCam ) == eSuspended ) vTaskResume( tCam );
}


// ==== Actually stream content to all connected clients ========================
void streamCB(void * pvParameters) {
  char buf[16];
  TickType_t xLastWakeTime;
  TickType_t xFrequency;

  streamInfo* info = (streamInfo*) pvParameters;

  if ( info == NULL ) {
    Serial.println("streamCB: a NULL pointer passed");
  }
  //  Immediately send this client a header
  info->client.write(HEADER, hdrLen);
  info->client.write(BOUNDARY, bdrLen);
  taskYIELD();

  xLastWakeTime = xTaskGetTickCount();
  xFrequency = pdMS_TO_TICKS(1000 / FPS);

  for (;;) {
    //  Only bother to send anything if there is someone watching
    if ( info->client.connected() ) {

      if ( info->frame != frameNumber) {
        xSemaphoreTake( frameSync, portMAX_DELAY );
        if ( info->buffer == NULL ) {
          info->buffer = allocateMemory (info->buffer, camSize);
          info->len = camSize;
        }
        else {
          if ( camSize > info->len ) {
            info->buffer = allocateMemory (info->buffer, camSize);
            info->len = camSize;
          }
        }
        memcpy(info->buffer, (const void*) camBuf, info->len);
        xSemaphoreGive( frameSync );
        taskYIELD();

        info->frame = frameNumber;
        info->client.write(CTNTTYPE, cntLen);
        sprintf(buf, "%d\r\n\r\n", info->len);
        info->client.write(buf, strlen(buf));
        info->client.write((char*) info->buffer, (size_t)info->len);
        info->client.write(BOUNDARY, bdrLen);
        info->client.flush();
      }
    }
    else {
      //  client disconnected - clean up.
      noActiveClients--;
      Serial.printf("streamCB: Stream Task stack wtrmark  : %d\n", uxTaskGetStackHighWaterMark(info->task));
      Serial.flush();
      info->client.flush();
      info->client.stop();
      if ( info->buffer ) {
        free( info->buffer );
        info->buffer = NULL;
      }
      delete info;
      info = NULL;
      vTaskDelete(NULL);
    }
    //  Let other tasks run after serving every client
    taskYIELD();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ==== Handle invalid URL requests ============================================
void handleNotFound()
{
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text / plain", message);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;

    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      int readServo = pintuServo.read();
      if (readServo == POS_BUKA){
        webSocket.sendTXT(num, "pintu_buka");
      }else{
        webSocket.sendTXT(num, "pintu_tutup");
      }
      break;
    }

    case WStype_TEXT:
      Serial.printf("[%u] Received text: %s\n", num, payload);
      if (strcmp((const char*)payload, "tutup") == 0) {
        if (kondisi == Terbuka){
          Serial.println(F("pintu masih terbuka"));
          webSocket.sendTXT(num, "pintu_terbuka");
          return;
        }
        Serial.println("tutup pintu");
        tutup_pintu();
      }else if (strcmp((const char*)payload, "buka") == 0){
        if (kondisi == Terbuka){
          Serial.println(F("pintu masih terbuka"));
          webSocket.sendTXT(num, "pintu_terbuka");
          return;
        }
        Serial.println("buka pintu");
        buka_pintu();
      }
      break;

    default:
      break;
  }
}


void tutup_pintu()
{
  pintuServo.write(POS_TUTUP);
  webSocket.broadcastTXT("pintu_tutup");
  kondisi = Tutup;
}

void buka_pintu()
{
  pintuServo.write(POS_BUKA);
  webSocket.broadcastTXT("pintu_buka");
  kondisi = Buka;
}


// ==== SETUP method ==================================================================
void setup()
{
  // Setup Serial connection:
  Serial.begin(115200);
  delay(1000); // wait for a second to let Serial connect
  Serial.printf("setup: free heap  : %d\n", ESP.getFreeHeap());

  dummyServo1.attach(DUMMY_SERVO1_PIN);
  dummyServo2.attach(DUMMY_SERVO2_PIN);  

  pintuServo.attach(SERVO1_PIN);

  pinMode(BELL_PIN, INPUT_PULLUP);

  //Configure the camera
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
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;

  pinMode(PINTU_PIN, INPUT_PULLUP);;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Error initializing the camera");
    delay(10000);
    ESP.restart();
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, false);

  //  Configure and connect to WiFi
  IPAddress ip;

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID1, PWD1);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.println("");
  Serial.print("Stream Link: http://");
  Serial.print(ip);
  Serial.println("/mjpeg/1");

  buka_pintu();
  tutup_pintu();

  // Start mainstreaming RTOS task
  xTaskCreatePinnedToCore(
    mjpegCB,
    "mjpeg",
    4 * 1024,
    NULL,
    2,
    &tMjpeg,
    APP_CPU);

  Serial.printf("setup complete: free heap  : %d\n", ESP.getFreeHeap());
}

void loop() {
  switch(kondisi){
    case Buka:
      if (digitalRead(PINTU_PIN)){
        kondisi = Terbuka;
      }
      break;
    case Terbuka:
      if (!digitalRead(PINTU_PIN)){
        if (millis() - prevTutup >= delayTutup){
          tutup_pintu();
        }
      }else{
        prevTutup = millis();
      }
      break;
    default:
      break;
  }
  if (btnSingleClick(BELL_PIN, prevBell)){
    webSocket.broadcastTXT("bell");
    Serial.println("bell di tekan");
  }
  vTaskDelay(100);
}

bool btnSingleClick(int pin, int& prev){
  if (digitalRead(pin) == LOW && prev == HIGH){
    prev=LOW;
    return true;
  }else if(digitalRead(pin) == HIGH){
    prev=HIGH;
  }
  return false;
}
