#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <vector>

#define LIGHT_PIN 4

#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define STOP 0

#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

#define FORWARD 1
#define BACKWARD -1

const char* ssid = "iPhone hadi";
const char* password = "71357928";

// Node.js WebSocket server IP and port
const char* websocket_server_host = "172.20.10.5"; 
const int websocket_server_port = 8080;

// Initialize WebSocket client
WebSocketsClient webSocket;

// Motor Pins configuration
struct MOTOR_PINS {
    int pinEn;
    int pinIN1;
    int pinIN2;
};

std::vector<MOTOR_PINS> motorPins = {
    {12, 13, 15},  // RIGHT_MOTOR Pins (EnA, IN1, IN2)
    {12, 14, 2}    // LEFT_MOTOR Pins (EnB, IN3, IN4)
};

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int PWMSpeedChannel = 2;
const int PWMLightChannel = 3;

// Camera related constants
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

void rotateMotor(int motorNumber, int motorDirection) {
    Serial.printf("Rotating motor %d in direction %d\n", motorNumber, motorDirection);
    if (motorDirection == FORWARD) {
        digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
        digitalWrite(motorPins[motorNumber].pinIN2, LOW);
        ledcWrite(PWMSpeedChannel, 120);
    } else if (motorDirection == BACKWARD) {
        digitalWrite(motorPins[motorNumber].pinIN1, LOW);
        digitalWrite(motorPins[motorNumber].pinIN2, HIGH);
        ledcWrite(PWMSpeedChannel, 120);
    } else {
        digitalWrite(motorPins[motorNumber].pinIN1, LOW);
        digitalWrite(motorPins[motorNumber].pinIN2, LOW);
        ledcWrite(PWMSpeedChannel, 0);
    }
}

void moveCar(int inputValue) {
    Serial.printf("Received command: %d\n", inputValue);
    switch (inputValue) {
        case UP:
            rotateMotor(RIGHT_MOTOR, FORWARD);
            rotateMotor(LEFT_MOTOR, FORWARD);            
            Serial.printf("moving up");
            break;
        case DOWN:
            rotateMotor(RIGHT_MOTOR, BACKWARD);
            rotateMotor(LEFT_MOTOR, BACKWARD);
            Serial.printf("moving down");
            break;
        case LEFT:
            rotateMotor(RIGHT_MOTOR, FORWARD);
            rotateMotor(LEFT_MOTOR, BACKWARD);
            Serial.printf("moving left");
            break;
        case RIGHT:
            rotateMotor(RIGHT_MOTOR, BACKWARD);
            rotateMotor(LEFT_MOTOR, FORWARD);            
            Serial.printf("moving right");
            break;
        case STOP:
            rotateMotor(RIGHT_MOTOR, STOP);
            rotateMotor(LEFT_MOTOR, STOP);
            break;
        default:
            rotateMotor(RIGHT_MOTOR, STOP);
            rotateMotor(LEFT_MOTOR, STOP);
            break;
    }
}

void sendCameraPicture() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Frame buffer could not be acquired");
        return;
    }

    Serial.printf("Captured frame size: %u bytes\n", fb->len);
    webSocket.sendBIN(fb->buf, fb->len);

    esp_camera_fb_return(fb);
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            Serial.println("Connected to WebSocket server");
            webSocket.sendTXT("ESP32_CONNECTED");
            Serial.println("Sent ESP32_CONNECTED message");
            break;
        case WStype_DISCONNECTED:
            Serial.println("Disconnected from WebSocket server");
            break;
        case WStype_TEXT:
            Serial.printf("Received text: %s\n", payload);
            if (strcmp((char *)payload, "UP") == 0) {
                moveCar(UP);
            } else if (strcmp((char *)payload, "DOWN") == 0) {
                moveCar(DOWN);
            } else if (strcmp((char *)payload, "LEFT") == 0) {
                moveCar(LEFT);
            } else if (strcmp((char *)payload, "RIGHT") == 0) {
                moveCar(RIGHT);
            } else if (strcmp((char *)payload, "STOP") == 0) {
                moveCar(STOP);
            } else {
                Serial.println("Received unknown command");
            }
            break;
        case WStype_BIN:
            Serial.printf("Received binary data of length: %u\n", length);
            break;
        default:
            break;
    }
}

void setupCamera() {
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

    config.frame_size = FRAMESIZE_QVGA;  // Lower frame size to QVGA (320x240)
    config.jpeg_quality = 12;  // Lower quality for smaller image size
    config.fb_count = 1;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return;
    }

    if (psramFound()) {
        heap_caps_malloc_extmem_enable(20000);
        Serial.println("PSRAM initialized. malloc to take memory from PSRAM above this size");
    }
}

void setup() {
    Serial.begin(115200);

    // Setup Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Setup WebSocket connection
    webSocket.begin(websocket_server_host, websocket_server_port, "/");
    webSocket.onEvent(webSocketEvent);

    

    // Setup camera
    setupCamera();

    // Setup motor pins
    ledcSetup(PWMSpeedChannel, PWMFreq, PWMResolution);
    ledcSetup(PWMLightChannel, PWMFreq, PWMResolution);
    for (int i = 0; i < motorPins.size(); i++) {
        pinMode(motorPins[i].pinEn, OUTPUT);
        pinMode(motorPins[i].pinIN1, OUTPUT);
        pinMode(motorPins[i].pinIN2, OUTPUT);

        ledcAttachPin(motorPins[i].pinEn, PWMSpeedChannel);
        ledcWrite(PWMSpeedChannel, 0);
    }
    moveCar(STOP);

    pinMode(LIGHT_PIN, OUTPUT);
    ledcAttachPin(LIGHT_PIN, PWMLightChannel);
}

void loop() {
    webSocket.loop();

    static unsigned long lastSendTime = 0;
    if (millis() - lastSendTime > 100) {  // Send frame every second
        sendCameraPicture();
        lastSendTime = millis();
    }

    delay(10);  // Small delay to avoid overwhelming the loop
}
