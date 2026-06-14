#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <BleGamepad.h>

//================================================
// SLAVE ID - CHANGE THIS VALUE: 1 or 2
//================================================

#define SLAVE_ID 1

//================================================
// GPIO PINS
//================================================

#define D0 12
#define D1 4
#define D2 14
#define D3 16

// Mode buttons
#define MODE_BTN_1 27  // First mode button
#define MODE_BTN_2 17  // Second mode button

// Axis buttons
#define AXIS_Y_BTN 25  // Y axis button (Left Trigger)
#define AXIS_y_BTN 26  // Z axis button (Right Trigger)
#define AXIS_Z_BTN 19  
#define AXIS_z_BTN 18 
// Zero button
#define ZERO_BTN 23    // Zero steering button

//================================================
// OPERATING MODES
//================================================

#define MODE_BLE_ONLY      0
#define MODE_ESPNOW_ONLY   1
#define MODE_BLE_AND_ESPNOW 2

uint8_t operatingMode = MODE_BLE_AND_ESPNOW;  // Default mode

//================================================
// INPUT STATE
//================================================

bool digitalInputs[6];
int16_t xAxisValue = 0;
int16_t yAxisValue = 0;
int16_t zAxisValue = 0;

//================================================
// GAMEPAD STATE
//================================================

uint32_t buttons = 0;
int16_t axis[3];

//================================================
// MASTER MAC
//================================================

uint8_t masterAddress[] = {0xCC, 0x8D, 0xA2, 0xEC, 0xDC, 0xAC};

//================================================
// ESPNOW PACKET
//================================================

typedef struct
{
    uint8_t slaveID;
    uint32_t buttons;
    int16_t axis[3];
} GamepadPacket;

GamepadPacket packet;

//================================================
// BLE GAMEPAD
//================================================

//USBHIDGamepad usbGamepad;
BleGamepad bleGamepad("EL-GP1", "LEHIVXX", 69);

//================================================
// MPU6050
//================================================

MPU6050 mpu(Wire);

//================================================
// FILTER SETTINGS
//================================================

float angleOffset = 0.0f;
bool zeroButtonState = false;
bool lastZeroButtonState = false;

float smoothedAngle = 0.0f;
const float alpha = 0.1f;

//================================================
// SAMPLING
//================================================

const unsigned long sampleIntervalUS = 2000;
unsigned long lastSampleTime = 0;

//================================================
// AVERAGING
//================================================

float sampleBuffer[10];
uint8_t sampleIndex = 0;
uint8_t sampleCount = 0;

//================================================
// STEERING
//================================================

const float maxAngle = 80.0f;
const int16_t maxStick = 32767;
const int16_t deadzone = 0;
const float steeringGain = 1.0f;

//================================================
// MODE DETECTION
//================================================

void detectMode()
{
    delay(100);  // Wait for pins to stabilize
    
    bool btn1Pressed = digitalRead(MODE_BTN_1) == LOW;
    bool btn2Pressed = digitalRead(MODE_BTN_2) == LOW;

    if(btn1Pressed && !btn2Pressed)
    {
        operatingMode = MODE_BLE_ONLY;
        Serial.println("Mode: BLE GAMEPAD ONLY");
    }
    else if(btn2Pressed && !btn1Pressed)
    {
       
        operatingMode = MODE_BLE_AND_ESPNOW;
        Serial.println("Mode: BLE GAMEPAD + ESP-NOW");
    }
    else
    {
       operatingMode = MODE_ESPNOW_ONLY;
        Serial.println("Mode: ESP-NOW ONLY");
    }
}

//================================================
// READ INPUTS
//================================================

void readInputs()
{
    digitalInputs[0] = !digitalRead(D0);
    digitalInputs[1] = !digitalRead(D1);
    digitalInputs[2] = !digitalRead(D2);
    digitalInputs[3] = !digitalRead(D3);
    digitalInputs[4] = !digitalRead(MODE_BTN_1);
    digitalInputs[5] = !digitalRead(MODE_BTN_2);
}

//================================================
// PROCESS MPU6050
//================================================

void processMpu6050()
{
    mpu.update();

    // =====================================
    // Zero Button
    // =====================================
    zeroButtonState = (digitalRead(ZERO_BTN) == LOW);

    if (zeroButtonState && !lastZeroButtonState)
    {
        angleOffset = mpu.getAngleX();
        smoothedAngle = 0.0f;

        for (int i = 0; i < 10; i++)
        {
            sampleBuffer[i] = 0.0f;
        }

        sampleIndex = 0;
        sampleCount = 0;

        Serial.println("Angle Zeroed");
    }

    lastZeroButtonState = zeroButtonState;

    // =====================================
    // Sampling
    // =====================================
    unsigned long now = micros();

    if ((now - lastSampleTime) >= sampleIntervalUS)
    {
        lastSampleTime = now;

        float rawAngle = mpu.getAngleX() - angleOffset;

        smoothedAngle = alpha * rawAngle + (1.0f - alpha) * smoothedAngle;

        smoothedAngle = constrain(smoothedAngle, -maxAngle, maxAngle);

        sampleBuffer[sampleIndex] = smoothedAngle;

        sampleIndex++;

        if (sampleIndex >= 10)
            sampleIndex = 0;

        if (sampleCount < 10)
            sampleCount++;

        // =====================================
        // Average every 10 samples
        // =====================================
        if (sampleCount == 10 && sampleIndex == 0)
        {
            float sum = 0.0f;

            for (int i = 0; i < 10; i++)
            {
                sum += sampleBuffer[i];
            }

            float avgAngle = sum / 10.0f;

            avgAngle = constrain(avgAngle, -maxAngle, maxAngle);

            // =====================================
            // Steering Mapping
            // =====================================

            float normalized = avgAngle / maxAngle;
            float scaled = normalized * steeringGain;
            scaled = constrain(scaled, -1.0f, 1.0f);

            xAxisValue = (int16_t)(scaled * maxStick);

            if (abs(xAxisValue) < deadzone)
                xAxisValue = 0;
        }
    }

    // Y axis from GPIO25 button (max when pressed, min when released)
    bool YBtnPressed = digitalRead(AXIS_Y_BTN) == LOW;
      bool yBtnPressed = digitalRead(AXIS_y_BTN) == LOW;
    //zAxisValue = yBtnPressed ? 0 : 16384;

    // Z axis from GPIO26 button (max when pressed, min when released)
    bool ZBtnPressed = digitalRead(AXIS_Z_BTN) == LOW;

   bool zBtnPressed = digitalRead(AXIS_z_BTN) == LOW;
  
    if(!zBtnPressed && ZBtnPressed){
      zAxisValue = 32767;
    }
     if(zBtnPressed && !ZBtnPressed){
    zAxisValue = 0;
    }
     if(!zBtnPressed && !ZBtnPressed){
    zAxisValue = 16384;
    }
  
    if(!yBtnPressed && YBtnPressed){
      yAxisValue = 32767;
    }
 if(yBtnPressed && !YBtnPressed){
    yAxisValue = 0;
    }
   if(!yBtnPressed && !YBtnPressed){
    yAxisValue = 16384;
    }
    //zAxisValue = zBtnPressed ? 32767 : 16384;
}

//================================================
// APPLY MAPPING
//================================================

void applyMapping()
{
    buttons = 0;
    memset(axis, 0, sizeof(axis));

    // Digital inputs (D0-D3) -> buttons
    for(int i = 0; i < 6; i++)
    {
        if(digitalInputs[i])
            buttons |= (1 << i);
    }

    // Axes from MPU6050 and buttons
    axis[0] = xAxisValue;  // X from MPU6050
    axis[1] = yAxisValue;  // Y from GPIO25
    axis[2] = zAxisValue;  // Z from GPIO26
}

//================================================
// SEND TO BLE GAMEPAD
//================================================

void sendBleGamepad()
{
    if(!bleGamepad.isConnected())
        return;

    // Map buttons to BLE gamepad
    for(int i = 0; i < 6; i++)
    {
        if(digitalInputs[i])
            bleGamepad.press(i + 1);  // Button 1-4
        else
            bleGamepad.release(i + 1);
    }

    // Map analog sticks
    bleGamepad.setLeftThumb(axis[0], axis[1]);
    bleGamepad.setRightThumb(axis[2], 0);

    bleGamepad.sendReport();
}

//================================================
// ESPNOW SEND
//================================================

void sendESPNOW()
{
    packet.slaveID = SLAVE_ID;
    packet.buttons = buttons;
    memcpy(packet.axis, axis, sizeof(axis));

    esp_now_send(masterAddress, (uint8_t*)&packet, sizeof(packet));
}

//================================================
// INIT MPU6050
//================================================

void initMpu6050()
{
    Wire.begin(21, 22);

    Serial.println("Initializing MPU6050...");

    byte status = mpu.begin();

    if (status != 0)
    {
        Serial.print("MPU6050 Error: ");
        Serial.println(status);
        while(1)
        {
            delay(100);
        }
    }

    delay(1000);

    Serial.println("Calibrating MPU6050...");
    mpu.calcOffsets(true, true);

    for (int i = 0; i < 10; i++)
    {
        sampleBuffer[i] = 0.0f;
    }

    Serial.println("MPU6050 initialized");
}

//================================================
// INIT ESPNOW
//================================================

void initESPNow()
{
    WiFi.mode(WIFI_STA);
    esp_now_init();

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, masterAddress, 6);
    peer.channel = 0;
    peer.encrypt = false;

    esp_now_add_peer(&peer);

    Serial.print("Slave ");
    Serial.print(SLAVE_ID);
    Serial.println(" ESP-NOW initialized");
}

//================================================
// INIT BLE GAMEPAD
//================================================

void initBleGamepad()
{
    BleGamepadConfiguration bleGamepadConfig;
    bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
    bleGamepadConfig.setButtonCount(24);
//    bleGamepadConfig.setAxesMin(0x8001); // 0 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
    bleGamepadConfig.setAxesMax(0x7FFF); // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal 
    bleGamepadConfig.setIncludeStart(true);
    bleGamepadConfig.setIncludeSelect(true);

    bleGamepad.begin(&bleGamepadConfig);
    Serial.println("BLE Gamepad initialized");
}

//================================================
// SETUP
//================================================

void setup()
{
    Serial.begin(115200);

    // Setup GPIO pins
    pinMode(D0, INPUT_PULLUP);
    pinMode(D1, INPUT_PULLUP);
    pinMode(D2, INPUT_PULLUP);
    pinMode(D3, INPUT_PULLUP);
    pinMode(MODE_BTN_1, INPUT_PULLUP);
    pinMode(MODE_BTN_2, INPUT_PULLUP);
    pinMode(AXIS_Y_BTN, INPUT_PULLUP);
    pinMode(AXIS_Z_BTN, INPUT_PULLUP);
      pinMode(AXIS_y_BTN, INPUT_PULLUP);
    pinMode(AXIS_z_BTN, INPUT_PULLUP);
    pinMode(ZERO_BTN, INPUT_PULLUP);

    Serial.println();
    Serial.println("================================");
    Serial.println(" ELGP CONTROLLER SLAVE");
    Serial.println("================================");
    Serial.println();

    // Initialize MPU6050
    initMpu6050();

    // Detect operating mode based on button presses at startup
    detectMode();

    // Initialize based on mode
    if(operatingMode == MODE_BLE_ONLY)
    {
        initBleGamepad();
    }
    else if(operatingMode == MODE_ESPNOW_ONLY)
    {
        initESPNow();
    }
    else  // MODE_BLE_AND_ESPNOW
    {
        initBleGamepad();
        initESPNow();
    }

    Serial.println("GPIO19 = Zero Steering");
    Serial.println("GPIO25 = Y Axis (Left Trigger)");
    Serial.println("GPIO26 = Z Axis (Right Trigger)");
    Serial.println("GPIO27 = Mode Button 1");
    Serial.println("GPIO14 = Mode Button 2");
    Serial.println("GPIO16 = Button D3");
    Serial.println("GPIO17 = Button D2");
    Serial.println("GPIO18 = Button D1");
    Serial.println("GPIO19 = Button D0");
    Serial.println();
}

//================================================
// LOOP
//================================================

unsigned long lastSend = 0;

void loop()
{
    processMpu6050();
    readInputs();
    applyMapping();

    // Send based on operating mode
    if(operatingMode == MODE_BLE_ONLY)
    {
        sendBleGamepad();
    }
    else if(operatingMode == MODE_ESPNOW_ONLY)
    {
        if(millis() - lastSend > 5)
        {
            sendESPNOW();
            lastSend = millis();
        }
    }
    else  // MODE_BLE_AND_ESPNOW
    {
        sendBleGamepad();
        
        if(millis() - lastSend > 5)
        {
            sendESPNOW();
            lastSend = millis();
        }
    }

    delay(1);
}
