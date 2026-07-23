#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <BleGamepad.h>
#include <U8g2lib.h>

struct PotentiometerState
{
    uint16_t sampleBuffer[10];
    uint8_t sampleIndex;
    uint8_t samplesCollected;
    unsigned long lastSampleTime;
    unsigned long sampleInterval;
};

PotentiometerState yAxisPot = {{0}, 0, 0, 0, 2};
PotentiometerState zAxisPot = {{0}, 0, 0, 0, 2};

#define SDA_PIN 5
#define SCL_PIN 6
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE);

unsigned long lastDisplayUpdate = 0;

extern BleGamepad bleGamepad;
extern uint8_t operatingMode;
extern int16_t xAxisValue;
extern int16_t yAxisValue;
extern int16_t zAxisValue;
extern bool digitalInputs[];


void updateOLED()
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x7_tr);



    int steer = map(xAxisValue,-32767,32767,-30,30);
    u8g2.drawVLine(36,1,7);
    if(steer>0) u8g2.drawBox(36,2,steer,5);
    else if(steer<0) u8g2.drawBox(36+steer,2,-steer,5);

    int thr = map(zAxisValue,-32767,32767,0,26);
    thr = constrain(thr,0,20);
    u8g2.drawFrame(1,19,8,20);
    u8g2.drawBox(2,39-thr,6,thr);

    int brk = map(yAxisValue,-32767,32767,0,26);
    brk = constrain(brk,0,20);
    u8g2.drawFrame(63,19,8,20);
    u8g2.drawBox(64,39-brk,6,brk);

    for(int i=0;i<4;i++)
    {
      if(digitalInputs[i]) u8g2.drawBox(18+i*10,31,6,6);
      else u8g2.drawFrame(18+i*10,31,6,6);
    }
    u8g2.sendBuffer();
}

//================================================
// SLAVE ID - CHANGE THIS VALUE: 1 or 2
//================================================

#define SLAVE_ID 1

//================================================
// GPIO PINS
//================================================

#define D0 1
#define D1 2
#define D2 4
//#define D3 3

// Analog axis pins (Potentiometers)
#define AXIS_Y_PIN 3  // Y axis potentiometer (Left Trigger)
#define AXIS_Z_PIN 0  // Z axis potentiometer (Right Trigger)

// Zero button
#define ZERO_BTN 9    // Zero steering button



//================================================
// INPUT STATE
//================================================

// Digital inputs (D0-D3 and MODE buttons)
bool digitalInputs[6];
bool lastDigitalInputs[6];
uint8_t debounceCounter[6] = {0, 0, 0, 0, 0, 0};
const uint8_t DEBOUNCE_THRESHOLD = 3;  // ~3-5ms debounce

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
BleGamepad bleGamepad("EL-GP2", "LEHIVXX", 69);

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
// POTENTIOMETER SETTINGS - NON-BLOCKING
//================================================

const uint8_t POT_SAMPLE_COUNT = 10;
//const uint16_t ADC_MAX = 4095;
//const uint16_t ADC_MIN = 0;

// Circular buffer for non-blocking pot sampling

//================================================
// MODE DETECTION
//================================================

//================================================
// READ INPUTS WITH DEBOUNCING
//================================================

void readInputs()
{
    // Read raw input states (D0-D3 and MODE buttons)
    bool rawInputs[6] = {
        !digitalRead(D0),
        !digitalRead(D1),
        !digitalRead(D2),
        0,
        0,
        0
    };

    // Simple debouncing: counter-based
    for(int i = 0; i < 6; i++)
    {
        if(rawInputs[i] == lastDigitalInputs[i])
        {
            // Input consistent with last state
            if(debounceCounter[i] < DEBOUNCE_THRESHOLD)
                debounceCounter[i]++;
            
            // Confirm state after threshold
            if(debounceCounter[i] >= DEBOUNCE_THRESHOLD)
                digitalInputs[i] = lastDigitalInputs[i];
        }
        else
        {
            // Input changed, reset counter
            debounceCounter[i] = 0;
            lastDigitalInputs[i] = rawInputs[i];
        }
    }
}

//================================================
// NON-BLOCKING POTENTIOMETER SAMPLING
//================================================

void updatePotentiometerSampling(PotentiometerState &pot, uint8_t adcPin)
{
    unsigned long now = micros();
    
    if((now - pot.lastSampleTime) >= (pot.sampleInterval * 1000))
    {
        pot.lastSampleTime = now;
        
        // Collect one sample
        pot.sampleBuffer[pot.sampleIndex] = analogRead(adcPin);
        pot.sampleIndex++;
        
        if(pot.sampleIndex >= POT_SAMPLE_COUNT)
            pot.sampleIndex = 0;
        
        if(pot.samplesCollected < POT_SAMPLE_COUNT)
            pot.samplesCollected++;
    }
}

int16_t calculateAveragedPotValue(const PotentiometerState &pot,
uint16_t ADC_MIN,uint16_t ADC_MAX )
{
    if(pot.samplesCollected == 0)
        return 0;
    
    uint32_t sum = 0;
    for(uint8_t i = 0; i < pot.samplesCollected; i++)
    {
        sum += pot.sampleBuffer[i];
    }
    
    int potValue = sum / pot.samplesCollected;
    
    // Constrain to valid range
    potValue = constrain(potValue, ADC_MIN, ADC_MAX);
  //  Serial.print(potValue);
    // Map to axis range (-32767 to 32767)
    int16_t mappedValue = (int16_t)map(potValue, ADC_MIN, ADC_MAX, -32767, 32767);
    
    return mappedValue;
}

void readPotentiometers()
{
    // Non-blocking: only collect one sample per call
    updatePotentiometerSampling(yAxisPot, AXIS_Y_PIN);
    updatePotentiometerSampling(zAxisPot, AXIS_Z_PIN);
     //Serial.print("y= ");
    // Update axis values from averaged samples
    yAxisValue = calculateAveragedPotValue(yAxisPot,650,3050);
 //   Serial.print(" | z= ");

  zAxisValue = calculateAveragedPotValue(zAxisPot,650,3050);
   
 // Serial.print(yAxisValue);
 //   Serial.print(" | z= ");
// Serial.println();
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
}

//================================================
// APPLY MAPPING
//================================================

void applyMapping()
{
    buttons = 0;
    memset(axis, 0, sizeof(axis));

    // Digital inputs (D0-D3 and MODE buttons) -> buttons
    for(int i = 0; i < 6; i++)
    {
        if(digitalInputs[i])
            buttons |= (1 << i);
    }

    // Axes from MPU6050 and potentiometers
    axis[0] = xAxisValue;  // X from MPU6050
    axis[1] = yAxisValue;  // Y from Pin 36 potentiometer
    axis[2] = zAxisValue;  // Z from Pin 39 potentiometer
}

//================================================
// SEND TO BLE GAMEPAD
//================================================

void sendBleGamepad()
{
    if(!bleGamepad.isConnected())
        return;

    // Map buttons to BLE gamepad (all 6 buttons)
    for(int i = 0; i < 6; i++)
    {
        if(digitalInputs[i])
            bleGamepad.press(i + 1);  // Button 1-6
        else
            bleGamepad.release(i + 1);
    }

    // Map analog sticks
  //  bleGamepad.setLeftThumb(axis[0], );
   //  bleGamepad.setSteering();
  //    bleGamepad.setBrake(axis[1]);
  //  bleGamepad.setAccelerator(axis[2]);
  //  bleGamepad.setRightThumb(axis[2], 0);
        bleGamepad.setAxes(axis[0], 0, axis[1], 0, axis[2], 0, 0, 0);       //(X, Y, Z, RX, RY, RZ)

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

    esp_err_t result = esp_now_send(masterAddress, (uint8_t*)&packet, sizeof(packet));
    
    if(result != ESP_OK)
    {
      //  Serial.print("ESP-NOW Send Error: ");
     //   Serial.println(result);
    }
}

//================================================
// INIT MPU6050
//================================================

void initMpu6050()
{
 //   Wire.begin(5, 6);

  //  Serial.println("Initializing MPU6050...");

    byte status = mpu.begin();

    if (status != 0)
    {
        Serial.print("MPU6050 Error: ");
        Serial.println(status);
       // while(1)
       // {
            delay(500);
        
    }

    delay(500);

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

  //  Serial.print("Slave ");
   // Serial.print(SLAVE_ID);
  //  Serial.println(" ESP-NOW initialized");
}

//================================================
// INIT BLE GAMEPAD
//================================================

void initBleGamepad()
{
    BleGamepadConfiguration bleGamepadConfig;
    bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);
    bleGamepadConfig.setButtonCount(24);
   //  bleGamepadConfig.setWhichAxes(true, true, true, true, true, true, false, false);      // Can also be done per-axis individually. All are true by default
  //  bleGamepadConfig.setWhichSimulationControls(false, false, true, true, true); // Can also be done per-control individually. All are false by default
   // bleGamepadConfig.setHatSwitchCount(numOfHatSwitches);                                                                      // 1 by default

   bleGamepadConfig.setAxesMin(0x8000); // 0 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
    bleGamepadConfig.setAxesMax(0x7FFF); // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal 
      bleGamepadConfig.setSimulationMin(0x8000);
    bleGamepadConfig.setSimulationMax(0x7FFF);
    bleGamepadConfig.setIncludeStart(true);
    bleGamepadConfig.setIncludeSelect(true);

    bleGamepad.begin(&bleGamepadConfig);
   // Serial.println("BLE Gamepad initialized");
}

//================================================
// SETUP
//================================================

void setup()
{
    Serial.begin(115200);
    Wire.begin(SDA_PIN,SCL_PIN);
    u8g2.begin();

    // Setup GPIO pins
    pinMode(D0, INPUT_PULLUP);
    pinMode(D1, INPUT_PULLUP);
    pinMode(D2, INPUT_PULLUP);
    // pinMode(AXIS_Y_PIN, INPUT_PULLUP);
    // pinMode(AXIS_Z_PIN, INPUT_PULLUP);
    pinMode(ZERO_BTN, INPUT_PULLUP);

  //  Serial.println();
  //  Serial.println("================================");
 // //  Serial.println(" ELGP CONTROLLER SLAVE");
   // Serial.println("================================");
  //  Serial.println();

    // Initialize MPU6050
    initMpu6050();

        initBleGamepad();
        initESPNow();
    

  /*  Serial.println("GPIO23 = Zero Steering");
    Serial.println("GPIO36 = Y Axis Potentiometer (Left Trigger)");
    Serial.println("GPIO39 = Z Axis Potentiometer (Right Trigger)");
    Serial.println("GPIO27 = Mode Button 1");
    Serial.println("GPIO17 = Mode Button 2");
    Serial.println("GPIO16 = Button D3");
    Serial.println("GPIO14 = Button D2");
    Serial.println("GPIO4 = Button D1");
    Serial.println("GPIO12 = Button D0");*/
  //  Serial.println();
}

//================================================
// LOOP
//================================================

unsigned long lastSend = 0;

void loop()
{
    processMpu6050();
    readInputs();
    readPotentiometers();
    applyMapping();

        sendBleGamepad();
        
        if(millis() - lastSend > 5)
        {
            sendESPNOW();
            lastSend = millis();
        }
    

    if(millis()-lastDisplayUpdate>50){ updateOLED(); lastDisplayUpdate=millis(); }
}
