#include <Arduino.h>
#include <BleGamepad.h>
#include <U8g2lib.h>
#include <Wire.h>

// ======================================================
// PINS
// ======================================================
#define BTN1_PIN 1
#define BTN2_PIN 2
#define POT1_PIN 3   // X Axis
#define POT2_PIN 4   // Y Axis

#define SDA_PIN 5
#define SCL_PIN 6

#define AVG_COUNT 10

// ======================================================
// OLED
// ======================================================
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(
    U8G2_R0,
    U8X8_PIN_NONE);

// ======================================================
// BLE GAMEPAD
// ======================================================
BleGamepad bleGamepad(
    "ESP32 Controller",
    "Custom",
    100);

// ======================================================
// FILTERING
// ======================================================
float buffer1[AVG_COUNT];
float buffer2[AVG_COUNT];

int index1 = 0;
int index2 = 0;

bool filled1 = false;
bool filled2 = false;

// ======================================================
// VARIABLES
// ======================================================
int X = 0;
int Y = 0;

bool btn1State = false;
bool btn2State = false;

unsigned long lastSend = 0;

// ======================================================
// RUNNING AVERAGE
// ======================================================
float runningAverage(
    float *buffer,
    int &index,
    bool &filled,
    float newValue)
{
    buffer[index] = newValue;

    index = (index + 1) % AVG_COUNT;

    if (index == 0)
        filled = true;

    int count = filled ? AVG_COUNT : index;

    if (count == 0)
        return newValue;

    float sum = 0;

    for (int i = 0; i < count; i++)
        sum += buffer[i];

    return sum / count;
}

// ======================================================
// MAP TO 0-255
// ======================================================
int mapTo255(int value, int in_min, int in_max)
{
    int mapped = map(value, in_min, in_max, 0, 255);
    return constrain(mapped, 0, 255);
}

// ======================================================
// OLED
// ======================================================
void doScreen()
{
    u8g2.clearBuffer();

    // -------------------------------
    // Connection indicator
    // -------------------------------
    u8g2.setFont(u8g2_font_5x7_tr);

    if (bleGamepad.isConnected())
        u8g2.drawStr(0, 7, "BLE");
    else
        u8g2.drawStr(0, 7, "DISC");

    // -------------------------------
    // Throttle Bar (Y)
    // -------------------------------
    int barLength1 = map(Y, 0, 255, 0, 30);

    u8g2.drawLine(32, 39, 40, 39);
    u8g2.drawBox(
        32,
        39 - barLength1,
        8,
        barLength1);

    // -------------------------------
    // Steering Bar (X)
    // -------------------------------
    int centerX = 36;

    int mappedSteer = map(X, 0, 255, -127, 127);

    int barLength2 =
        map(abs(mappedSteer), 0, 127, 0, 32);

    u8g2.drawVLine(centerX, 5, 7);

    if (mappedSteer < 0)
    {
        u8g2.drawBox(
            centerX - barLength2,
            6,
            barLength2,
            5);
    }
    else if (mappedSteer > 0)
    {
        u8g2.drawBox(
            centerX + 1,
            6,
            barLength2,
            5);
    }

    // -------------------------------
    // Buttons
    // -------------------------------
    if (btn1State)
        u8g2.drawBox(2, 32, 6, 6);
    else
        u8g2.drawFrame(2, 32, 6, 6);

    if (btn2State)
        u8g2.drawBox(64, 32, 6, 6);
    else
        u8g2.drawFrame(64, 32, 6, 6);

    // -------------------------------
    // Steering Value
    // -------------------------------
    char buf[16];
    sprintf(buf, "%d", mappedSteer);
    u8g2.drawStr(18, 28, buf);

    u8g2.sendBuffer();
}

// ======================================================
// SETUP
// ======================================================
void setup()
{
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);

    u8g2.begin();

    pinMode(BTN1_PIN, INPUT_PULLUP);
    pinMode(BTN2_PIN, INPUT_PULLUP);

    analogReadResolution(12);

    BleGamepadConfiguration config;

    config.setButtonCount(2);

    config.setWhichAxes(
        true,   // X
        true,   // Y
        false,  // Z
        false,  // RX
        false,  // RY
        false,  // RZ
        false,  // Rudder
        false,  // Throttle
        false,  // Accelerator
        false,  // Brake
        false   // Steering
    );

    bleGamepad.setConfiguration(config);
    bleGamepad.begin();

    Serial.println("BLE Gamepad Started");
}

// ======================================================
// LOOP
// ======================================================
void loop()
{
    unsigned long now = millis();

    // -------------------------------
    // Read buttons
    // -------------------------------
    btn1State = !digitalRead(BTN1_PIN);
    btn2State = !digitalRead(BTN2_PIN);

    // -------------------------------
    // Read analogs
    // -------------------------------
    int raw1 = analogRead(POT1_PIN);
    int raw2 = analogRead(POT2_PIN);

    float avg1 =
        runningAverage(
            buffer1,
            index1,
            filled1,
            raw1);

    float avg2 =
        runningAverage(
            buffer2,
            index2,
            filled2,
            raw2);

    X = mapTo255((int)avg1, 0, 4095);
    Y = mapTo255((int)avg2, 0, 4095);

    // -------------------------------
    // Send gamepad report @ 50Hz
    // -------------------------------
    if (now - lastSend >= 20)
    {
        lastSend = now;

        doScreen();

        int joyX =
            map((int)avg1,
                0,
                4095,
                -32767,
                32767);

        int joyY =
            map((int)avg2,
                0,
                4095,
                -32767,
                32767);

        if (bleGamepad.isConnected())
        {
            bleGamepad.setX(joyX);
            bleGamepad.setY(joyY);

            if (btn1State)
                bleGamepad.press(BUTTON_1);
            else
                bleGamepad.release(BUTTON_1);

            if (btn2State)
                bleGamepad.press(BUTTON_2);
            else
                bleGamepad.release(BUTTON_2);

            bleGamepad.sendReport();
        }

        Serial.printf(
            "X=%6d  Y=%6d  B1=%d  B2=%d  BLE=%d\n",
            joyX,
            joyY,
            btn1State,
            btn2State,
            bleGamepad.isConnected());
    }
}
