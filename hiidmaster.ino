#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Keypad.h>

#include "USB.h"
#include "USBHID.h"
#include "USBHIDGamepad.h"

#include <BleGamepad.h>

//================================================
// MASTER MODE DETECTION PINS
//================================================

#define BTN1 4
#define BTN2 5

//================================================
// KEYPAD CONFIGURATION
//================================================

#define KEYPAD_ROWS 4
#define KEYPAD_COLS 3

uint8_t rowPins[KEYPAD_ROWS] = {6, 7, 8, 9};     // Adjust to suit
uint8_t colPins[KEYPAD_COLS] = {10, 11, 12};     // Adjust to suit

uint8_t keymap[KEYPAD_ROWS][KEYPAD_COLS] =
{
    {0, 1, 2},      // Buttons 1, 2, 3
    {3, 4, 5},      // Buttons 4, 5, 6
    {6, 7, 8},      // Buttons 7, 8, 9
    {9, 10, 11}     // Buttons 10, 11, 12
};

Keypad customKeypad = Keypad(makeKeymap(keymap), rowPins, colPins, KEYPAD_ROWS, KEYPAD_COLS);

//================================================
// SYSTEM LIMITS
//================================================

#define MAX_BUTTONS 24              // 12 from keypad + 6 from slave1 + 6 from slave2
#define MAX_AXES 6                  // 3 from each slave
#define MAX_SLAVES 2

//================================================
// DEVICE MODE
//================================================

enum DeviceMode
{
    MODE_USB,
    MODE_BLE
};

DeviceMode mode;

//================================================
// GAMEPAD STATE
//================================================

uint32_t buttons = 0;
int16_t axis[MAX_AXES];

//================================================
// USB + BLE DEVICES
//================================================

USBHIDGamepad usbGamepad;
BleGamepad bleGamepad("ESP32_Modular_Gamepad","ESP32",100);

//================================================
// SLAVE PACKET FORMAT
//================================================

typedef struct
{
    char name[32];
    uint32_t buttons;
    int16_t axis[3];    // 3 axes per slave
} GamepadPacket;

//================================================
// SLAVE STORAGE
//================================================

struct SlaveDevice
{
    uint8_t mac[6];
    GamepadPacket data;
    bool active;
    unsigned long lastSeen;
};

SlaveDevice slaves[MAX_SLAVES];

//================================================
// FIND / REGISTER SLAVE
//================================================

int findSlave(uint8_t *mac)
{
    for(int i = 0; i < MAX_SLAVES; i++)
    {
        if(memcmp(slaves[i].mac, mac, 6) == 0)
            return i;
    }

    for(int i = 0; i < MAX_SLAVES; i++)
    {
        if(!slaves[i].active)
        {
            memcpy(slaves[i].mac, mac, 6);
            slaves[i].active = true;

            Serial.print("New slave registered: ");

            for(int b = 0; b < 6; b++)
            {
                Serial.print(mac[b], HEX);
                if(b < 5) Serial.print(":");
            }

            Serial.println();

            return i;
        }
    }

    return -1;
}

//================================================
// ESP-NOW RECEIVE
//================================================

void onReceive(const uint8_t *mac, const uint8_t *data, int len)
{
    if(len != sizeof(GamepadPacket)) return;

    GamepadPacket packet;
    memcpy(&packet, data, sizeof(packet));

    int id = findSlave((uint8_t*)mac);

    if(id < 0) return;

    slaves[id].data = packet;
    slaves[id].lastSeen = millis();
}

//================================================
// INIT ESP-NOW
//================================================

void initESPNow()
{
    WiFi.mode(WIFI_STA);

    if(esp_now_init() != ESP_OK)
    {
        Serial.println("ESP-NOW INIT FAILED");
        return;
    }

    esp_now_register_recv_cb(onReceive);

    Serial.println("ESP-NOW READY");
}

//================================================
// READ KEYPAD
//================================================

void readKeypad()
{
    customKeypad.getKeys();

    for(int i = 0; i < LIST_MAX; i++)
    {
        if(customKeypad.key[i].stateChanged)
        {
            uint8_t keystate = customKeypad.key[i].kstate;
            uint8_t buttonIndex = customKeypad.key[i].kchar;

            if(keystate == PRESSED)
            {
                buttons |= (1 << buttonIndex);
            }
            else if(keystate == RELEASED)
            {
                buttons &= ~(1 << buttonIndex);
            }
        }
    }
}

//================================================
// MERGE SLAVE INPUTS
//================================================

void mergeSlaveInputs()
{
    for(int i = 0; i < MAX_SLAVES; i++)
    {
        if(!slaves[i].active) continue;

        // Timeout check (1 second)
        if(millis() - slaves[i].lastSeen > 1000)
            continue;

        // Map slave buttons to master buttons (starting at button 12)
        // Slave 0: buttons 12-17 (6 buttons)
        // Slave 1: buttons 18-23 (6 buttons)
        buttons |= (slaves[i].data.buttons << (12 + (i * 6)));

        // Map slave axes
        // Slave 0: axes 0-2
        // Slave 1: axes 3-5
        for(int a = 0; a < 3; a++)
        {
            if(slaves[i].data.axis[a] != 0)
                axis[(i * 3) + a] = slaves[i].data.axis[a];
        }
    }
}

//================================================
// UPDATE USB GAMEPAD
//================================================

void updateUSB()
{
    int8_t x  = constrain(axis[0] / 256, -127, 127);
    int8_t y  = constrain(axis[1] / 256, -127, 127);
    int8_t z  = constrain(axis[2] / 256, -127, 127);
    int8_t rz = constrain(axis[3] / 256, -127, 127);
    int8_t rx = constrain(axis[4] / 256, -127, 127);
    int8_t ry = constrain(axis[5] / 256, -127, 127);

    uint8_t hat = 0;

    usbGamepad.send(
        x, y, z, rz, rx, ry, hat, buttons
    );
}

//================================================
// UPDATE BLE GAMEPAD
//================================================

void updateBLE()
{
    if(!bleGamepad.isConnected()) return;

    for(int i = 0; i < MAX_BUTTONS; i++)
    {
        if(buttons & (1 << i))
            bleGamepad.press(i + 1);
        else
            bleGamepad.release(i + 1);
    }

    bleGamepad.setAxes(
        axis[0], axis[1], axis[2],
        axis[3], axis[4], axis[5],
        0, 0
    );
}

//================================================
// DETECT MODE
//================================================

void detectMode()
{
    bool b1 = !digitalRead(BTN1);
    bool b2 = !digitalRead(BTN2);

    if(b1)
        mode = MODE_USB;
    else if(b2)
        mode = MODE_BLE;
    else
        mode = MODE_USB;
}

//================================================
// SETUP
//================================================

void setup()
{
    Serial.begin(115200);

    pinMode(BTN1, INPUT_PULLUP);
    pinMode(BTN2, INPUT_PULLUP);

    detectMode();

    initESPNow();

    if(mode == MODE_USB)
    {
        Serial.println("USB MODE");
        USB.begin();
        usbGamepad.begin();
    }
    else
    {
        Serial.println("BLE MODE");
        bleGamepad.begin();
    }
}

//================================================
// LOOP
//================================================

void loop()
{
    buttons = 0;
    memset(axis, 0, sizeof(axis));

    readKeypad();
    mergeSlaveInputs();

    if(mode == MODE_USB)
        updateUSB();
    else
        updateBLE();

    delay(5);
}
