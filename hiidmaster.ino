#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "USB.h"
#include "USBHID.h"
#include "USBHIDGamepad.h"

#include <BleGamepad.h>
#include <Keypad.h>

//================================================
// MASTER MODE DETECTION PINS
//================================================

#define BTN1 1
#define BTN2 2

//================================================
// KEYPAD CONFIGURATION
//================================================

#define KEYPAD_ROWS 4
#define KEYPAD_COLS 3

uint8_t rowPins[KEYPAD_ROWS] = {16, 4, 5, 7};
uint8_t colPins[KEYPAD_COLS] = {15, 17, 6};

uint8_t keymap[KEYPAD_ROWS][KEYPAD_COLS] =
{
    {0, 1, 2},
    {3, 4, 5},
    {6, 7, 8},
    {9, 10, 11}
};

Keypad customKeypad = Keypad(makeKeymap(keymap), rowPins, colPins, KEYPAD_ROWS, KEYPAD_COLS);

//================================================
// SYSTEM LIMITS
//================================================

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
uint32_t keypadButtons = 0;  // Track keypad buttons separately
int16_t axis[6];

//================================================
// USB + BLE DEVICES
//================================================

USBHIDGamepad usbGamepad;
BleGamepad bleGamepad("ESP32_Gamepad", "ESP32", 100);

//================================================
// SLAVE PACKET FORMAT
//================================================

typedef struct
{
    uint8_t slaveID;
    uint32_t buttons;
    int16_t axis[3];
} GamepadPacket;

//================================================
// SLAVE STORAGE (Fixed slots: Slave 1 and Slave 2)
//================================================

struct SlaveDevice
{
    uint8_t mac[6];
    GamepadPacket data;
    bool active;
    unsigned long lastSeen;
};

SlaveDevice slaves[2];  // Slave 0 = ID1, Slave 1 = ID2

//================================================
// REGISTER / UPDATE SLAVE
//================================================

void registerSlave(uint8_t *mac, uint8_t slaveID)
{
    if(slaveID < 1 || slaveID > 2) return;

    int idx = slaveID - 1;
    memcpy(slaves[idx].mac, mac, 6);
    slaves[idx].active = true;
    slaves[idx].lastSeen = millis();

    Serial.print("Slave ");
    Serial.print(slaveID);
    Serial.print(" registered: ");
    for(int b = 0; b < 6; b++)
    {
        Serial.print(mac[b], HEX);
        if(b < 5) Serial.print(":");
    }
    Serial.println();
}

//================================================
// ESP-NOW RECEIVE
//================================================

void onReceive(const uint8_t *mac, const uint8_t *data, int len)
{
    if(len != sizeof(GamepadPacket)) return;

    GamepadPacket packet;
    memcpy(&packet, data, sizeof(packet));

    registerSlave((uint8_t*)mac, packet.slaveID);

    int idx = packet.slaveID - 1;
    slaves[idx].data = packet;
    slaves[idx].lastSeen = millis();
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
                keypadButtons |= (1 << buttonIndex);
            }
            else if(keystate == RELEASED)
            {
                keypadButtons &= ~(1 << buttonIndex);
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

        // Timeout check
        if(millis() - slaves[i].lastSeen > 1000)
            continue;

        uint8_t slaveID = i + 1;

        if(slaveID == 1)
        {
            // Slave 1: buttons 1-6 (bit positions 0-5)
            buttons |= (slaves[i].data.buttons & 0x3F);

            // Slave 1: axes X, Y, Z (axis positions 0, 1, 2)
            axis[0] = slaves[i].data.axis[0];
            axis[3] = slaves[i].data.axis[1];
            axis[2] = slaves[i].data.axis[2];
        }
        else if(slaveID == 2)
        {
            // Slave 2: buttons 7-13 (bit positions 6-12)
            buttons |= ((slaves[i].data.buttons & 0x7F) << 6);

            // Slave 2: axes RZ, RX, RY (axis positions 3, 4, 5)
            axis[1] = slaves[i].data.axis[0];
            axis[4] = slaves[i].data.axis[1];
            axis[5] = slaves[i].data.axis[2];
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

    usbGamepad.send(x, y, z, rz, rx, ry, hat, buttons);
}

//================================================
// UPDATE BLE GAMEPAD
//================================================

void updateBLE()
{
    if(!bleGamepad.isConnected()) return;

    for(int i = 0; i < 26; i++)
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
    buttons = keypadButtons;  // Start with keypad buttons
    memset(axis, 0, sizeof(axis));

    readKeypad();
    mergeSlaveInputs();

    if(mode == MODE_USB)
        updateUSB();
    else
        updateBLE();

    delay(5);
}
