#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <BleGamepad.h>

//================================================
// SLAVE ID - CHANGE THIS VALUE: 1 or 2
//================================================

#define SLAVE_ID 2

//================================================
// GPIO PINS
//================================================

#define D0 19
#define D1 18
#define D2 17
#define D3 16

#define A0 36
#define A1 39

// Mode buttons
#define MODE_BTN_1 27  // First mode button
#define MODE_BTN_2 14  // Second mode button

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
uint16_t analogInputs[3];

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

BleGamepad bleGamepad;

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
        operatingMode = MODE_ESPNOW_ONLY;
        Serial.println("Mode: ESP-NOW ONLY");
    }
    else
    {
        operatingMode = MODE_BLE_AND_ESPNOW;
        Serial.println("Mode: BLE GAMEPAD + ESP-NOW");
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

    analogInputs[0] = analogRead(A0);
    analogInputs[1] = analogRead(A1);
    analogInputs[2] = analogRead(35);  // Add third analog input if available
}

//================================================
// ANALOG PROCESSING
//================================================

int16_t processAnalog(uint8_t index, int raw)
{
    int center = 2048;
    int deadzone = 100;
    bool invert = false;
    int outMin = -32767;
    int outMax = 32767;

    if(abs(raw - center) < deadzone)
        raw = center;

    long v = map(raw, 0, 4095, outMin, outMax);

    if(invert)
        v = -v;

    return constrain(v, outMin, outMax);
}

//================================================
// APPLY MAPPING
//================================================

void applyMapping()
{
    buttons = 0;
    memset(axis, 0, sizeof(axis));

    // Digital inputs (D0-D3) -> buttons
    for(int i = 0; i < 4; i++)
    {
        if(digitalInputs[i])
            buttons |= (1 << i);
    }

    // Analog inputs -> axes
    axis[0] = processAnalog(0, analogInputs[0]);
    axis[1] = processAnalog(1, analogInputs[1]);
    axis[2] = processAnalog(2, analogInputs[2]);
}

//================================================
// SEND TO BLE GAMEPAD
//================================================

void sendBleGamepad()
{
    if(!bleGamepad.isConnected())
        return;

    // Map buttons to BLE gamepad
    for(int i = 0; i < 4; i++)
    {
        if(digitalInputs[i])
            bleGamepad.press(i + 1);  // Button 1-4
        else
            bleGamepad.release(i + 1);
    }

    // Map analog sticks
    bleGamepad.setLeftStick(axis[0], axis[1]);
    bleGamepad.setRightStick(0, 0);  // Or use axis[2] for right stick

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
    bleGamepadConfig.setControllerName("ELGP Gamepad");
    bleGamepadConfig.setButtonCount(4);
    bleGamepadConfig.setIncludeStart(true);
    bleGamepadConfig.setIncludeSelect(true);
    bleGamepadConfig.setIncludeAnalogSticks(true);

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

    analogReadResolution(12);

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
}

//================================================
// LOOP
//================================================

unsigned long lastSend = 0;

void loop()
{
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
