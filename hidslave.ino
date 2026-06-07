#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

//================================================
// SLAVE ID - CHANGE THIS VALUE: 1 or 2
//================================================

#define SLAVE_ID 1

//================================================
// GPIO PINS
//================================================

#define D0 21
#define D1 14
#define D2 19
#define D3 18

#define A0 36
#define A1 39

//================================================
// INPUT STATE
//================================================

bool digitalInputs[4];
uint16_t analogInputs[2];

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
// READ INPUTS
//================================================

void readInputs()
{
    digitalInputs[0] = !digitalRead(D0);
    digitalInputs[1] = !digitalRead(D1);
    digitalInputs[2] = !digitalRead(D2);
    digitalInputs[3] = !digitalRead(D3);

    analogInputs[0] = analogRead(A0);
    analogInputs[1] = analogRead(A1);
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

    // Digital inputs -> buttons
    for(int i = 0; i < 4; i++)
    {
        if(digitalInputs[i])
            buttons |= (1 << i);
    }

    // Analog inputs -> axes
    axis[0] = processAnalog(0, analogInputs[0]);
    axis[1] = processAnalog(1, analogInputs[1]);
    axis[2] = 0;  // Unused
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
    Serial.println(" initialized");
}

//================================================
// SETUP
//================================================

void setup()
{
    Serial.begin(115200);

    pinMode(D0, INPUT_PULLUP);
    pinMode(D1, INPUT_PULLUP);
    pinMode(D2, INPUT_PULLUP);
    pinMode(D3, INPUT_PULLUP);

    analogReadResolution(12);

    initESPNow();
}

//================================================
// LOOP
//================================================

unsigned long lastSend = 0;

void loop()
{
    readInputs();
    applyMapping();

    if(millis() - lastSend > 5)
    {
        sendESPNOW();
        lastSend = millis();
    }

    delay(1);
}
