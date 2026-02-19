#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SX128XLT.h>

// ================================================================
// --- 1. PIN DEFINITIONS (MATCHING YOUR SCHEMATIC) ---
// ================================================================

// Analog Inputs (Joysticks + Knobs)
#define PIN_JOY_YAW     35
#define PIN_JOY_THR     34
#define PIN_JOY_ROLL    33
#define PIN_JOY_PITCH   32
#define PIN_KNOB_PITCH  36 // VP
#define PIN_KNOB_ROLL   39 // VN

// Digital Inputs (Buttons)
#define PIN_BTN_UP      13
#define PIN_BTN_DOWN    12
#define PIN_BTN_ARM     17
#define PIN_BTN_DISARM  16

// Radio (E28-2G4M27S / SX1280)
#define NSS_PIN         5
#define RST_PIN         14
#define BUSY_PIN        4
#define DIO1_PIN        2
#define RX_EN_PIN       27
#define TX_EN_PIN       26
// SPI (VSPI Default): MISO=19, MOSI=23, SCK=18

// Battery & OLED
#define PIN_BATTERY     25
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1 

// ================================================================
// --- 2. CONFIGURATION & DATA STRUCTURES ---
// ================================================================

// Radio Settings (Must match Drone Receiver Settings)
#define LORA_FREQ       2400000000 // 2.4GHz
#define LORA_BW         LORA_BW_0400
#define LORA_SF         LORA_SF7
#define LORA_CR         LORA_CR_4_5
#define TX_POWER        12         // dBm

// Control Packet (13 Bytes - Sends TO Drone)
typedef struct __attribute__((packed)) {
    int16_t throttle;   // 1000 - 2000
    int16_t roll;       // 1000 - 2000
    int16_t pitch;      // 1000 - 2000
    int16_t yaw;        // 1000 - 2000
    int16_t knob_pitch; // 1000 - 2000
    int16_t knob_roll;  // 1000 - 2000
    uint8_t buttons;    // Bitmask
} RC_Input_t;

// Objects
SX128XLT LT;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RC_Input_t packet;

// Globals
unsigned long last_tx_time = 0;
bool local_armed_state = false; // Tracks button presses for UI only

// ================================================================
// --- 3. SETUP ---
// ================================================================
void setup() {
    Serial.begin(115200);

    // 1. Init Pins
    pinMode(PIN_JOY_YAW, INPUT); pinMode(PIN_JOY_THR, INPUT);
    pinMode(PIN_JOY_ROLL, INPUT); pinMode(PIN_JOY_PITCH, INPUT);
    pinMode(PIN_KNOB_PITCH, INPUT); pinMode(PIN_KNOB_ROLL, INPUT);
    pinMode(PIN_BATTERY, INPUT);
    
    // Buttons (Internal Pullups active)
    pinMode(PIN_BTN_UP, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_BTN_ARM, INPUT_PULLUP);
    pinMode(PIN_BTN_DISARM, INPUT_PULLUP);

    // 2. Init OLED
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
        Serial.println(F("OLED Failed")); for(;;);
    }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 20);
    display.setTextSize(2);
    display.println("VISAGE | PULSE");
    display.setTextSize(1);
    display.setCursor(30, 45);
    display.println("SYSTEM INITIALISING...");
    display.display();

    // 3. Init Radio
    SPI.begin();
    if (LT.begin(NSS_PIN, RST_PIN, 0, DIO1_PIN, BUSY_PIN)) {
        Serial.println(F("SX1280 Found"));
        LT.setupLoRa(LORA_FREQ, 0, LORA_SF, LORA_BW, LORA_CR); 
    } else {
        Serial.println(F("SX1280 Error"));
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("RADIO ERROR");
        display.display();
        while(1);
    }
    
    // Config RX/TX Switching Pins (E28 specific)
    LT.setRXENPin(RX_EN_PIN);
    LT.setTXENPin(TX_EN_PIN);
}

// ================================================================
// --- 4. HELPER FUNCTIONS ---
// ================================================================

// Map ADC (0-4095) to RC (1000-2000)
int16_t readChannel(int pin, bool invert) {
    int raw = analogRead(pin);
    int16_t val = map(raw, 0, 4095, 1000, 2000);
    if(val < 1000) val = 1000;
    if(val > 2000) val = 2000;
    return invert ? (3000 - val) : val;
}

void drawDashboard() {
    display.clearDisplay();

    // --- TOP BAR (Local Battery %) ---
    // 1. Read Voltage
    // ADC 4095 = 3.3V Pin Voltage. 
    // Divider is 10k/10k (Divide by 2), so Battery Voltage = Pin Voltage * 2
    float gs_volt = (analogRead(PIN_BATTERY) / 4095.0) * 3.3 * 2.0;
    
    // 2. Convert to Percentage (Linear mapping for 1S LiPo: 3.3V to 4.2V)
    int bat_pct = (int) ((gs_volt - 3.3) / (4.2 - 3.3) * 100.0);
    
    // 3. Constrain to 0-100%
    if(bat_pct > 100) bat_pct = 100;
    if(bat_pct < 0) bat_pct = 0;
    
    display.setTextSize(1);
    display.setCursor(0,0);
    display.print("BATTERY: "); display.print(bat_pct); display.print("%");

    // --- CENTER (Arm Status) ---
    display.setCursor(35, 20);
    display.setTextSize(2);
    if(local_armed_state) {
        display.print("ARMED");
    } else {
        display.print("DISARMED");
    }

    // --- BOTTOM (Stick Debug) ---
    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print("Throttle:"); display.print(packet.throttle);
    display.print(" Roll:"); display.print(packet.roll);
    display.print(" Pitch:"); display.print(packet.pitch);
    
    display.display();
}

// ================================================================
// --- 5. MAIN LOOP ---
// ================================================================
void loop() {
    // 1. READ INPUTS
    // Note: Change 'false' to 'true' if any stick is reversed
    packet.yaw        = readChannel(PIN_JOY_YAW, false);
    packet.throttle   = readChannel(PIN_JOY_THR, false); 
    packet.roll       = readChannel(PIN_JOY_ROLL, false);
    packet.pitch      = readChannel(PIN_JOY_PITCH, false);
    packet.knob_pitch = readChannel(PIN_KNOB_PITCH, false);
    packet.knob_roll  = readChannel(PIN_KNOB_ROLL, false);

    // Read Buttons (Active LOW -> Set Bit High)
    packet.buttons = 0;
    if(digitalRead(PIN_BTN_UP) == LOW)     packet.buttons |= 0x01;
    if(digitalRead(PIN_BTN_DOWN) == LOW)   packet.buttons |= 0x02;
    if(digitalRead(PIN_BTN_ARM) == LOW)    packet.buttons |= 0x04;
    if(digitalRead(PIN_BTN_DISARM) == LOW) packet.buttons |= 0x08;

    // Update Local Status for Display
    if(packet.buttons & 0x04) local_armed_state = true;
    if(packet.buttons & 0x08) local_armed_state = false;

    // 2. TRANSMIT CYCLE (50Hz - every 20ms)
    if (millis() - last_tx_time > 20) {
        // Send packet blindly (No ACK required)
        LT.transmit((uint8_t*)&packet, sizeof(packet), 0, TX_POWER, WAIT_TX);
        last_tx_time = millis();
    }

    // 3. UPDATE DISPLAY (10Hz)
    static unsigned long last_ui_time = 0;
    if (millis() - last_ui_time > 100) {
        drawDashboard();
        last_ui_time = millis();
    }
}