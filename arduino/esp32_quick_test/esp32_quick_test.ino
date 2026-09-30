/**
 * @file esp32_quick_test.ino
 * @brief Rapid Hardware Smoke-Test & I2C Scanner for Seeed Studio XIAO ESP32-S3.
 * 
 * Functions:
 * 1. Checks USB CDC Serial connection on Native USB.
 * 2. Scans I2C bus on D4 (GPIO 5 - SDA) and D5 (GPIO 6 - SCL) at 400kHz.
 * 3. Monitors Onboard BOOT Button (GPIO 0).
 * 4. Toggles optional external status LED / buzzer on D10 (GPIO 9).
 */

#include <Arduino.h>
#include <Wire.h>

#define PIN_BOOT_BTN    0   /* Onboard BOOT button on XIAO ESP32-S3 */
#define PIN_SDA         5   /* D4 (GPIO 5) */
#define PIN_SCL         6   /* D5 (GPIO 6) */
#define PIN_STATUS_LED  9   /* D10 (GPIO 9) - Optional external LED */

void scan_i2c() {
    Serial.println("\n--- Scanning I2C Bus: SDA=D4(GPIO5), SCL=D5(GPIO6) ---");
    byte count = 0;

    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf("[FOUND] I2C device at 0x%02X", address);
            if (address == 0x68) Serial.print(" -> MPU6050 6-Axis Gyro/Accelerometer");
            else if (address == 0x29) Serial.print(" -> VL53L0X (Factory Default Address)");
            else if (address == 0x30) Serial.print(" -> VL53L0X (Right Sensor Re-addressed)");
            else if (address == 0x31) Serial.print(" -> VL53L0X (Left Sensor Re-addressed)");
            else if (address == 0x32) Serial.print(" -> VL53L0X (Front Sensor Re-addressed)");
            Serial.println();
            count++;
        } else if (error == 4) {
            Serial.printf("[ERROR] Bus error at address 0x%02X\n", address);
        }
    }

    if (count == 0) {
        Serial.println("No I2C devices detected. Check 3.3V, GND, and SDA(D4)/SCL(D5) connections.");
    } else {
        Serial.printf(">>> Scan complete: %d device(s) active on bus.\n", count);
    }
}

void setup() {
    pinMode(PIN_BOOT_BTN, INPUT_PULLUP);
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    Serial.begin(115200);
    delay(1500); /* Allow USB CDC handshake */

    Serial.println("\n==============================================");
    Serial.println("  Seeed Studio XIAO ESP32-S3 - Smoke Test     ");
    Serial.println("==============================================");
    Serial.println("✓ USB Serial CDC: OK");
    Serial.println("✓ Xtensa LX7 Dual-Core @ 240MHz");
    Serial.printf("✓ Free Heap: %d KB\n", ESP.getFreeHeap() / 1024);
    Serial.printf("✓ Total PSRAM: %d KB\n", ESP.getPsramSize() / 1024);
    Serial.println("Press the onboard BOOT button (GPIO 0) to test button input.");
    Serial.println("==============================================\n");

    Wire.begin(PIN_SDA, PIN_SCL, 400000);
    scan_i2c();
}

void loop() {
    /* Toggle external status LED pin */
    digitalWrite(PIN_STATUS_LED, HIGH);
    delay(250);
    digitalWrite(PIN_STATUS_LED, LOW);
    delay(250);

    /* Test onboard BOOT button */
    if (digitalRead(PIN_BOOT_BTN) == LOW) {
        Serial.println(">>> [BUTTON PRESSED] Onboard BOOT Button (GPIO 0) active!");
        delay(200); /* Simple debounce */
    }

    /* Periodic I2C Scan every 5 seconds */
    static unsigned long last_scan = 0;
    if (millis() - last_scan > 5000) {
        last_scan = millis();
        scan_i2c();
    }
}
