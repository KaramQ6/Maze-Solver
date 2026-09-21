/**
 * @file esp32_quick_test.ino
 * @brief Rapid Hardware Smoke-Test & I2C Scanner for DOIT ESP32 DevKit V1.
 * 
 * Functions:
 * 1. Blinks the onboard blue LED on GPIO 2.
 * 2. Continuously scans the I2C bus on GPIO 21 (SDA) and GPIO 22 (SCL).
 * 3. Monitors the Start Button on GPIO 13.
 */

#include <Arduino.h>
#include <Wire.h>

#define LED_PIN     2   /* Onboard Blue LED on DOIT DevKit V1 */
#define BUTTON_PIN  13  /* Start Button input */
#define SDA_PIN     21  /* Default ESP32 I2C Data */
#define SCL_PIN     22  /* Default ESP32 I2C Clock */

void scan_i2c() {
    Serial.println("\n--- Scanning I2C Bus (SDA=21, SCL=22) ---");
    byte count = 0;

    for (byte address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0) {
            Serial.printf("[FOUND] I2C device detected at address 0x%02X", address);
            if (address == 0x68) Serial.print(" (MPU6050 Gyro/Accel)");
            else if (address == 0x29) Serial.print(" (VL53L0X Default / Front)");
            else if (address == 0x30) Serial.print(" (VL53L0X Left)");
            else if (address == 0x31) Serial.print(" (VL53L0X Right)");
            Serial.println();
            count++;
        } else if (error == 4) {
            Serial.printf("[ERROR] Unknown error at address 0x%02X\n", address);
        }
    }

    if (count == 0) {
        Serial.println("No I2C devices attached yet (normal if sensors are not wired).");
    } else {
        Serial.printf("Done. Total %d device(s) found.\n", count);
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    delay(1000);

    Serial.println("\n==========================================");
    Serial.println("  DOIT ESP32 DevKit V1 - Hardware Test    ");
    Serial.println("==========================================");
    Serial.println("If you can read this, USB Serial & Flash are 100% OK!");

    Wire.begin(SDA_PIN, SCL_PIN, 400000);
}

void loop() {
    /* Blink onboard LED */
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(300);

    /* Check button */
    if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("[BUTTON] Start Button (GPIO 13) is PRESSED!");
    }

    static unsigned long last_scan = 0;
    if (millis() - last_scan > 4000) {
        last_scan = millis();
        scan_i2c();
    }
}
