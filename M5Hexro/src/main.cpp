#include <M5Unified.h>
// #include <M5Stack.h>

// #define DEBUG_PA

#include <Wire.h>

#ifdef DEBUG_PA
#include <SparkFun_I2C_Mux_Arduino_Library.h> // http://librarymanager/All#SparkFun_I2C_Mux https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library
#endif

#include <M5GFX.h>
#include "M5_UNIT_8SERVO.h"

#ifdef DEBUG_PA
// PA HUB
QWIICMUX i2cMux;
#endif

// 8SERVO
M5GFX display;
M5Canvas canvas(&display);
M5_UNIT_8SERVO unit_8servo;
char info[50];

// ----
void setup()
{
#ifdef DEBUG_PA
    // PA HUB
    Serial.begin(115200);
    delay(100);
    Wire.begin(32, 33);
    if (i2cMux.begin(0x70, Wire) == false)
    {
        Serial.println("Mux not detected. Freezing...");
        while (1)
            ;
    }
    Serial.println("Mux detected");
    Serial.println("Begin scanning for I2C devices");
#endif

    // 8SERVO
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Power.begin();

    display.begin();
    canvas.setColorDepth(1);  // mono color
    canvas.setFont(&fonts::efontCN_14);
    canvas.createSprite(display.width(), display.height());
    canvas.setPaletteColor(1, GREEN);

    const int ERROR_COUNT_MAX = 3;
    int error_count = 0;
    // M5Stack Core2 PortB : 32
    // M5Stack Basic PortB : 26
    while (!unit_8servo.begin(&Wire, 32, 33, M5_UNIT_8SERVO_DEFAULT_ADDR)) {
        if(error_count < ERROR_COUNT_MAX) {
            Serial.println("extio Connect Error");
            M5.Lcd.print("extio Connect Error");
            error_count++;
        }
        delay(100);
    }
    unit_8servo.setAllPinMode(SERVO_CTL_MODE);
}



// ----
void loop()
{
    M5.update();

#ifdef DEBUG_PA
    // PA HUB
    for (uint8_t channel = 0; channel < 8; channel++)
    {
        Serial.printf("CH%d : ", channel);
        i2cMux.setPort(channel);
        Serial.print("I2C device = ");
        for (uint8_t address = 0x01; address < 0x7F; address++)
        {
            Wire.beginTransmission(address);
            uint8_t returnCode = Wire.endTransmission();
            if (returnCode == 0)
            {
                Serial.printf("0X%X ", address);
            }
        }
        Serial.println();
    }
    delay(1000);
#endif

    // 8SERVO
    canvas.fillSprite(0);
    canvas.setTextSize(2);
    canvas.drawString("SERVO CTL MODE", 10, 10);
    canvas.drawString("FW VERSION: " + String(unit_8servo.getVersion()), 10, 40);
    for (uint8_t deg = 0; deg <= 180; deg += 45) {
        for (uint8_t i = 0; i < 8; i++) {
            unit_8servo.setServoAngle(i, deg);
            Serial.printf("CH:%d DEG: %d", i, deg);
            canvas.drawRect(0, i * 20 + 75, 200, 15, 1);
            canvas.fillRect(0, i * 20 + 75, map(deg, 0, 180, 0, 200), 15, 1);
            canvas.setCursor(220, i * 28 + 10);
            canvas.setTextSize(1);
            canvas.printf("CH:%d DEG: %d", i, deg);
        }
        canvas.pushSprite(0, 0);
        vTaskDelay(500);
    }
    delay(100);
}

// Mux detected
// Begin scanning for I2C devices
// CH0 : I2C device = 0X25 0X70
// CH1 : I2C device = 0X25 0X70 
// CH2 : I2C device = 0X70 
// CH3 : I2C device = 0X70 
// CH4 : I2C device = 0X70 
// CH5 : I2C device = 0X70 
// CH6 : I2C device = 0X70 
// CH7 : I2C device = 0X70 