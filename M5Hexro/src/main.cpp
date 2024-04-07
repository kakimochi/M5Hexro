#include <M5Unified.h>
#include <M5GFX.h>

#define DEBUG_PA

// PA HUB
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h> // http://librarymanager/All#SparkFun_I2C_Mux https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library

// 8Servo
#include "M5_UNIT_8SERVO.h"

#define PIN_PA_SDA 32   // for Core2, 26 for M5Stack Basic
#define PIN_PA_SCL 33   // for Core2, 27 for M5Stack Basic

#define NUM_SERVO_UNITS_MAX 2
#define NUM_SERVOS_PER_UNIT 8
#define NUM_CHANNELS_PAHUB 6

// PA HUB
QWIICMUX i2cMux;

// 8SERVO
M5GFX display;
M5Canvas canvas(&display);
M5_UNIT_8SERVO unit_8servo[NUM_SERVO_UNITS_MAX];
char info[50];

// application
#define ANGLE_LIST_SIZE 5
uint8_t angle_list[ANGLE_LIST_SIZE] = {0, 45, 90, 135, 180};// for demo
uint8_t i_angle = 0;

// for debug
// Serial.printf("[Info] %d\n", __LINE__);

// ----
void setup()
{
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Power.begin();
    delay(100);

    Wire.begin(PIN_PA_SDA, PIN_PA_SCL);
    if (i2cMux.begin(0x70, Wire) == false)
    {
        Serial.println("[Error] PaHUB not detected. Freezing...");
        while (1)
            ;
    }
    Serial.println("[Info] PaHUB detected. Begin scanning for I2C devices");

    // 8SERVO
    display.begin();
    canvas.setColorDepth(1);  // mono color
    canvas.setFont(&fonts::efontCN_14);
    canvas.createSprite(display.width(), display.height());
    canvas.setPaletteColor(1, GREEN);

    const int ERROR_COUNT_MAX = 3;
    int error_count = 0;

    for(uint8_t ch_pahub = 0; ch_pahub < NUM_CHANNELS_PAHUB; ch_pahub++)
    {
        // set PA_HUB channel
        if(i2cMux.setPort(ch_pahub)) {
            Serial.printf("[Info] PaHUB set Port %d done.\n", ch_pahub);
        } else {
            // error
            Serial.printf("[Error] PaHUB set Port %d error.\n", ch_pahub);
        }

        switch(ch_pahub) {
            case 0: // for 8servo unit 0
            case 1: // for 8servo unit 1
                while (!unit_8servo[ch_pahub].begin(&Wire, PIN_PA_SDA, PIN_PA_SCL, M5_UNIT_8SERVO_DEFAULT_ADDR)) {
                    if(error_count < ERROR_COUNT_MAX) {
                        Serial.printf("[Error] L%d, ch_pahub %d\n", __LINE__, ch_pahub);
                        Serial.println("[Error] extio Connect Error\n");
                        M5.Lcd.print("[Error] extio Connect Error\n");
                        error_count++;
                    }
                    delay(100);
                }
                unit_8servo[ch_pahub].setAllPinMode(SERVO_CTL_MODE);
                break;
            default:
                break;
        }
    }

    Serial.printf("[Info] init done.\n");
}



// ----
void loop()
{
    M5.update();

    // 8SERVO
    canvas.fillSprite(0);
    canvas.setTextSize(2);
    canvas.drawString("SERVO CTL MODE", 10, 10);
    // canvas.drawString("FW VERSION: " + String(unit_8servo[].getVersion()), 10, 40);

    for(uint8_t ch_pahub = 0; ch_pahub < NUM_CHANNELS_PAHUB; ch_pahub++)
    {
        // set PA_HUB channel
        i2cMux.setPort(ch_pahub);
        Serial.printf("[Info] L%d, CH_PaHUB: %d\n", __LINE__, ch_pahub);

        switch(ch_pahub) {
            case 0: // for 8servo unit 0
            case 1: // for 8servo unit 1
                for(uint8_t servo_ch = 0; servo_ch < NUM_SERVOS_PER_UNIT; servo_ch++) {
                    // set servo angle
                    unit_8servo[ch_pahub].setServoAngle(servo_ch, angle_list[i_angle]);
                    // monitor
                    Serial.printf("[Info] L%d, CH:%d DEG: %d\n", __LINE__, servo_ch, angle_list[i_angle]);

                    canvas.drawRect(0, servo_ch * 20 + 75, 200, 15, 1);
                    canvas.fillRect(0, servo_ch * 20 + 75, map(angle_list[i_angle], 0, 180, 0, 200), 15, 1);
                    canvas.setCursor(220, servo_ch * 28 + 10);
                    canvas.setTextSize(1);
                    canvas.printf("CH:%d DEG: %d\n", servo_ch, angle_list[i_angle]);
                }
                break;
            default:
                break;
        }
        i_angle++;
        i_angle = i_angle % ANGLE_LIST_SIZE;

        canvas.pushSprite(0, 0);
        vTaskDelay(50);    // 500
    }
}
