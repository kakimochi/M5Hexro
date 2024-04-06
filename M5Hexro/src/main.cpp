#include <M5Unified.h>
// #include <M5Stack.h>

#define DEBUG_PA

#include <Wire.h>

#ifdef DEBUG_PA
#include <SparkFun_I2C_Mux_Arduino_Library.h> // http://librarymanager/All#SparkFun_I2C_Mux https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library
#endif

#include <M5GFX.h>
#include "M5_UNIT_8SERVO.h"

#define PIN_PA_SDA 32   // for Core2, 26 for M5Stack Basic
#define PIN_PA_SCL 33   // for Core2, 27 for M5Stack Basic

#define NUM_SERVO_UNITS_MAX 2
#define NUM_SERVOS_PER_UNIT 8
#define NUM_CHANNELS_PAHUB 6

#ifdef DEBUG_PA
// PA HUB
QWIICMUX i2cMux;
#endif

// 8SERVO
M5GFX display;
M5Canvas canvas(&display);
M5_UNIT_8SERVO unit_8servo[NUM_SERVO_UNITS_MAX];
char info[50];

// Error
uint8_t error_connection = 0;

// ----
void setup()
{
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Power.begin();

#ifdef DEBUG_PA
    // Serial.begin(115200);
    delay(100);
    Wire.begin(PIN_PA_SDA, PIN_PA_SCL);
    if (i2cMux.begin(0x70, Wire) == false)
    {
        Serial.println("[Error] PaHUB not detected. Freezing...");
        while (1)
            ;
    }
    Serial.println("[Info] PaHUB detected. Begin scanning for I2C devices");
#endif

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
            case 0: // for 8servo 0
            case 1: // for 8servo 1
                // connect the module that connected PA_HUB
                Wire.beginTransmission(M5_UNIT_8SERVO_DEFAULT_ADDR);
                Serial.printf("[Info] L%d, ch_pahub %d\n", __LINE__, ch_pahub);
                if(&unit_8servo[ch_pahub] == NULL) {
                    Serial.printf("[Error] L%d, ch_pahub %d\n", __LINE__, ch_pahub);
                }

                while (!unit_8servo[ch_pahub].begin(&Wire, PIN_PA_SDA, PIN_PA_SCL, M5_UNIT_8SERVO_DEFAULT_ADDR)) {
                    Serial.printf("[Info] L%d, ch_pahub %d\n", __LINE__, ch_pahub);
                    if(error_count < ERROR_COUNT_MAX) {
                        Serial.println("[Error] extio Connect Error\n");
                        M5.Lcd.print("[Error] extio Connect Error\n");
                        error_count++;
                    }
                    delay(100);
                }
                unit_8servo[ch_pahub].setAllPinMode(SERVO_CTL_MODE);
                Serial.printf("[Info] %d\n", __LINE__);

                // close connection
                error_connection = Wire.endTransmission();
                if (error_connection == 0) {
                    Serial.printf("[Info] PaHUB CH:%d, 0X%X\n", ch_pahub, M5_UNIT_8SERVO_DEFAULT_ADDR);
                } else {
                    Serial.printf("[Error] PaHUB CH:%d, 0X%X\n", ch_pahub, M5_UNIT_8SERVO_DEFAULT_ADDR);
                }
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

#ifdef DEBUG_PA
    // PA HUB
    // for (uint8_t channel = 0; channel < 8; channel++)
    // {
    //     Serial.printf("CH%d : ", channel);
    //     i2cMux.setPort(channel);
    //     Serial.print("I2C device = ");
    //     for (uint8_t address = 0x01; address < 0x7F; address++)
    //     {
    //         Wire.beginTransmission(address);
    //         uint8_t returnCode = Wire.endTransmission();
    //         if (returnCode == 0)
    //         {
    //             Serial.printf("0X%X ", address);
    //         }
    //     }
    //     Serial.println();
    // }
    // delay(1000);
#endif

    // 8SERVO
    canvas.fillSprite(0);
    canvas.setTextSize(2);
    canvas.drawString("SERVO CTL MODE", 10, 10);
    // canvas.drawString("FW VERSION: " + String(unit_8servo[].getVersion()), 10, 40);

    error_connection = 0;
    uint8_t deg_list[5] = {0, 45, 90, 135, 180};    // for demo
    uint8_t deg_i = 0;
    for(uint8_t ch_pahub = 0; ch_pahub < NUM_CHANNELS_PAHUB; ch_pahub++)
    {
        // set PA_HUB channel
        i2cMux.setPort(ch_pahub);
        Serial.printf("CH_PaHUB: %d\n", ch_pahub);

        switch(ch_pahub) {
            case 0: // for 8servo 0
                // open connection
                Wire.beginTransmission(M5_UNIT_8SERVO_DEFAULT_ADDR);
                for(uint8_t servo_ch = 0; servo_ch < NUM_SERVOS_PER_UNIT; servo_ch++) {
                    // set servo angle
                    unit_8servo[ch_pahub].setServoAngle(servo_ch, deg_list[deg_i]);
                    // monitor
                    Serial.printf("CH:%d DEG: %d\n", servo_ch, deg_list[deg_i]);

                    canvas.drawRect(0, servo_ch * 20 + 75, 200, 15, 1);
                    canvas.fillRect(0, servo_ch * 20 + 75, map(deg_list[deg_i], 0, 180, 0, 200), 15, 1);
                    canvas.setCursor(220, servo_ch * 28 + 10);
                    canvas.setTextSize(1);
                    canvas.printf("CH:%d DEG: %d\n", servo_ch, deg_list[deg_i]);
                }
                // close connection
                error_connection = Wire.endTransmission();
                if (error_connection == 0)
                {
                    Serial.printf("[Info] PaHUB CH:%d, 0X%X\n", ch_pahub, M5_UNIT_8SERVO_DEFAULT_ADDR);
                }
                break;
            case 1: // for 8servo 1
                // connect the module that connected PA_HUB
                Wire.beginTransmission(M5_UNIT_8SERVO_DEFAULT_ADDR);

                // close connection
                error_connection = Wire.endTransmission();
                if (error_connection == 0)
                {
                    Serial.printf("[Info] PaHUB CH:%d, 0X%X\n", ch_pahub, M5_UNIT_8SERVO_DEFAULT_ADDR);
                }
                break;
            default:
                break;
        }

        canvas.pushSprite(0, 0);
        vTaskDelay(500);
    }

    deg_i++;
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