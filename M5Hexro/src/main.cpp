#include <M5Unified.h>

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

#define ASSERT(condition, line) \
    if (!(condition)) { \
        Serial.printf("[ERROR] Assertion failed: %d at L%d", #condition, #line);\
        while(1);\
    }

// Sound
#define TONE_C5 523.251
#define TONE_E5 659.255
#define TONE_G5 783.991
#define TONE_C6 (TONE_C5 * 2)
#define TONE_E6 (TONE_E5 * 2)
#define TONE_G6 (TONE_G5 * 2)

// for debug
#define MONITOR_ENABLE
// Serial.printf("[Info] %d\n", __LINE__);

// PA HUB
QWIICMUX i2cMux;

// 8Servo
#ifdef MONITOR_ENABLE
M5GFX display;
M5Canvas canvas(&display);
#endif
M5_UNIT_8SERVO unit_8servo[NUM_SERVO_UNITS_MAX];

// application
#define ANGLE_LIST_SIZE 5
uint8_t angle_list[ANGLE_LIST_SIZE] = {0, 45, 90, 135, 180};// for demo
uint8_t i_angle = 0;

// --
#define ID_TIMER_MOTION 0
const uint64_t TIMER_MOTION_INTERBAL_US = 500000; // interval : 500ms
hw_timer_t *timerMotion = NULL;
void IRAM_ATTR onTimerMotion()
{
    int time_ms = millis();

    Serial.println("[Info] onTimerMotion, time_ms:" + String(time_ms));

#if 0
    for(uint8_t ch_pahub = 0; ch_pahub < NUM_CHANNELS_PAHUB; ch_pahub++) {
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

#ifdef MONITOR_ENABLE
                    // canvas.drawRect(0, servo_ch * 20 + 75, 200, 15, 1);
                    // canvas.fillRect(0, servo_ch * 20 + 75, map(angle_list[i_angle], 0, 180, 0, 200), 15, 1);
                    // canvas.setCursor(220, servo_ch * 28 + 10);
                    // canvas.setTextSize(1);
                    // canvas.printf("CH:%d DEG: %d\n", servo_ch, angle_list[i_angle]);
#endif
                }
                break;
            default:
                break;
        }
        i_angle++;
        i_angle = i_angle % ANGLE_LIST_SIZE;

#ifdef MONITOR_ENABLE
        // canvas.pushSprite(0, 0);
#endif
        // vTaskDelay(50);    // 500
    }
#endif
}

void beep()
{
    M5.Speaker.tone(TONE_E5, 200);
}

void beep_init_done()
{
    M5.Speaker.tone(TONE_C5, 500);
    M5.Speaker.tone(TONE_E5, 500);
    M5.Speaker.tone(TONE_G5, 500);
}

// ----
void setup()
{
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Power.begin();
    delay(100);

    // I2C init
    Wire.begin(PIN_PA_SDA, PIN_PA_SCL);
    if (!i2cMux.begin(0x70, Wire)) {
        Serial.println("[Error] PaHUB not detected. Freezing...");
        ASSERT(0, __LINE__);
    }
    Serial.println("[Info] PaHUB detected. Begin scanning for I2C devices");

    // 8Servo init
#ifdef MONITOR_ENABLE
    display.begin();
    canvas.setColorDepth(1);  // mono color
    canvas.setFont(&fonts::efontCN_14);
    canvas.createSprite(display.width(), display.height());
    canvas.setPaletteColor(1, GREEN);
#endif

    for(uint8_t ch_pahub = 0; ch_pahub < NUM_CHANNELS_PAHUB; ch_pahub++)
    {
        // set PA_HUB channel
        if(!i2cMux.setPort(ch_pahub)) {
            Serial.printf("[Error] PaHUB set Port %d error.\n", ch_pahub);
            ASSERT(0, __LINE__);
        } else {
            Serial.printf("[Info] PaHUB set Port %d done.\n", ch_pahub);
        }

        switch(ch_pahub) {
            case 0: // for 8servo unit 0
            case 1: // for 8servo unit 1
                if (!unit_8servo[ch_pahub].begin(&Wire, PIN_PA_SDA, PIN_PA_SCL, M5_UNIT_8SERVO_DEFAULT_ADDR)) {
                    Serial.printf("[Error] L%d, ch_pahub %d\n", __LINE__, ch_pahub);
                    Serial.println("[Error] extio Connect Error\n");
                    M5.Lcd.print("[Error] extio Connect Error\n");
                    ASSERT(0, __LINE__);
                }
                unit_8servo[ch_pahub].setAllPinMode(SERVO_CTL_MODE);
                break;
            default:
                break;
        }
    }

    // timer must start after device init done
    timerMotion = timerBegin(ID_TIMER_MOTION, 80, true);   // divider:80 for 1us count
    timerAttachInterrupt(timerMotion, &onTimerMotion, true);
    timerAlarmWrite(timerMotion, TIMER_MOTION_INTERBAL_US, true);
    timerAlarmEnable(timerMotion);

    // init done
    Serial.printf("[Info] init done.\n");
    beep_init_done();
}



// ----
void loop()
{
    M5.update();

    // Button Event
    //   BtnA : stop timerMotion
    //   BtnC : restart timerMotion
    if(M5.BtnA.wasPressed()) {
        Serial.printf("[Info] L%d, BtnA was pressed.\n", __LINE__);
        timerAlarmDisable(timerMotion);
        beep();
        while(1) {
            // pause
            M5.update();
            if(M5.BtnC.wasPressed()) {
                // restart
                timerAlarmEnable(timerMotion);
                beep();
                Serial.printf("[Info] L%d, BtnC was pressed.\n", __LINE__);
                break;
            }
        }
    }

    // 8SERVO
#ifdef MONITOR_ENABLE
    canvas.fillSprite(0);
    canvas.setTextSize(2);
    canvas.drawString("SERVO CTL MODE", 10, 10);
    // canvas.drawString("FW VERSION: " + String(unit_8servo[].getVersion()), 10, 40);
#endif

#if 0 // TODO: port to timer interrupt
    for(uint8_t ch_pahub = 0; ch_pahub < NUM_CHANNELS_PAHUB; ch_pahub++) {
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

#ifdef MONITOR_ENABLE
                    canvas.drawRect(0, servo_ch * 20 + 75, 200, 15, 1);
                    canvas.fillRect(0, servo_ch * 20 + 75, map(angle_list[i_angle], 0, 180, 0, 200), 15, 1);
                    canvas.setCursor(220, servo_ch * 28 + 10);
                    canvas.setTextSize(1);
                    canvas.printf("CH:%d DEG: %d\n", servo_ch, angle_list[i_angle]);
#endif
                }
                break;
            default:
                break;
        }
        i_angle++;
        i_angle = i_angle % ANGLE_LIST_SIZE;

#ifdef MONITOR_ENABLE
        canvas.pushSprite(0, 0);
#endif
        vTaskDelay(50);    // 500
    }
#endif

    vTaskDelay(50);
}
