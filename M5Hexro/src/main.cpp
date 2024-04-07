#include <M5Unified.h>

// PA HUB
#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
// http://librarymanager/All#SparkFun_I2C_Mux
// https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library

// 8Servo
#include "M5_UNIT_8SERVO.h"

// Hardware Config
#define PIN_PA_SDA 32   // for Core2, 26 for M5Stack Basic
#define PIN_PA_SCL 33   // for Core2, 27 for M5Stack Basic

// GUI
#define APP_NAME "M5Hexro"
#define APP_VERSION "ver.1.0"

#define NUM_SERVOS_PER_UNIT 8
#define NUM_CHANNELS_PAHUB 6
#define NUM_SERVO_UNITS_MAX 2
#define ID_8SERVO_UNIT_0 0      // PA_HUB 0 : 8Servo Unit 0
#define ID_8SERVO_UNIT_1 1      // PA_HUB 1 : 8Servo Unit 1

// Utility
#if 0
typedef enum {
    DEBUG = 0,
    INFO,
    WARN,
    ERROR,
    CRITICAL,
} MessageLevel;
void logf(MessageLevel level, const char *format, ...) {
    va_list args;
    va_start(args, format);
    
    switch (level) {
        case DEBUG:
            Serial.printf("[DEBUG] ");
            break;
        case INFO:
            Serial.printf("[INFO] ");
            break;
        case WARN:
            Serial.printf("[WARN] ");
            break;
        case ERROR:
            Serial.printf("[ERROR] ");
            break;
        case CRITICAL:
            Serial.printf("[CRITICAL] ");
            break;
        default:
            break;
    }
    
    vprintf(format, args);
    va_end(args);
}
#endif

#define ASSERT(condition, line) \
    if (!(condition)) { \
        Serial.printf("[Error] Assertion failed: %d at L%d", #condition, #line);\
        while(1);\
    }

// Beep Sound
#define TONE_C5 523.251
#define TONE_E5 659.255
#define TONE_G5 783.991
#define TONE_C6 (TONE_C5 * 2)
#define TONE_E6 (TONE_E5 * 2)
#define TONE_G6 (TONE_G5 * 2)

// for debug
// #define MONITOR_MOTION_ENABLE
// Serial.printf("[Info] %d\n", __LINE__);

// Static Members
// PA HUB
QWIICMUX i2cMux;
// GUI
M5GFX display;
M5Canvas canvas(&display);
// 8Servo
M5_UNIT_8SERVO unit_8servo[NUM_SERVO_UNITS_MAX];

// for application
#define ANGLE_LIST_SIZE 5
uint8_t angle_list[ANGLE_LIST_SIZE] = {0, 45, 90, 135, 180}; // for demo
uint8_t i_angle = 0;

// Timer Interrupt
#define ID_TIMER_MOTION 0
const uint64_t TIMER_MOTION_INTERVAL_US = 500000; // interval : 500ms
hw_timer_t *timerMotion = NULL;
bool motion_trigger = false;
void IRAM_ATTR onTimerMotion()
{
    if(motion_trigger)  // block multiple events
        return;

    int time_ms = millis();

    Serial.printf("[Info] onTimerMotion, time_ms:%d\n", time_ms);
    motion_trigger = true;
}

// user functions
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
        Serial.printf("[Error] PaHUB not detected. Freezing...");
        ASSERT(0, __LINE__);
    }
    Serial.printf("[Info] PaHUB detected. Begin scanning for I2C devices");

    // 8Servo init
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
                    Serial.printf("[Error] extio Connect Error\n");
                    M5.Lcd.print("[Error] extio Connect Error\n");
                    ASSERT(0, __LINE__);
                }
                unit_8servo[ch_pahub].setAllPinMode(SERVO_CTL_MODE);
                break;
            default:
                break;
        }
    }

    // GUI
    M5.Display.begin();
    M5.Display.startWrite();    // Occupies the SPI bus to speed up drawing
        M5.Display.setColorDepth(1); // mono color
        M5.Display.fillScreen(BLACK);
        M5.Display.setFont(&fonts::efontCN_14);
        M5.Display.setTextColor(GOLD);
        M5.Display.setTextSize(2);  // 14*2
        M5.Display.drawString(APP_NAME, 7, 7);
        M5.Display.drawString(APP_VERSION, 7, 7 + (14*2));
        M5.Display.setTextSize(1);  // 14
        M5.Display.drawString("- push BtnA to pause motion.", 7, 7 + (14*2)*2);
        M5.Display.drawString("- push BtnC to restart motion.", 7, 7 + (14*2)*2+14);
    M5.Display.endWrite();

    // timer must start after all devices init done
    timerMotion = timerBegin(ID_TIMER_MOTION, 80, true);   // divider:80 for 1us count
    timerAttachInterrupt(timerMotion, &onTimerMotion, true);
    timerAlarmWrite(timerMotion, TIMER_MOTION_INTERVAL_US, true);
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
    //   BtnA : pause timerMotion
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
                Serial.printf("[Info] L%d, BtnC was pressed.\n", __LINE__);
                timerAlarmEnable(timerMotion);
                beep();
                break;
            }
        }
    }

    // 8SERVO
    if(motion_trigger) {
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
                        Serial.printf("[Info] L%d, CH:%d DEG: %d\n", __LINE__, servo_ch, angle_list[i_angle]);

    #ifdef MONITOR_MOTION_ENABLE
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

    #ifdef MONITOR_MOTION_ENABLE
            canvas.pushSprite(0, 0);
    #endif
            // vTaskDelay(5);
        }
        i_angle++;
        i_angle = i_angle % ANGLE_LIST_SIZE;

        motion_trigger = false; // reset trigger
    }

    vTaskDelay(50);
}
