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
// const uint64_t TIMER_MOTION_INTERVAL_US = 30000;     // interval : 30ms
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


typedef enum {
    FrontL = 0,
    FrontR,
    CenterL,
    CenterR,
    BackL,
    BackR,
    NUM_LEGS
} LegLayout;

typedef enum {
    Leg = 0,
    Foot,
    NUM_LEG_STRUCTURE
} LegStructure;

typedef enum {
    // 仮定：おそらく上側であるLegから角度を決めて、
    // その後にFootを決めてやったほうがmotionが安定しそうに思う
    FL_Leg = 0,
    FR_Leg,
    CL_Leg,
    CR_Leg,
    BL_Leg,
    BR_Leg,
    FL_Foot,
    FR_Foot,
    CL_Foot,
    CR_Foot,
    BL_Foot,
    BR_Foot,
    NUM_SERVOS
} ServoAssign;

void setServoAngle(uint8_t ch, uint8_t angle_deg)
{
    // TODO: DEBUG
    // Hardware Limit
    if(angle_deg < 0 && 180 < angle_deg) {
        Serial.printf("[Warn] HW Limit, L%02d over angle:%d [deg]\n", __LINE__, ch, angle_deg);
        return;
    }
    // Software Limit
    if(angle_deg < 45 && 135 < angle_deg) {
        Serial.printf("[Warn] SW Limit, L%02d over angle:%d [deg]\n", __LINE__, ch, angle_deg);
        return;
    }

    uint8_t ch_pahub = ch / NUM_SERVOS_PER_UNIT;
    uint8_t ch_servo = ch % NUM_SERVOS_PER_UNIT;
    // Serial.printf("[Debug] L%d ch_pahub:%d, ch_servo:%d\n", __LINE__,ch_pahub, ch_servo);

    i2cMux.setPort(ch_pahub);
    // Serial.printf("[Info] L%d, CH_PaHUB: %d\n", __LINE__, ch_pahub);

    unit_8servo[ch_pahub].setServoAngle(ch_servo, angle_deg);
    // Serial.printf("[Info] L%d, CH:%d DEG: %d\n", __LINE__, ch_servo, angle_deg);
}

typedef enum {
    MP_MotionDebug = 0,
    MP_MotionOrigin,
    MP_MotionIdle,
    MP_MotionStretching,
    MP_MotionWalk,
    Num_MotionPattern
} MotionPattern;

static uint8_t motionPattern = (uint8_t) MotionPattern::MP_MotionOrigin;
void MotionOrigin()
{
    setServoAngle(FL_Leg,  90);
    setServoAngle(FR_Leg,  90);
    setServoAngle(CL_Leg,  90);
    setServoAngle(CR_Leg,  90);
    setServoAngle(BL_Leg,  90);
    setServoAngle(BR_Leg,  90);
    setServoAngle(FL_Foot, 90);
    setServoAngle(FR_Foot, 90);
    setServoAngle(CL_Foot, 90);
    setServoAngle(CR_Foot, 90);
    setServoAngle(BL_Foot, 90);
    setServoAngle(BR_Foot, 90);
}

void MotionIdle()
{
    setServoAngle(FL_Leg,  90);
    setServoAngle(FR_Leg,  90);
    setServoAngle(CL_Leg,  90);
    setServoAngle(CR_Leg,  90);
    setServoAngle(BL_Leg,  90);
    setServoAngle(BR_Leg,  90);
    setServoAngle(FL_Foot, 90 + 10);
    setServoAngle(FR_Foot, 90 + 10);
    setServoAngle(CL_Foot, 90 + 10);
    setServoAngle(CR_Foot, 90 + 10);
    setServoAngle(BL_Foot, 90 + 10);
    setServoAngle(BR_Foot, 90 + 10);
}

static uint8_t count;
static int8_t polarity = +1;
void MotionStretching()
{
    const uint8_t step = 3;
    const uint8_t count_max = 11;
    for(int i=0; i<NUM_SERVOS; i++) {
        uint8_t angle = (90 - step*count_max/2) + step * count;
        Serial.printf("[Info] (i, count, angle) = (%2d, %2d, %3d)\n", i, count, angle);
        setServoAngle(i, angle);
    }
    if(count<=0) {
        polarity = +1;
    } else if(count>=count_max) {
        polarity = -1;
    }
    if(polarity == +1) {
        count++;
    } else {
        count--;
    }
}

float pattern_walk[][1+NUM_LEG_STRUCTURE*NUM_LEGS] = {
    { 0 ,90.0,91.812,78.544,99.223,90.0,91.812,106.699,91.097,90.0,91.812,83.21,90.212 },
    { 1 ,88.921,90.917,78.574,99.204,90.549,91.232,106.668,91.1,90.531,90.632,83.221,90.208 },
    { 2 ,87.847,90.025,78.663,99.148,91.113,90.688,106.575,91.107,91.043,89.485,83.252,90.199 },
    { 3 ,86.783,89.152,78.81,99.055,91.692,90.192,106.42,91.119,91.535,88.388,83.305,90.183 },
    { 4 ,85.733,88.311,79.013,98.925,92.284,89.756,106.203,91.136,92.007,87.357,83.379,90.162 },
    { 5 ,84.704,87.517,79.271,98.76,92.885,89.389,105.924,91.157,92.457,86.406,83.474,90.137 },
    { 6 ,83.698,86.782,79.58,98.56,93.494,89.103,105.586,91.182,92.885,85.547,83.591,90.109 },
    { 7 ,82.721,86.119,79.938,98.328,94.107,88.904,105.187,91.211,93.291,84.793,83.729,90.081 },
    { 8 ,81.776,85.539,80.341,98.066,94.722,88.799,104.729,91.245,93.675,84.152,83.889,90.054 },
    { 9 ,80.868,85.053,80.785,97.775,95.334,88.795,104.214,91.281,94.036,83.633,84.07,90.03 },
    { 10 ,80.0,84.668,81.265,97.458,95.941,88.893,103.642,91.32,94.374,83.239,84.274,90.012 },
    { 11 ,79.174,84.393,81.777,97.118,96.537,89.096,103.016,91.362,94.689,82.974,84.499,90.001 },
    { 12 ,78.395,84.231,82.318,96.757,97.119,89.402,102.336,91.406,94.982,82.839,84.747,90.001 },
    { 13 ,77.664,84.186,82.881,96.38,97.682,89.808,101.605,91.451,95.253,82.834,85.018,90.014 },
    { 14 ,76.984,84.258,83.463,95.989,98.223,90.309,100.826,91.496,95.501,82.953,85.311,90.042 },
    { 15 ,76.358,84.445,84.059,95.588,98.735,90.899,100.0,91.541,95.726,83.194,85.626,90.086 },
    { 16 ,75.786,84.745,84.666,95.18,99.215,91.568,99.132,91.585,95.93,83.549,85.964,90.151 },
    { 17 ,75.271,85.151,85.278,94.77,99.659,92.306,98.224,91.627,96.111,84.01,86.325,90.236 },
    { 18 ,74.813,85.657,85.893,94.362,100.062,93.101,97.279,91.667,96.271,84.569,86.709,90.344 },
    { 19 ,74.414,86.254,86.506,93.958,100.42,93.942,96.302,91.703,96.409,85.214,87.115,90.476 },
    { 20 ,74.076,86.931,87.115,93.562,100.729,94.816,95.296,91.735,96.526,85.934,87.543,90.634 },
    { 21 ,73.797,87.677,87.716,93.177,100.987,95.71,94.267,91.762,96.621,86.717,87.993,90.817 },
    { 22 ,73.58,88.48,88.308,92.808,101.19,96.61,93.217,91.783,96.695,87.55,88.465,91.027 },
    { 23 ,73.425,89.327,88.887,92.455,101.337,97.504,92.153,91.799,96.748,88.419,88.957,91.263 },
    { 24 ,73.332,90.204,89.451,92.123,101.426,98.379,91.079,91.809,96.779,89.311,89.469,91.525 },
    { 25 ,73.301,91.097,90.0,91.812,101.456,99.223,450.0,91.812,96.79,90.212,90.0,91.812 },
    { 26 ,73.332,91.1,89.451,91.232,101.426,99.204,91.079,90.917,96.779,90.208,89.469,90.632 },
    { 27 ,73.425,91.107,88.887,90.688,101.337,99.148,92.153,90.025,96.748,90.199,88.957,89.485 },
    { 28 ,73.58,91.119,88.308,90.192,101.19,99.055,93.217,89.152,96.695,90.183,88.465,88.388 },
    { 29 ,73.797,91.136,87.716,89.756,100.987,98.925,94.267,88.311,96.621,90.162,87.993,87.357 },
    { 30 ,74.076,91.157,87.115,89.389,100.729,98.76,95.296,87.517,96.526,90.137,87.543,86.406 },
    { 31 ,74.414,91.182,86.506,89.103,100.42,98.56,96.302,86.782,96.409,90.109,87.115,85.547 },
    { 32 ,74.813,91.211,85.893,88.904,100.062,98.328,97.279,86.119,96.271,90.081,86.709,84.793 },
    { 33 ,75.271,91.245,85.278,88.799,99.659,98.066,98.224,85.539,96.111,90.054,86.325,84.152 },
    { 34 ,75.786,91.281,84.666,88.795,99.215,97.775,99.132,85.053,95.93,90.03,85.964,83.633 },
    { 35 ,76.358,91.32,84.059,88.893,98.735,97.458,100.0,84.668,95.726,90.012,85.626,83.239 },
    { 36 ,76.984,91.362,83.463,89.096,98.223,97.118,100.826,84.393,95.501,90.001,85.311,82.974 },
    { 37 ,77.664,91.406,82.881,89.402,97.682,96.757,101.605,84.231,95.253,90.001,85.018,82.839 },
    { 38 ,78.395,91.451,82.318,89.808,97.119,96.38,102.336,84.186,94.982,90.014,84.747,82.834 },
    { 39 ,79.174,91.496,81.777,90.309,96.537,95.989,103.016,84.258,94.689,90.042,84.499,82.953 },
    { 40 ,80.0,91.541,81.265,90.899,95.941,95.588,103.642,84.445,94.374,90.086,84.274,83.194 },
    { 41 ,80.868,91.585,80.785,91.568,95.334,95.18,104.214,84.745,94.036,90.151,84.07,83.549 },
    { 42 ,81.776,91.627,80.341,92.306,94.722,94.77,104.729,85.151,93.675,90.236,83.889,84.01 },
    { 43 ,82.721,91.667,79.938,93.101,94.107,94.362,105.187,85.657,93.291,90.344,83.729,84.569 },
    { 44 ,83.698,91.703,79.58,93.942,93.494,93.958,105.586,86.254,92.885,90.476,83.591,85.214 },
    { 45 ,84.704,91.735,79.271,94.816,92.885,93.562,105.924,86.931,92.457,90.634,83.474,85.934 },
    { 46 ,85.733,91.762,79.013,95.71,92.284,93.177,106.203,87.677,92.007,90.817,83.379,86.717 },
    { 47 ,86.783,91.783,78.81,96.61,91.692,92.808,106.42,88.48,91.535,91.027,83.305,87.55 },
    { 48 ,87.847,91.799,78.663,97.504,91.113,92.455,106.575,89.327,91.043,91.263,83.252,88.419 },
    { 49 ,88.921,91.809,78.574,98.379,90.549,92.123,106.668,90.204,90.531,91.525,83.221,89.311 },
};

int count_max = sizeof(pattern_walk) / sizeof(pattern_walk[0]);

void MotionWalk()
{
    Serial.printf("[DEBUG] %d, ", int(pattern_walk[count][0]));
    for(int leg=0; leg<NUM_LEGS;leg++) {
        for(int joint=0; joint<NUM_LEG_STRUCTURE; joint++) {
            Serial.printf("%.3f,", pattern_walk[count][1+joint+leg*NUM_LEG_STRUCTURE]);
            setServoAngle(joint+leg*NUM_LEG_STRUCTURE, pattern_walk[count][1+joint+leg*NUM_LEG_STRUCTURE]);
        }
    }
    Serial.printf("\n");
 
    count++;
    if(count >= count_max)
        count = 0;
}

void gui_disp_pattern(uint8_t pattern)
{
    std::string str_pattern = "";
    switch (pattern)
    {
    case MotionPattern::MP_MotionDebug:
        str_pattern = "motion debug";
        break;
    case MotionPattern::MP_MotionIdle:
        str_pattern = "motion idle";
        break;
    case MotionPattern::MP_MotionOrigin:
        str_pattern = "motion origin";
        break;
    case MotionPattern::MP_MotionStretching:
        str_pattern = "motion stretching";
        break;
    case MotionPattern::MP_MotionWalk:
        str_pattern = "motion walk";
        break;
    default:
        str_pattern = "unknown motion";
        break;
    }

    int pos_x = 7*2;
    int pos_y = 7 + 7 + (14*2)*2+14*2+(14);
    M5.Display.startWrite();    // Occupies the SPI bus to speed up drawing
        M5.Display.setTextColor(GOLD);
        M5.Display.setTextSize(2);  // 14*2
        M5.Display.fillRect(pos_x, pos_y, 320, 14*2, BLACK);    // clear
        M5.Display.drawString(str_pattern.c_str(), pos_x, pos_y);
    M5.Display.endWrite();
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
        M5.Display.setTextSize(1);  // 14
        M5.Display.drawString(APP_VERSION, M5.Lcd.width() - 14/2 * 8, 7);     // 8 characters in "ver.1.0 "
        M5.Display.drawString("- push BtnA to pause motion", 7*2, 7 + (14*2)*2);
        M5.Display.drawString("- push BtnB to select motion", 7*2, 7 + (14*2)*2+14);
        M5.Display.drawString("- push BtnC to restart motion", 7*2, 7 + (14*2)*2+14*2);
        M5.Display.drawRect( 20, 220, 80, 20, GOLD);
        M5.Display.drawString(" A:PAUSE ", 20+5, 220+2);
        M5.Display.drawRect(120, 220, 80, 20, GOLD);
        M5.Display.drawString("B:PATTERN", 120+5, 220+2);
        M5.Display.drawRect(220, 220, 80, 20, GOLD);
        M5.Display.drawString(" C:PLAY  ", 220+5, 220+2);
    M5.Display.endWrite();

    gui_disp_pattern(motionPattern);

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
    //   BtnB : select motion pattern
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
    if(M5.BtnB.wasPressed()) {
        // select motion pattern
        Serial.printf("[Info] L%d, BtnB was pressed.\n", __LINE__);
        motionPattern++;
        if(motionPattern >= MotionPattern::Num_MotionPattern) {
            motionPattern = 0;
        }
        gui_disp_pattern(motionPattern);
        beep();
    }

    if(motion_trigger) {
        switch(motionPattern) {
            case MotionPattern::MP_MotionOrigin:
                MotionOrigin();
                break;
            case MotionPattern::MP_MotionIdle:
                MotionIdle();
                break;
            case MotionPattern::MP_MotionStretching:
                MotionStretching();
                break;
            case MotionPattern::MP_MotionWalk:
                MotionWalk();
                break;
            default:
                break;
        }
        motion_trigger = false; // reset trigger
    }

#if 0   // demo
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
#endif
    vTaskDelay(50);
}
