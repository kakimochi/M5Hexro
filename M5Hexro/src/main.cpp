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
    NUM_ServoAssign
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
    Serial.printf("[Debug] L%d ch_pahub:%d, ch_servo:%d\n", __LINE__,ch_pahub, ch_servo);

    i2cMux.setPort(ch_pahub);
    // Serial.printf("[Info] L%d, CH_PaHUB: %d\n", __LINE__, ch_pahub);

    unit_8servo[ch_pahub].setServoAngle(ch_servo, angle_deg);
    Serial.printf("[Info] L%d, CH:%d DEG: %d\n", __LINE__, ch_servo, angle_deg);
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
    for(int i=0; i<NUM_ServoAssign; i++) {
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

float pattern_walk[][1+3*NUM_LEGS] = {
    { 1 ,88.921,90.917,-103.54,78.574,99.204,-113.651,90.549,91.232,-104.438,106.668,91.1,-100.144,90.531,90.632,-102.636,83.221,90.208,-84.904 },
    { 2 ,87.847,90.025,-104.4,78.663,99.148,-113.605,91.113,90.688,-106.194,106.575,91.107,-100.174,91.043,89.485,-102.582,83.252,90.199,-85.031 },
    { 3 ,86.783,89.152,-105.202,78.81,99.055,-113.527,91.692,90.192,-107.887,106.42,91.119,-100.223,91.535,88.388,-102.464,83.305,90.183,-85.241 },
    { 4 ,85.733,88.311,-105.935,79.013,98.925,-113.418,92.284,89.756,-109.502,106.203,91.136,-100.29,92.007,87.357,-102.277,83.379,90.162,-85.533 },
    { 5 ,84.704,87.517,-106.588,79.271,98.76,-113.277,92.885,89.389,-111.027,105.924,91.157,-100.375,92.457,86.406,-102.015,83.474,90.137,-85.905 },
    { 6 ,83.698,86.782,-107.153,79.58,98.56,-113.103,93.494,89.103,-112.448,105.586,91.182,-100.476,92.885,85.547,-101.673,83.591,90.109,-86.352 },
    { 7 ,82.721,86.119,-107.623,79.938,98.328,-112.895,94.107,88.904,-113.754,105.187,91.211,-100.591,93.291,84.793,-101.252,83.729,90.081,-86.873 },
    { 8 ,81.776,85.539,-107.993,80.341,98.066,-112.652,94.722,88.799,-114.936,104.729,91.245,-100.719,93.675,84.152,-100.749,83.889,90.054,-87.461 },
    { 9 ,80.868,85.053,-108.258,80.785,97.775,-112.374,95.334,88.795,-115.984,104.214,91.281,-100.857,94.036,83.633,-100.168,84.07,90.03,-88.113 },
    { 10 ,80.0,84.668,-108.416,81.265,97.458,-112.06,95.941,88.893,-116.891,103.642,91.32,-101.004,94.374,83.239,-99.509,84.274,90.012,-88.824 },
    { 11 ,79.174,84.393,-108.467,81.777,97.118,-111.708,96.537,89.096,-117.652,103.016,91.362,-101.156,94.689,82.974,-98.779,84.499,90.001,-89.588 },
    { 12 ,78.395,84.231,-108.41,82.318,96.757,-111.317,97.119,89.402,-118.262,102.336,91.406,-101.312,94.982,82.839,-97.98,84.747,90.001,-90.399 },
    { 13 ,77.664,84.186,-108.248,82.881,96.38,-110.887,97.682,89.808,-118.718,101.605,91.451,-101.469,95.253,82.834,-97.119,85.018,90.014,-91.252 },
    { 14 ,76.984,84.258,-107.983,83.463,95.989,-110.417,98.223,90.309,-119.02,100.826,91.496,-101.625,95.501,82.953,-96.203,85.311,90.042,-92.142 },
    { 15 ,76.358,84.445,-107.619,84.059,95.588,-109.906,98.735,90.899,-119.169,100.0,91.541,-101.777,95.726,83.194,-95.238,85.626,90.086,-93.061 },
    { 16 ,75.786,84.745,-107.162,84.666,95.18,-109.355,99.215,91.568,-119.167,99.132,91.585,-101.922,95.93,83.549,-94.231,85.964,90.151,-94.004 },
    { 17 ,75.271,85.151,-106.617,85.278,94.77,-108.762,99.659,92.306,-119.02,98.224,91.627,-102.059,96.111,84.01,-93.191,86.325,90.236,-94.965 },
    { 18 ,74.813,85.657,-105.99,85.893,94.362,-108.128,100.062,93.101,-118.732,97.279,91.667,-102.186,96.271,84.569,-92.127,86.709,90.344,-95.937 },
    { 19 ,74.414,86.254,-105.292,86.506,93.958,-107.454,100.42,93.942,-118.313,96.302,91.703,-102.299,96.409,85.214,-91.048,87.115,90.476,-96.917 },
    { 20 ,74.076,86.931,-104.529,87.115,93.562,-106.74,100.729,94.816,-117.772,95.296,91.735,-102.399,96.526,85.934,-89.963,87.543,90.634,-97.897 },
    { 21 ,73.797,87.677,-103.712,87.716,93.177,-105.988,100.987,95.71,-117.119,94.267,91.762,-102.482,96.621,86.717,-88.884,87.993,90.817,-98.873 },
    { 22 ,73.58,88.48,-102.852,88.308,92.808,-105.2,101.19,96.61,-116.368,93.217,91.783,-102.549,96.695,87.55,-87.822,88.465,91.027,-99.839 },
    { 23 ,73.425,89.327,-101.961,88.887,92.455,-104.376,101.337,97.504,-115.533,92.153,91.799,-102.597,96.748,88.419,-86.789,88.957,91.263,-100.791 },
    { 24 ,73.332,90.204,-101.05,89.451,92.123,-103.521,101.426,98.379,-114.627,91.079,91.809,-102.626,96.779,89.311,-85.797,89.469,91.525,-101.725 },
    { 25 ,73.301,91.097,-100.134,90.0,91.812,-102.636,101.456,99.223,-113.666,450.0,91.812,-102.636,96.79,90.212,-84.861,90.0,91.812,-102.636 },
    { 26 ,73.332,91.1,-100.144,89.451,91.232,-104.438,101.426,99.204,-113.651,91.079,90.917,-103.54,96.779,90.208,-84.904,89.469,90.632,-102.636 },
    { 27 ,73.425,91.107,-100.174,88.887,90.688,-106.194,101.337,99.148,-113.605,92.153,90.025,-104.4,96.748,90.199,-85.031,88.957,89.485,-102.582 },
    { 28 ,73.58,91.119,-100.223,88.308,90.192,-107.887,101.19,99.055,-113.527,93.217,89.152,-105.202,96.695,90.183,-85.241,88.465,88.388,-102.464 },
    { 29 ,73.797,91.136,-100.29,87.716,89.756,-109.502,100.987,98.925,-113.418,94.267,88.311,-105.935,96.621,90.162,-85.533,87.993,87.357,-102.277 },
    { 30 ,74.076,91.157,-100.375,87.115,89.389,-111.027,100.729,98.76,-113.277,95.296,87.517,-106.588,96.526,90.137,-85.905,87.543,86.406,-102.015 },
    { 31 ,74.414,91.182,-100.476,86.506,89.103,-112.448,100.42,98.56,-113.103,96.302,86.782,-107.153,96.409,90.109,-86.352,87.115,85.547,-101.673 },
    { 32 ,74.813,91.211,-100.591,85.893,88.904,-113.754,100.062,98.328,-112.895,97.279,86.119,-107.623,96.271,90.081,-86.873,86.709,84.793,-101.252 },
    { 33 ,75.271,91.245,-100.719,85.278,88.799,-114.936,99.659,98.066,-112.652,98.224,85.539,-107.993,96.111,90.054,-87.461,86.325,84.152,-100.749 },
    { 34 ,75.786,91.281,-100.857,84.666,88.795,-115.984,99.215,97.775,-112.374,99.132,85.053,-108.258,95.93,90.03,-88.113,85.964,83.633,-100.168 },
    { 35 ,76.358,91.32,-101.004,84.059,88.893,-116.891,98.735,97.458,-112.06,100.0,84.668,-108.416,95.726,90.012,-88.824,85.626,83.239,-99.509 },
    { 36 ,76.984,91.362,-101.156,83.463,89.096,-117.652,98.223,97.118,-111.708,100.826,84.393,-108.467,95.501,90.001,-89.588,85.311,82.974,-98.779 },
    { 37 ,77.664,91.406,-101.312,82.881,89.402,-118.262,97.682,96.757,-111.317,101.605,84.231,-108.41,95.253,90.001,-90.399,85.018,82.839,-97.98 },
    { 38 ,78.395,91.451,-101.469,82.318,89.808,-118.718,97.119,96.38,-110.887,102.336,84.186,-108.248,94.982,90.014,-91.252,84.747,82.834,-97.119 },
    { 39 ,79.174,91.496,-101.625,81.777,90.309,-119.02,96.537,95.989,-110.417,103.016,84.258,-107.983,94.689,90.042,-92.142,84.499,82.953,-96.203 },
    { 40 ,80.0,91.541,-101.777,81.265,90.899,-119.169,95.941,95.588,-109.906,103.642,84.445,-107.619,94.374,90.086,-93.061,84.274,83.194,-95.238 },
    { 41 ,80.868,91.585,-101.922,80.785,91.568,-119.167,95.334,95.18,-109.355,104.214,84.745,-107.162,94.036,90.151,-94.004,84.07,83.549,-94.231 },
    { 42 ,81.776,91.627,-102.059,80.341,92.306,-119.02,94.722,94.77,-108.762,104.729,85.151,-106.617,93.675,90.236,-94.965,83.889,84.01,-93.191 },
    { 43 ,82.721,91.667,-102.186,79.938,93.101,-118.732,94.107,94.362,-108.128,105.187,85.657,-105.99,93.291,90.344,-95.937,83.729,84.569,-92.127 },
    { 44 ,83.698,91.703,-102.299,79.58,93.942,-118.313,93.494,93.958,-107.454,105.586,86.254,-105.292,92.885,90.476,-96.917,83.591,85.214,-91.048 },
    { 45 ,84.704,91.735,-102.399,79.271,94.816,-117.772,92.885,93.562,-106.74,105.924,86.931,-104.529,92.457,90.634,-97.897,83.474,85.934,-89.963 },
    { 46 ,85.733,91.762,-102.482,79.013,95.71,-117.119,92.284,93.177,-105.988,106.203,87.677,-103.712,92.007,90.817,-98.873,83.379,86.717,-88.884 },
    { 47 ,86.783,91.783,-102.549,78.81,96.61,-116.368,91.692,92.808,-105.2,106.42,88.48,-102.852,91.535,91.027,-99.839,83.305,87.55,-87.822 },
    { 48 ,87.847,91.799,-102.597,78.663,97.504,-115.533,91.113,92.455,-104.376,106.575,89.327,-101.961,91.043,91.263,-100.791,83.252,88.419,-86.789 },
    { 49 ,88.921,91.809,-102.626,78.574,98.379,-114.627,90.549,92.123,-103.521,106.668,90.204,-101.05,90.531,91.525,-101.725,83.221,89.311,-85.797 },
    { 50 ,90.0,91.812,-102.636,78.544,99.223,-113.666,90.0,91.812,-102.636,106.699,91.097,-100.134,90.0,91.812,-102.636,83.21,90.212,-84.861 },
};

int count_max = sizeof(pattern_walk) / sizeof(pattern_walk[0]);

void MotionWalk()
{
    Serial.printf("[DEBUG] %d, \n", int(pattern_walk[count][0]));
    for(int leg=0; leg<6;leg++) {
        for(int joint=0; joint<3; joint++) {
            Serial.printf("%.3f,", pattern_walk[count][1+joint+leg*3]);
        }
    }
    Serial.printf("\n");
 
    count++;
    if(count > count_max)
        count = 1;
}

void MotionDebug()
{
    MotionWalk();    
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
            default:
            case MotionPattern::MP_MotionDebug:
                MotionDebug();
                break;
            case MotionPattern::MP_MotionOrigin:
                MotionOrigin();
                break;
            case MotionPattern::MP_MotionIdle:
                MotionIdle();
                break;
            case MotionPattern::MP_MotionStretching:
                MotionStretching();
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
