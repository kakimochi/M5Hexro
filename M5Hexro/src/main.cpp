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
const uint64_t TIMER_MOTION_INTERVAL_US = 30000;     // interval : 30ms
// const uint64_t TIMER_MOTION_INTERVAL_US = 500000; // interval : 500ms
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
    NUM_Legs
} LegLayout;

typedef enum {
    Leg = 0,
    Foot,
    NUM_LegStructure
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
    if(angle_deg < 0 && 180 < angle_deg) {
        Serial.printf("[Warn] L%d over angle:%d [deg]\n", __LINE__, angle_deg);
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

uint8_t motionPattern = MotionPattern::MP_MotionOrigin;
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

void MotionWalk()
{
    // typedef struct TripodGaitPattern {
    //     int frame_count;
    //     float theta[3*NUM_Legs];
    // } TripodGaitPattern;

    float tripod_gait_pattern[] = {
        {1, 1.079, -0.917, -103.54, 11.426, -9.204, -113.651, -0.549, -1.232, -104.438, 343.332, -1.1, -100.144, 359.469, -0.632, -102.636, 366.779, -0.208, -84.904},
        {2, 2.153, -0.025, -104.4, 11.337, -9.148, -113.605, -1.113, -0.688, -106.194, 343.425, -1.107, -100.174, 358.957, 0.515, -102.582, 366.748, -0.199, -85.031},
        {3, 3.217, 0.848, -105.202, 11.19, -9.055, -113.527, -1.692, -0.192, -107.887, 343.58, -1.119, -100.223, 358.465, 1.612, -102.464, 366.695, -0.183, -85.241},
        {4, 4.267, 1.689, -105.935, 10.987, -8.925, -113.418, -2.284, 0.244, -109.502, 343.797, -1.136, -100.29, 357.993, 2.643, -102.277, 366.621, -0.162, -85.533},
        {5, 5.296, 2.483, -106.588, 10.729, -8.76, -113.277, -2.885, 0.611, -111.027, 344.076, -1.157, -100.375, 357.543, 3.594, -102.015, 366.526, -0.137, -85.905},
        {6, 6.302, 3.218, -107.153, 10.42, -8.56, -113.103, -3.494, 0.897, -112.448, 344.414, -1.182, -100.476, 357.115, 4.453, -101.673, 366.409, -0.109, -86.352},
        {7, 7.279, 3.881, -107.623, 10.062, -8.328, -112.895, -4.107, 1.096, -113.754, 344.813, -1.211, -100.591, 356.709, 5.207, -101.252, 366.271, -0.081, -86.873},
        {8, 8.224, 4.461, -107.993, 9.659, -8.066, -112.652, -4.722, 1.201, -114.936, 345.271, -1.245, -100.719, 356.325, 5.848, -100.749, 366.111, -0.054, -87.461},
        {9, 9.132, 4.947, -108.258, 9.215, -7.775, -112.374, -5.334, 1.205, -115.984, 345.786, -1.281, -100.857, 355.964, 6.367, -100.168, 365.93, -0.03, -88.113},
        {10, 10.0, 5.332, -108.416, 8.735, -7.458, -112.06, -5.941, 1.107, -116.891, 346.358, -1.32, -101.004, 355.626, 6.761, -99.509, 365.726, -0.012, -88.824},
        {11, 10.826, 5.607, -108.467, 8.223, -7.118, -111.708, -6.537, 0.904, -117.652, 346.984, -1.362, -101.156, 355.311, 7.026, -98.779, 365.501, -0.001, -89.588},
        {12, 11.605, 5.769, -108.41, 7.682, -6.757, -111.317, -7.119, 0.598, -118.262, 347.664, -1.406, -101.312, 355.018, 7.161, -97.98, 365.253, -0.001, -90.399},
        {13, 12.336, 5.814, -108.248, 7.119, -6.38, -110.887, -7.682, 0.192, -118.718, 348.395, -1.451, -101.469, 354.747, 7.166, -97.119, 364.982, -0.014, -91.252},
        {14, 13.016, 5.742, -107.983, 6.537, -5.989, -110.417, -8.223, -0.309, -119.02, 349.174, -1.496, -101.625, 354.499, 7.047, -96.203, 364.689, -0.042, -92.142},
        {15, 13.642, 5.555, -107.619, 5.941, -5.588, -109.906, -8.735, -0.899, -119.169, 350.0, -1.541, -101.777, 354.274, 6.806, -95.238, 364.374, -0.086, -93.061},
        {16, 14.214, 5.255, -107.162, 5.334, -5.18, -109.355, -9.215, -1.568, -119.167, 350.868, -1.585, -101.922, 354.07, 6.451, -94.231, 364.036, -0.151, -94.004},
        {17, 14.729, 4.849, -106.617, 4.722, -4.77, -108.762, -9.659, -2.306, -119.02, 351.776, -1.627, -102.059, 353.889, 5.99, -93.191, 363.675, -0.236, -94.965},
        {18, 15.187, 4.343, -105.99, 4.107, -4.362, -108.128, -10.062, -3.101, -118.732, 352.721, -1.667, -102.186, 353.729, 5.431, -92.127, 363.291, -0.344, -95.937},
        {19, 15.586, 3.746, -105.292, 3.494, -3.958, -107.454, -10.42, -3.942, -118.313, 353.698, -1.703, -102.299, 353.591, 4.786, -91.048, 362.885, -0.476, -96.917},
        {20, 15.924, 3.069, -104.529, 2.885, -3.562, -106.74, -10.729, -4.816, -117.772, 354.704, -1.735, -102.399, 353.474, 4.066, -89.963, 362.457, -0.634, -97.897},
        {21, 16.203, 2.323, -103.712, 2.284, -3.177, -105.988, -10.987, -5.71, -117.119, 355.733, -1.762, -102.482, 353.379, 3.283, -88.884, 362.007, -0.817, -98.873},
        {22, 16.42, 1.52, -102.852, 1.692, -2.808, -105.2, -11.19, -6.61, -116.368, 356.783, -1.783, -102.549, 353.305, 2.45, -87.822, 361.535, -1.027, -99.839},
    };

    int frames = sizeof(tripod_gait_pattern)/sizeof(tripod_gait_pattern[0]);
    for(int i=0; i<frames+1; i++) {
            Serial.printf("%d", int(tripod_gait_pattern[i].frame_count));
            // setServoAngle();
    }
}

void MotionDebug()
{
    MotionWalk();    
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
