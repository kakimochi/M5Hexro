#include <M5Unified.h>
#include <esp32-hal-timer.h>

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

#define USE_NUMERICAL_CALCULATION  // Comment out to use table-based method

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
// Battery
int battery_level = -1;

// for application
const unsigned long interval_10sec = 10000;
const unsigned long interval_5sec  =  5000;
const unsigned long interval_1sec  =  1000;
unsigned long pre_ms = 0;
unsigned long current_ms = 0;

// Timer Interrupt
#define ID_TIMER_MOTION 0
const uint64_t TIMER_MOTION_INTERVAL_US = 50000;     // interval : 50ms
hw_timer_t *timerMotion = NULL;
bool motion_trigger = false;
void IRAM_ATTR onTimerMotion(void)
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
    FR_Leg = 0,
    FR_Foot,
    CR_Leg,
    CR_Foot,
    BR_Leg,
    BR_Foot,
    BL_Leg,
    BL_Foot,
    CL_Leg,
    CL_Foot,
    FL_Leg,
    FL_Foot,
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
    MP_MotionTapping,
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
    const int pos_init = 90;
    const int pos_idle = 60;

    setServoAngle(FL_Leg,  pos_init);
    setServoAngle(FR_Leg,  pos_init);
    setServoAngle(CL_Leg,  pos_init);
    setServoAngle(CR_Leg,  pos_init);
    setServoAngle(BL_Leg,  pos_init);
    setServoAngle(BR_Leg,  pos_init);
    setServoAngle(FL_Foot, pos_idle + random(-10, 10));
    setServoAngle(FR_Foot, pos_idle + random(-10, 10));
    setServoAngle(CL_Foot, pos_idle + random(-10, 10));
    setServoAngle(CR_Foot, pos_idle + random(-10, 10));
    setServoAngle(BL_Foot, pos_idle + random(-10, 10));
    setServoAngle(BR_Foot, pos_idle + random(-10, 10));
}

#ifdef USE_NUMERICAL_CALCULATION
const float WALK_STEP_HEIGHT = 30.0;     // Maximum height of leg lift during step (degrees)
const float WALK_STRIDE_LENGTH = 30.0;   // Forward/backward movement range (degrees)
const float WALK_NEUTRAL_LEG = 90.0;     // Neutral position for leg servos
const float WALK_NEUTRAL_FOOT = 60.0;    // Neutral position for foot servos
const float WALK_FOOT_GROUND = 75.0;     // Foot angle when on ground
const unsigned long WALK_CYCLE_DURATION = 2000; // Walking cycle duration in milliseconds

const bool LEG_IN_GROUP1[NUM_LEGS] = {
    true,   // FrontL
    false,  // FrontR
    false,  // CenterL
    true,   // CenterR
    true,   // BackL
    false   // BackR
};

unsigned long walkStartTime = 0;
#endif

static uint8_t count;
static int8_t polarity = +1;
void MotionStretching()
{
    const uint8_t step = 2;
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
    { 0 ,90.0,61.812,66.206,75.879,90.0,61.812,116.565,60.301,90.0,61.812,80.104,63.008 },
    { 1 ,86.414,52.743,66.499,75.749,91.898,53.936,116.384,60.315,91.702,51.918,80.157,62.918 },
    { 2 ,82.912,43.688,67.361,75.36,93.986,46.319,115.841,60.355,93.213,42.614,80.319,62.658 },
    { 3 ,79.571,35.268,68.739,74.713,96.248,39.3,114.933,60.424,94.539,34.747,80.59,62.258 },
    { 4 ,76.457,28.292,70.555,73.814,98.653,33.46,113.661,60.526,95.691,29.024,80.976,61.767 },
    { 5 ,73.622,23.529,72.708,72.676,101.151,29.67,112.024,60.662,96.681,25.794,81.481,61.241 },
    { 6 ,71.105,21.464,75.088,71.331,103.671,28.93,110.026,60.829,97.52,25.049,82.114,60.741 },
    { 7 ,68.93,22.214,77.587,69.824,106.123,31.894,107.678,61.02,98.219,26.579,82.881,60.332 },
    { 8 ,67.112,25.597,80.106,68.219,108.404,38.24,104.998,61.225,98.787,30.109,83.795,60.069 },
    { 9 ,65.657,31.213,82.565,66.592,110.402,46.627,102.018,61.426,99.231,35.358,84.864,60.005 },
    { 10 ,64.568,38.513,84.903,65.021,112.01,55.549,98.783,61.602,99.559,42.038,86.102,60.181 },
    { 11 ,63.842,46.893,87.081,63.583,113.139,64.133,95.352,61.733,99.775,49.847,87.519,60.625 },
    { 12 ,63.48,55.797,89.076,62.343,113.72,72.117,91.798,61.803,99.883,58.477,89.124,61.347 },
    { 13 ,63.48,60.305,89.076,57.856,113.72,75.846,91.798,57.305,99.883,62.986,89.124,56.842 },
    { 14 ,63.842,60.331,87.081,50.078,113.139,75.587,95.352,48.181,99.775,62.807,87.519,47.139 },
    { 15 ,64.568,60.386,84.903,42.705,112.01,75.069,98.783,39.35,99.559,62.473,86.102,38.45 },
    { 16 ,65.657,60.471,82.565,36.185,110.402,74.294,102.018,31.547,99.231,62.021,84.864,31.586 },
    { 17 ,67.112,60.59,80.106,31.245,108.404,73.273,104.998,25.595,98.787,61.504,83.795,27.091 },
    { 18 ,68.93,60.742,77.587,28.862,106.123,72.027,107.678,22.144,98.219,60.984,82.881,25.121 },
    { 19 ,71.105,60.922,75.088,29.941,103.671,70.594,110.026,21.492,97.52,60.522,82.114,25.546 },
    { 20 ,73.622,61.122,72.708,34.711,101.151,69.03,112.024,23.597,96.681,60.179,81.481,28.111 },
    { 21 ,76.457,61.327,70.555,42.282,98.653,67.404,113.661,28.157,95.691,60.01,80.976,32.536 },
    { 22 ,79.571,61.518,68.739,51.094,96.248,65.794,114.933,34.69,94.539,60.061,80.59,38.538 },
    { 23 ,82.912,61.674,67.361,59.91,93.986,64.281,115.841,42.605,93.213,60.368,80.319,45.821 },
    { 24 ,86.414,61.777,66.499,68.202,91.898,62.935,116.384,51.31,91.702,60.951,80.157,54.078 },
    { 25 ,90.0,61.812,66.206,75.879,90.0,61.812,116.565,60.301,90.0,61.812,80.104,63.008 },
    { 26 ,86.414,52.743,66.499,75.749,91.898,53.936,116.384,60.315,91.702,51.918,80.157,62.918 },
    { 27 ,82.912,43.688,67.361,75.36,93.986,46.319,115.841,60.355,93.213,42.614,80.319,62.658 },
    { 28 ,79.571,35.268,68.739,74.713,96.248,39.3,114.933,60.424,94.539,34.747,80.59,62.258 },
    { 29 ,76.457,28.292,70.555,73.814,98.653,33.46,113.661,60.526,95.691,29.024,80.976,61.767 },
    { 30 ,73.622,23.529,72.708,72.676,101.151,29.67,112.024,60.662,96.681,25.794,81.481,61.241 },
    { 31 ,71.105,21.464,75.088,71.331,103.671,28.93,110.026,60.829,97.52,25.049,82.114,60.741 },
    { 32 ,68.93,22.214,77.587,69.824,106.123,31.894,107.678,61.02,98.219,26.579,82.881,60.332 },
    { 33 ,67.112,25.597,80.106,68.219,108.404,38.24,104.998,61.225,98.787,30.109,83.795,60.069 },
    { 34 ,65.657,31.213,82.565,66.592,110.402,46.627,102.018,61.426,99.231,35.358,84.864,60.005 },
    { 35 ,64.568,38.513,84.903,65.021,112.01,55.549,98.783,61.602,99.559,42.038,86.102,60.181 },
    { 36 ,63.842,46.893,87.081,63.583,113.139,64.133,95.352,61.733,99.775,49.847,87.519,60.625 },
    { 37 ,63.48,55.797,89.076,62.343,113.72,72.117,91.798,61.803,99.883,58.477,89.124,61.347 },
    { 38 ,63.48,60.305,89.076,57.856,113.72,75.846,91.798,57.305,99.883,62.986,89.124,56.842 },
    { 39 ,63.842,60.331,87.081,50.078,113.139,75.587,95.352,48.181,99.775,62.807,87.519,47.139 },
    { 40 ,64.568,60.386,84.903,42.705,112.01,75.069,98.783,39.35,99.559,62.473,86.102,38.45 },
    { 41 ,65.657,60.471,82.565,36.185,110.402,74.294,102.018,31.547,99.231,62.021,84.864,31.586 },
    { 42 ,67.112,60.59,80.106,31.245,108.404,73.273,104.998,25.595,98.787,61.504,83.795,27.091 },
    { 43 ,68.93,60.742,77.587,28.862,106.123,72.027,107.678,22.144,98.219,60.984,82.881,25.121 },
    { 44 ,71.105,60.922,75.088,29.941,103.671,70.594,110.026,21.492,97.52,60.522,82.114,25.546 },
    { 45 ,73.622,61.122,72.708,34.711,101.151,69.03,112.024,23.597,96.681,60.179,81.481,28.111 },
    { 46 ,76.457,61.327,70.555,42.282,98.653,67.404,113.661,28.157,95.691,60.01,80.976,32.536 },
    { 47 ,79.571,61.518,68.739,51.094,96.248,65.794,114.933,34.69,94.539,60.061,80.59,38.538 },
    { 48 ,82.912,61.674,67.361,59.91,93.986,64.281,115.841,42.605,93.213,60.368,80.319,45.821 },
    { 49 ,86.414,61.777,66.499,68.202,91.898,62.935,116.384,51.31,91.702,60.951,80.157,54.078 },
};

int count_max = sizeof(pattern_walk) / sizeof(pattern_walk[0]);

#ifdef USE_NUMERICAL_CALCULATION
float constrainServoAngle(float angle) {
    if (angle < 45.0) return 45.0;
    if (angle > 135.0) return 135.0;
    return angle;
}

void calculateTripodGait(float phase, bool isGroup1, float& legAngle, float& footAngle) {
    if ((isGroup1 && phase < 0.5) || (!isGroup1 && phase >= 0.5)) {
        float liftPhase;
        
        if (isGroup1) {
            liftPhase = phase * 2.0; // 0.0 to 1.0 during first half of cycle
        } else {
            liftPhase = (phase - 0.5) * 2.0; // 0.0 to 1.0 during second half of cycle
        }
        
        float legProgress = sin(liftPhase * PI);
        
        legAngle = WALK_NEUTRAL_LEG - WALK_STRIDE_LENGTH/2 + WALK_STRIDE_LENGTH * legProgress;
        footAngle = WALK_NEUTRAL_FOOT - WALK_STEP_HEIGHT * sin(liftPhase * PI);
    } 
    else {
        float groundPhase;
        
        if (isGroup1) {
            groundPhase = (phase - 0.5) * 2.0; // 0.0 to 1.0 during second half of cycle
        } else {
            groundPhase = phase * 2.0; // 0.0 to 1.0 during first half of cycle
        }
        
        legAngle = WALK_NEUTRAL_LEG + WALK_STRIDE_LENGTH/2 - WALK_STRIDE_LENGTH * groundPhase;
        footAngle = WALK_FOOT_GROUND;
    }
    
    legAngle = constrainServoAngle(legAngle);
    footAngle = constrainServoAngle(footAngle);
}
#endif

void MotionWalk()
{
#ifdef USE_NUMERICAL_CALCULATION
    // Numerical calculation method
    if (walkStartTime == 0) {
        walkStartTime = millis();
    }
    
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - walkStartTime;
    float phase = (elapsedTime % WALK_CYCLE_DURATION) / (float)WALK_CYCLE_DURATION;
    
    Serial.printf("[DEBUG] Phase: %.3f, ", phase);
    
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float legAngle, footAngle;
        
        calculateTripodGait(phase, LEG_IN_GROUP1[leg], legAngle, footAngle);
        
        setServoAngle(Leg + leg * NUM_LEG_STRUCTURE, legAngle);
        setServoAngle(Foot + leg * NUM_LEG_STRUCTURE, footAngle);
        
        Serial.printf("%.3f,%.3f,", legAngle, footAngle);
    }
    Serial.printf("\n");
#else
    // Table-based method (original)
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
#endif
}

void MotionTapping()
{
    const int pos_init = 90;
    const int pos_idle = 60;

    setServoAngle(FL_Leg,  pos_init);
    setServoAngle(FR_Leg,  pos_init);
    setServoAngle(CL_Leg,  pos_init);
    setServoAngle(CR_Leg,  pos_init);
    setServoAngle(BL_Leg,  pos_init);
    setServoAngle(BR_Leg,  pos_init);

    if(polarity >= 0) {
        setServoAngle(FL_Foot, 45);
        setServoAngle(FR_Foot, 90);
        setServoAngle(CL_Foot, 90);
        setServoAngle(CR_Foot, 45);
        setServoAngle(BL_Foot, 45);
        setServoAngle(BR_Foot, 90);
    } else {
        setServoAngle(FL_Foot, 90);
        setServoAngle(FR_Foot, 45);
        setServoAngle(CL_Foot, 45);
        setServoAngle(CR_Foot, 90);
        setServoAngle(BL_Foot, 90);
        setServoAngle(BR_Foot, 45);
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

void gui_disp_batterylevel()
{
    battery_level = M5.Power.getBatteryLevel();

    int pos_x = M5.Lcd.width() - 14/2 * (4+1);
    int pos_y = 7;
    M5.Display.startWrite();    // Occupies the SPI bus to speed up drawing
        M5.Display.setTextColor(GOLD);
        M5.Display.setTextSize(1);
        M5.Display.fillRect(pos_x, pos_y, 320, 14*1, BLACK);    // clear
        M5.Display.drawString(String(battery_level)+"%", pos_x, pos_y);
    M5.Display.endWrite();
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
    case MotionPattern::MP_MotionTapping:
        str_pattern = "motion tapping";
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
    battery_level = M5.Power.getBatteryLevel();
    
    M5.Display.begin();
    M5.Display.startWrite();    // Occupies the SPI bus to speed up drawing
        M5.Display.setColorDepth(1); // mono color
        M5.Display.fillScreen(BLACK);
        M5.Display.setFont(&fonts::efontCN_14);
        M5.Display.setTextColor(GOLD);
        M5.Display.setTextSize(2);  // 14*2
        M5.Display.drawString(APP_NAME, 7, 7);
        M5.Display.setTextSize(1);  // 14
        M5.Display.drawString(APP_VERSION, 7 + 14*2 * 7, 7);     // 8 characters in "ver.1.0 "
        M5.Display.drawString(String(battery_level)+"%", M5.Lcd.width() - 14/2 * (4+1), 7);
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

    // application timer
    pre_ms = millis();

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
            case MotionPattern::MP_MotionTapping:
                MotionTapping();
                break;
            default:
                break;
        }
        motion_trigger = false; // reset trigger
    }

    // application timer
    current_ms = millis();
    if((current_ms - pre_ms) >= interval_10sec) {
        gui_disp_batterylevel();
        pre_ms = current_ms;
    }

    vTaskDelay(50);
}
