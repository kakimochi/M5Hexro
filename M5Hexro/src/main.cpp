#include <M5Unified.h>
#include <math.h>

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

static uint8_t count;
static int8_t polarity = +1;

// Walking motion parameters
typedef struct {
    float step_height;      // Maximum lift height for legs (degrees)
    float step_length;      // Step length range (degrees)  
    float base_leg_angle;   // Base angle for leg joints (degrees)
    float base_foot_angle;  // Base angle for foot joints (degrees)
    float cycle_time_ms;    // Complete walk cycle time (milliseconds)
    float frequency;        // Walking frequency (cycles per second)
} WalkParams;

// Default walking parameters
static WalkParams walk_params = {
    .step_height = 15.0f,     // 15 degree lift
    .step_length = 25.0f,     // 25 degree swing range
    .base_leg_angle = 90.0f,  // Center position for legs
    .base_foot_angle = 60.0f, // Center position for feet  
    .cycle_time_ms = 2500.0f, // 2.5 second cycle
    .frequency = 0.4f         // 0.4 Hz
};

// Walking state
static unsigned long walk_start_time = 0;
static bool walk_initialized = false;
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

// Leg grouping for tripod gait
// Group 1: FL, CR, BL (move together)
// Group 2: FR, CL, BR (move together) 
typedef enum {
    TRIPOD_GROUP_1 = 0,
    TRIPOD_GROUP_2 = 1
} TripodGroup;

// Get tripod group for each leg
TripodGroup getLegTripodGroup(LegLayout leg) {
    switch(leg) {
        case FrontL:
        case CenterR:
        case BackL:
            return TRIPOD_GROUP_1;
        case FrontR:
        case CenterL:
        case BackR:
            return TRIPOD_GROUP_2;
        default:
            return TRIPOD_GROUP_1;
    }
}

// Calculate leg position based on gait phase
void calculateLegPosition(LegLayout leg, float phase, float* leg_angle, float* foot_angle) {
    TripodGroup group = getLegTripodGroup(leg);
    
    // Offset phase for second tripod group
    float adjusted_phase = phase;
    if (group == TRIPOD_GROUP_2) {
        adjusted_phase = fmod(phase + 0.5f, 1.0f);  // 180 degree phase offset
    }
    
    // Calculate vertical (foot) motion - sinusoidal lift
    float lift_phase = sin(adjusted_phase * 2.0f * PI);
    if (lift_phase < 0) lift_phase = 0;  // Only lift, don't push down
    
    // Calculate horizontal (leg) motion - sinusoidal swing
    float swing_phase = cos(adjusted_phase * 2.0f * PI);
    
    // Apply motion to servo angles
    *leg_angle = walk_params.base_leg_angle + (swing_phase * walk_params.step_length);
    *foot_angle = walk_params.base_foot_angle - (lift_phase * walk_params.step_height);
    
    // Ensure angles stay within safe limits
    *leg_angle = constrain(*leg_angle, 45.0f, 135.0f);
    *foot_angle = constrain(*foot_angle, 45.0f, 135.0f);
}

void MotionWalk()
{
    // Initialize walk timing on first call
    if (!walk_initialized) {
        walk_start_time = millis();
        walk_initialized = true;
        Serial.printf("[INFO] Mathematical walking motion initialized\n");
    }
    
    // Calculate current phase in walk cycle (0.0 to 1.0)
    unsigned long current_time = millis();
    unsigned long elapsed_time = current_time - walk_start_time;
    float phase = fmod((float)elapsed_time / walk_params.cycle_time_ms, 1.0f);
    
    Serial.printf("[DEBUG] Phase: %.3f, Time: %lu, ", phase, elapsed_time);
    
    // Calculate and set servo angles for each leg
    for(int leg = 0; leg < NUM_LEGS; leg++) {
        float leg_angle, foot_angle;
        calculateLegPosition((LegLayout)leg, phase, &leg_angle, &foot_angle);
        
        // Map leg enum to servo channels
        uint8_t leg_servo_ch, foot_servo_ch;
        switch(leg) {
            case FrontL:
                leg_servo_ch = FL_Leg;
                foot_servo_ch = FL_Foot;
                break;
            case FrontR:
                leg_servo_ch = FR_Leg;
                foot_servo_ch = FR_Foot;
                break;
            case CenterL:
                leg_servo_ch = CL_Leg;
                foot_servo_ch = CL_Foot;
                break;
            case CenterR:
                leg_servo_ch = CR_Leg;
                foot_servo_ch = CR_Foot;
                break;
            case BackL:
                leg_servo_ch = BL_Leg;
                foot_servo_ch = BL_Foot;
                break;
            case BackR:
                leg_servo_ch = BR_Leg;
                foot_servo_ch = BR_Foot;
                break;
            default:
                continue;
        }
        
        // Set servo angles
        setServoAngle(leg_servo_ch, (uint8_t)leg_angle);
        setServoAngle(foot_servo_ch, (uint8_t)foot_angle);
        
        Serial.printf("L%d:%.1f,%.1f ", leg, leg_angle, foot_angle);
    }
    Serial.printf("\n");
}

void MotionTapping()
{
    const int pos_init = 90;
    const int pos_idle = 60;
    const uint8_t count_max = 11;  // Define count_max for tapping motion

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

// Walking motion utility functions
void resetWalkingMotion() {
    walk_initialized = false;
    walk_start_time = 0;
    Serial.printf(\"[INFO] Walking motion reset\\n\");
}

void setWalkingSpeed(float speed_multiplier) {
    if (speed_multiplier > 0.1f && speed_multiplier <= 3.0f) {
        walk_params.cycle_time_ms = 2500.0f / speed_multiplier;
        walk_params.frequency = 0.4f * speed_multiplier;
        resetWalkingMotion();
        Serial.printf(\"[INFO] Walking speed set to %.1fx (cycle: %.0fms)\\n\", 
                     speed_multiplier, walk_params.cycle_time_ms);
    }
}

void setWalkingParameters(float step_height, float step_length) {
    if (step_height >= 5.0f && step_height <= 30.0f) {
        walk_params.step_height = step_height;
    }
    if (step_length >= 10.0f && step_length <= 40.0f) {
        walk_params.step_length = step_length;
    }
    Serial.printf(\"[INFO] Walking parameters: height=%.1f, length=%.1f\\n\", 
                 walk_params.step_height, walk_params.step_length);
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
