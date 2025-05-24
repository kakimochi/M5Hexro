#include <M5Unified.h>

#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>

#include "M5_UNIT_8SERVO.h"

#define PIN_PA_SDA 32   // for Core2, 26 for M5Stack Basic
#define PIN_PA_SCL 33   // for Core2, 27 for M5Stack Basic

#define APP_NAME "M5Hexro"
#define APP_VERSION "ver.1.0"

#define NUM_SERVOS_PER_UNIT 8
#define NUM_CHANNELS_PAHUB 6
#define NUM_SERVO_UNITS_MAX 2
#define ID_8SERVO_UNIT_0 0      // PA_HUB 0 : 8Servo Unit 0
#define ID_8SERVO_UNIT_1 1      // PA_HUB 1 : 8Servo Unit 1

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

#define TONE_C5 523.251
#define TONE_E5 659.255
#define TONE_G5 783.991
#define TONE_C6 (TONE_C5 * 2)
#define TONE_E6 (TONE_E5 * 2)
#define TONE_G6 (TONE_G5 * 2)


QWIICMUX i2cMux;
M5GFX display;
M5Canvas canvas(&display);
M5_UNIT_8SERVO unit_8servo[NUM_SERVO_UNITS_MAX];
int battery_level = -1;

const unsigned long interval_10sec = 10000;
const unsigned long interval_5sec  =  5000;
const unsigned long interval_1sec  =  1000;
unsigned long pre_ms = 0;
unsigned long current_ms = 0;

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
    if(angle_deg < 0 && 180 < angle_deg) {
        Serial.printf("[Warn] HW Limit, L%02d over angle:%d [deg]\n", __LINE__, ch, angle_deg);
        return;
    }
    if(angle_deg < 45 && 135 < angle_deg) {
        Serial.printf("[Warn] SW Limit, L%02d over angle:%d [deg]\n", __LINE__, ch, angle_deg);
        return;
    }

    uint8_t ch_pahub = ch / NUM_SERVOS_PER_UNIT;
    uint8_t ch_servo = ch % NUM_SERVOS_PER_UNIT;

    i2cMux.setPort(ch_pahub);

    unit_8servo[ch_pahub].setServoAngle(ch_servo, angle_deg);
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
const uint8_t count_max = 11; // Fixed value for motion patterns
void MotionStretching()
{
    const uint8_t step = 2;
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

const float WALK_STEP_HEIGHT = 30.0;     // Maximum height of leg lift during step (degrees)
const float WALK_STRIDE_LENGTH = 30.0;   // Forward/backward movement range (degrees)
const float WALK_NEUTRAL_LEG = 90.0;     // Neutral position for leg servos
const float WALK_NEUTRAL_FOOT = 60.0;    // Neutral position for foot servos
const float WALK_FOOT_GROUND = 75.0;     // Foot angle when on ground
const float WALK_FOOT_LIFTED = 45.0;     // Foot angle when lifted

const bool LEG_IN_GROUP1[NUM_LEGS] = {
    true,   // FrontL
    false,  // FrontR
    false,  // CenterL
    true,   // CenterR
    true,   // BackL
    false   // BackR
};

unsigned long walkStartTime = 0;
const unsigned long WALK_CYCLE_DURATION = 1000; // Full walk cycle in milliseconds

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


void MotionWalk()
{
    if (walkStartTime == 0) {
        walkStartTime = millis();
    }
    
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - walkStartTime;
    float phase = (float)(elapsedTime % WALK_CYCLE_DURATION) / WALK_CYCLE_DURATION;
    
    Serial.printf("[DEBUG] Walk phase: %.3f\n", phase);
    
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        float legAngle, footAngle;
        bool isGroup1 = LEG_IN_GROUP1[leg];
        
        calculateTripodGait(phase, isGroup1, legAngle, footAngle);
        
        setServoAngle(leg * NUM_LEG_STRUCTURE + Leg, legAngle);
        setServoAngle(leg * NUM_LEG_STRUCTURE + Foot, footAngle);
        
        Serial.printf("Leg %d: %.3f, %.3f\n", leg, legAngle, footAngle);
    }
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

void setup()
{
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Power.begin();
    delay(100);

    Wire.begin(PIN_PA_SDA, PIN_PA_SCL);
    if (!i2cMux.begin(0x70, Wire)) {
        Serial.printf("[Error] PaHUB not detected. Freezing...");
        ASSERT(0, __LINE__);
    }
    Serial.printf("[Info] PaHUB detected. Begin scanning for I2C devices");

    for(uint8_t ch_pahub = 0; ch_pahub < NUM_CHANNELS_PAHUB; ch_pahub++)
    {
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

    timerMotion = timerBegin(ID_TIMER_MOTION, 80, true);   // divider:80 for 1us count
    timerAttachInterrupt(timerMotion, &onTimerMotion, true);
    timerAlarmWrite(timerMotion, TIMER_MOTION_INTERVAL_US, true);
    timerAlarmEnable(timerMotion);

    pre_ms = millis();

    Serial.printf("[Info] init done.\n");
    beep_init_done();
}


void loop()
{
    M5.update();

    if(M5.BtnA.wasPressed()) {
        Serial.printf("[Info] L%d, BtnA was pressed.\n", __LINE__);
        timerAlarmDisable(timerMotion);
        beep();
        while(1) {
            M5.update();
            if(M5.BtnC.wasPressed()) {
                Serial.printf("[Info] L%d, BtnC was pressed.\n", __LINE__);
                timerAlarmEnable(timerMotion);
                beep();
                break;
            }
        }
    }
    if(M5.BtnB.wasPressed()) {
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

    current_ms = millis();
    if((current_ms - pre_ms) >= interval_10sec) {
        gui_disp_batterylevel();
        pre_ms = current_ms;
    }

    vTaskDelay(50);
}
