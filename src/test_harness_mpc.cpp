/* =====================================================================
   Automated MPC characterisation harness
   Same protocol, same metrics, same actuator handling as
   test_harness_pid.cpp — but the control law is computed on the PC by
   MPC_HOST_TEST.m instead of on the Arduino.

   WHY THIS FILE IS A NEAR-COPY OF THE PID HARNESS
   -----------------------------------------------
   The whole point is a controller comparison. If PID settling time were
   computed by one implementation and MPC settling time by another, any
   difference in the numbers could be the measurement rather than the
   controller. So everything except the control law is byte-for-byte the
   same policy as the PID harness: identical phase machine, identical
   settling criterion, identical overshoot/RMS accumulators, identical
   time-proportioning window and relay deadband. Only computePID() is
   replaced by "ask the host".

   !! THE PROTOCOL CONSTANTS MUST MATCH test_harness_pid.cpp !!
   HEAT_OFFSET, COOL_OFFSET, FLOOR_OFFSET, T4_OFFSET, SETTLE_BAND,
   SETTLE_BAND_DWELL and STEP_TIMEOUT_MS below are the comparison's
   controlled variables. If you change one here, change it there too, or
   the two controllers are not being asked the same question.

   HOW THE LINK WORKS
   ------------------
   Once per second, while control is active, this sketch sends

       T:26.30,SP:27.40,A:25.10

   (inside temp, setpoint, ambient) and expects a line back:

       U:123.4        signed duty demand, -255..255, + = heat, - = cool

   The ambient field is an addition to main_mcp.cpp's protocol. The host
   needs it because the plant is really u -> (T - Tambient): a model
   written in absolute temperature predicts 0 degC at zero duty and makes
   the MPC chase an enormous phantom offset. Appending ",A:.." keeps the
   line compatible with a regexp that only looks for T: and SP:.

   SERIAL IS THE CONTROL CHANNEL — YOU CANNOT USE pio device monitor
   -----------------------------------------------------------------
   A Windows COM port is exclusive. MATLAB holds COM3 for the control
   loop, so PlatformIO's monitor cannot attach at the same time, which
   means monitor_filters/log2file CANNOT capture this run. MPC_HOST_TEST.m
   writes the log instead — it saves every line it receives to
   logs/mpc-test-<timestamp>.csv. Use that script, not the plain
   MPC_HOST.m, or the run produces no data file.

   Build and upload (close MATLAB first, it holds the port):
       pio run -e test_mpc -t upload
   then run MPC_HOST_TEST.m in MATLAB. Opening the port resets the
   Arduino, so the protocol starts when MATLAB connects.

   A KNOWN ASYMMETRY, STATED PLAINLY
   ---------------------------------
   The PID harness zeroes the integrator at every step change. The MPC's
   internal state lives in MATLAB's mpcstate object and this sketch has no
   way to reset it remotely, so the MPC carries state across step changes
   that the PID does not. That favours neither controller uniformly — it
   is simply a difference, and it belongs in the writeup rather than being
   quietly ignored.

   Likewise: whenever the relay deadband, an over-temperature trip or a
   NaN read means the applied duty differs from the commanded duty, the
   host's state estimator is predicting from a command that was not
   actually applied. Both controllers are blind to this in the same way.
   ===================================================================== */

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

#define QUICK_MODE 0

// ---------------------------------------------------------------------
// Where the protocol starts. Lets a session that died partway through be
// resumed without repeating the tests that already produced results.
//
//   1 = full protocol:  T1 -> T2 -> T3 -> T4
//   3 = resume at T3:   reheat to ambient+5, hold 2 min, step to the
//                       floor, then T4
//   4 = T4 only:        warm to ambient+2, hold, lid event
//
// Tests before the start point are neither run nor reported, so keep the
// earlier session's log — this build cannot regenerate those rows.
// ---------------------------------------------------------------------
#define START_AT_TEST 3

#if QUICK_MODE
  #define TSCALE 10UL
#else
  #define TSCALE 1UL
#endif

// -------------------- Pins (identical to both other sketches) ----------
#define DHTPIN      2
#define DHTTYPE     DHT22
#define DHTPIN_OUT  11
#define DHTTYPE_OUT DHT22

#define BTN_START   3
#define BTN_STOP    4
#define BTN_MODE    5

#define LED_POWER   6
#define LED_HEAT    7
#define LED_COOL    8

#define RELAY_HEAT  9
#define RELAY_FAN   10

#define POT_PIN     A0

#define RELAY_ACTIVE_LOW 1
#if RELAY_ACTIVE_LOW
  #define RELAY_ON   LOW
  #define RELAY_OFF  HIGH
#else
  #define RELAY_ON   HIGH
  #define RELAY_OFF  LOW
#endif

#define LED_ACTIVE_LOW 0
#if LED_ACTIVE_LOW
  #define LED_ON   LOW
  #define LED_OFF  HIGH
#else
  #define LED_ON   HIGH
  #define LED_OFF  LOW
#endif

inline void setLed(uint8_t pin, bool on)
{
    digitalWrite(pin, on ? LED_ON : LED_OFF);
}

DHT dht(DHTPIN, DHTTYPE);
DHT dhtOut(DHTPIN_OUT, DHTTYPE_OUT);
LiquidCrystal_I2C lcd(0x27, 16, 2);

const float ABSOLUTE_MAX_TEMP = 40.0;

// -------------------- Actuator limits --------------------
// Same numbers as the PID harness. outDeadband is a property of the
// mechanical relay (the shortest pulse that reliably closes it), not of
// the controller, so it must be identical for the duty comparison to mean
// anything. The host is told to respect the same +/-255 range.
const double outMin      = -255.0;
const double outMax      =  255.0;
const double outDeadband = 64.0;

unsigned long windowStartTime = 0;
const unsigned long windowSize = 2000;
unsigned long onTimeSnapshot = 0;

const unsigned long Ts = 1000;      // must match Ts in MPC_HOST_TEST.m

// -------------------- Host link --------------------
double   hostCommand     = 0;       // last U: received, already clamped
double   appliedCommand  = 0;       // what we actually acted on this tick
uint32_t lastCommandMs   = 0;
bool     linkOk          = false;
uint32_t staleSamples    = 0;
uint32_t commandsRx      = 0;

const unsigned long COMMAND_TIMEOUT_MS   = 5000;    // no U: this long -> go safe
const unsigned long LINK_LOST_ABORT_MS   = 120000;  // ...this long -> abort session

// Line assembly. main_mcp.cpp used String for this; a char buffer avoids
// heap fragmentation over a multi-hour run on a 2 KB part.
char    rxBuf[24];
uint8_t rxLen = 0;

// -------------------- Actuation (identical policy to the PID harness) --
bool heatOn = false;
bool fanOn  = false;

uint32_t heatTotalMs = 0;
uint32_t fanTotalMs  = 0;
unsigned long lastDutyTick = 0;

void setRelays(bool h, bool f)
{
    heatOn = h;
    fanOn  = f;
    digitalWrite(RELAY_HEAT, h ? RELAY_ON : RELAY_OFF);
    digitalWrite(RELAY_FAN,  f ? RELAY_ON : RELAY_OFF);
}

void accumulateDuty(unsigned long now)
{
    unsigned long dt = now - lastDutyTick;
    lastDutyTick = now;
    if (heatOn) heatTotalMs += dt;
    if (fanOn)  fanTotalMs  += dt;
}

void allOff()
{
    setRelays(false, false);
    setLed(LED_HEAT, false);
    setLed(LED_COOL, false);
}

void applyActuation(double u, unsigned long now)
{
    double magnitude = fabs(u);

    if (now - windowStartTime >= windowSize) {
        windowStartTime = now;
        onTimeSnapshot = (unsigned long)((magnitude / outMax) * windowSize);
    }
    unsigned long elapsed = now - windowStartTime;

    bool heating   = (u > 0);
    bool demanding = (magnitude > outDeadband);
    bool relayOn   = demanding && (elapsed < onTimeSnapshot);

    setRelays(relayOn && heating, relayOn && !heating);
    setLed(LED_HEAT, demanding &&  heating);
    setLed(LED_COOL, demanding && !heating);
}

// The PID harness resets its integrator here. The MPC's state is in
// MATLAB and cannot be reset from this side; clearing the last command is
// all this sketch can honestly do. See the header note.
void resetController(unsigned long now)
{
    hostCommand    = 0;
    appliedCommand = 0;
    (void)now;
}

// -------------------- Protocol timing (MUST MATCH THE PID HARNESS) -----
const unsigned long BASELINE_MS       =  120000UL / TSCALE;
const unsigned long SETTLE_BAND_DWELL =  300000UL / TSCALE;
const unsigned long HOLD_MS           =  120000UL / TSCALE;
const unsigned long LID_PREP_MS       =   30000UL / TSCALE;
const unsigned long LID_OPEN_MS       =    5000UL;
const unsigned long RECOVER_DWELL     =   60000UL / TSCALE;
const unsigned long STEP_TIMEOUT_MS   = 2400000UL / TSCALE;
const unsigned long SESSION_MAX_MS    = 5UL * 3600000UL / TSCALE;

const float SETTLE_BAND  = 0.5;
const float HEAT_OFFSET  = 5.0;
const float COOL_OFFSET  = 0.0;
const float FLOOR_OFFSET = -2.0;
const float T4_OFFSET    = 2.0;

const unsigned long SAMPLE_MS = 1000;
const unsigned long LCD_MS    = 500;
const unsigned long LOOP_MS   = 50;

// -------------------- Results (identical layout to the PID harness) ----
struct TestResult {
    uint8_t  test;
    uint8_t  settled;
    uint8_t  crossed;
    float    target;
    float    startTemp;
    float    ambient;
    float    overshoot;
    float    undershoot;
    float    rms;
    float    maxDev;
    float    extreme;
    uint32_t settleMs;
    uint32_t firstInBandMs;
    uint32_t durMs;
    uint32_t heatMs;
    uint32_t fanMs;
};

const uint8_t MAX_RESULTS = 4;
TestResult results[MAX_RESULTS];
uint8_t nResults = 0;

struct StepState {
    float    target;
    float    startTemp;
    bool     heatingStep;
    bool     crossed;
    bool     inBand;
    bool     settled;
    bool     record;
    uint8_t  test;
    uint32_t startMs;
    uint32_t bandEnterMs;
    uint32_t settleMs;
    uint32_t firstInBandMs;
    uint32_t dwellRequired;
    uint32_t timeoutMs;
    uint32_t heatMs0;
    uint32_t fanMs0;
    float    postMax;
    float    postMin;
    float    extreme;
    float    maxDev;
    uint64_t sumSqE4;
    uint32_t nSamples;
};
StepState step;

enum Phase : uint8_t {
    PH_BOOT, PH_BASELINE, PH_T1, PH_T2, PH_T3_REHEAT, PH_T3_HOLD,
    PH_T3_STEP, PH_T4_STABILIZE, PH_T4_PREP, PH_T4_OPEN, PH_T4_RECOVER,
    PH_SUMMARY, PH_DONE
};

Phase    phase        = PH_BOOT;
uint32_t phaseStart   = 0;
uint32_t sessionStart = 0;
bool     forceAdvance = false;
bool     aborted      = false;

float ambient       = NAN;
float ambientInside = NAN;
float setPoint      = 25.0;
bool  systemEnabled = false;

float temperature   = NAN;
float lastValidTemp = NAN;
float humidity      = NAN;
float outTemp       = NAN;
float outHum        = NAN;
uint16_t nanCount   = 0;
bool  overTempTrip  = false;

float    baseSumIn = 0, baseSumOut = 0;
uint16_t baseNIn = 0, baseNOut = 0;

unsigned long lastSampleMs = 0;
unsigned long lastLcdMs    = 0;

// -------------------- Logging --------------------
const __FlashStringHelper *phaseName(Phase p)
{
    switch (p) {
        case PH_BOOT:         return F("BOOT");
        case PH_BASELINE:     return F("BASELINE");
        case PH_T1:           return F("T1_HEAT");
        case PH_T2:           return F("T2_COOL");
        case PH_T3_REHEAT:    return F("T3_REHEAT");
        case PH_T3_HOLD:      return F("T3_HOLD");
        case PH_T3_STEP:      return F("T3_STEP");
        case PH_T4_STABILIZE: return F("T4_STAB");
        case PH_T4_PREP:      return F("T4_PREP");
        case PH_T4_OPEN:      return F("T4_OPEN");
        case PH_T4_RECOVER:   return F("T4_RECOV");
        case PH_SUMMARY:      return F("SUMMARY");
        default:              return F("DONE");
    }
}

void printTemp(float v)
{
    if (isnan(v)) Serial.print(F("NaN"));
    else          Serial.print(v, 2);
}

// Same columns as the PID harness plus link_ok, so the two logs load with
// the same parser and the MPC one additionally shows link health.
void logRow(const __FlashStringHelper *event)
{
    Serial.print((millis() - sessionStart) / 1000.0, 1);
    Serial.print(',');
    Serial.print(phaseName(phase));
    Serial.print(',');
    Serial.print(step.test);
    Serial.print(',');
    printTemp(temperature);
    Serial.print(',');
    printTemp(humidity);
    Serial.print(',');
    printTemp(outTemp);
    Serial.print(',');
    Serial.print(setPoint, 2);
    Serial.print(',');
    if (isnan(temperature)) Serial.print(F("NaN"));
    else                    Serial.print(setPoint - temperature, 2);
    Serial.print(',');
    Serial.print(appliedCommand, 1);
    Serial.print(',');
    Serial.print(fabs(appliedCommand) > outDeadband
                 ? (int)(100.0 * fabs(appliedCommand) / outMax) : 0);
    Serial.print(',');
    Serial.print(heatOn ? 1 : 0);
    Serial.print(',');
    Serial.print(fanOn ? 1 : 0);
    Serial.print(',');
    Serial.print((!isnan(temperature) && fabs(setPoint - temperature) <= SETTLE_BAND) ? 1 : 0);
    Serial.print(',');
    Serial.print(linkOk ? 1 : 0);
    Serial.print(',');
    if (event) Serial.print(event);
    Serial.println();
}

void banner(const __FlashStringHelper *msg)
{
    Serial.print(F("# ["));
    Serial.print((millis() - sessionStart) / 1000);
    Serial.print(F("s] "));
    Serial.println(msg);
}

// -------------------- Step machinery (identical to the PID harness) ----
void beginStep(float target, uint8_t test, bool record,
               uint32_t dwell, uint32_t timeout)
{
    unsigned long now = millis();
    float t0 = isnan(temperature) ? lastValidTemp : temperature;

    step.target        = target;
    step.startTemp     = t0;
    step.heatingStep   = isnan(t0) ? true : (target > t0);
    step.record        = record;
    step.test          = test;
    step.startMs       = now;
    step.bandEnterMs   = 0;
    step.settleMs      = 0;
    step.firstInBandMs = 0;
    step.inBand        = false;
    step.settled       = false;
    step.dwellRequired = dwell;
    step.timeoutMs     = timeout;
    step.heatMs0       = heatTotalMs;
    step.fanMs0        = fanTotalMs;
    step.postMax       = -300;
    step.postMin       =  300;
    step.extreme       = isnan(t0) ? target : t0;
    step.maxDev        = 0;
    step.sumSqE4       = 0;
    step.nSamples      = 0;
    step.crossed       = (!isnan(t0) && fabs(target - t0) <= SETTLE_BAND);

    setPoint = target;
    resetController(now);

    Serial.print(F("# --- step: target "));
    Serial.print(target, 2);
    Serial.print(F(" degC, from "));
    printTemp(step.startTemp);
    Serial.print(F(" degC, direction "));
    Serial.println(step.heatingStep ? F("HEAT") : F("COOL"));
}

void updateStep(unsigned long now)
{
    if (isnan(temperature)) return;

    float err  = step.target - temperature;
    float aerr = fabs(err);

    uint32_t q = (uint32_t)(err * err * 10000.0 + 0.5);
    step.sumSqE4 += q;
    step.nSamples++;

    if (step.heatingStep) {
        if (temperature > step.extreme) step.extreme = temperature;
        if (!step.crossed && temperature >= step.target) step.crossed = true;
    } else {
        if (temperature < step.extreme) step.extreme = temperature;
        if (!step.crossed && temperature <= step.target) step.crossed = true;
    }

    if (step.crossed) {
        if (temperature > step.postMax) step.postMax = temperature;
        if (temperature < step.postMin) step.postMin = temperature;
        if (aerr > step.maxDev) step.maxDev = aerr;
    }

    if (aerr <= SETTLE_BAND) {
        if (!step.inBand) {
            step.inBand = true;
            step.bandEnterMs = now;
            if (step.firstInBandMs == 0) step.firstInBandMs = now - step.startMs;
        } else if (!step.settled && (now - step.bandEnterMs) >= step.dwellRequired) {
            step.settled  = true;
            step.settleMs = step.bandEnterMs - step.startMs;
        }
    } else {
        step.inBand = false;
    }
}

bool stepDone(unsigned long now)
{
    if (forceAdvance) return true;
    if (step.settled) return true;
    return (now - step.startMs) >= step.timeoutMs;
}

void finishStep(unsigned long now)
{
    if (!step.record || nResults >= MAX_RESULTS) return;

    TestResult &r = results[nResults++];
    r.test          = step.test;
    r.settled       = step.settled ? 1 : 0;
    r.crossed       = step.crossed ? 1 : 0;
    r.target        = step.target;
    r.startTemp     = step.startTemp;
    r.ambient       = ambient;
    r.settleMs      = step.settled ? step.settleMs : 0xFFFFFFFFUL;
    r.firstInBandMs = step.firstInBandMs;
    r.durMs         = now - step.startMs;
    r.rms           = step.nSamples ? sqrt((double)step.sumSqE4 / 10000.0 / step.nSamples) : NAN;
    r.maxDev        = step.crossed ? step.maxDev : NAN;
    r.extreme       = step.extreme;
    r.heatMs        = heatTotalMs - step.heatMs0;
    r.fanMs         = fanTotalMs  - step.fanMs0;

    if (!step.crossed) {
        r.overshoot  = NAN;
        r.undershoot = NAN;
    } else if (step.heatingStep) {
        r.overshoot  = max(0.0f, step.postMax - step.target);
        r.undershoot = max(0.0f, step.target - step.postMin);
    } else {
        r.overshoot  = max(0.0f, step.target - step.postMin);
        r.undershoot = max(0.0f, step.postMax - step.target);
    }
}

void enterPhase(Phase p)
{
    phase = p;
    phaseStart = millis();
    forceAdvance = false;
    Serial.print(F("# ===== PHASE "));
    Serial.print(phaseName(p));
    Serial.println(F(" ====="));
}

void idleStep() { step.test = 0; }

// -------------------- Summary --------------------
void printSecs(uint32_t ms)
{
    if (ms == 0xFFFFFFFFUL) Serial.print(F("NA"));
    else                    Serial.print(ms / 1000.0, 1);
}

void printFloatOrNA(float v)
{
    if (isnan(v)) Serial.print(F("NA"));
    else          Serial.print(v, 2);
}

void printSummary()
{
    Serial.println();
    Serial.println(F("# ================= RESULTS ================="));
    Serial.println(F("# controller: MPC (computed on host, applied here)"));
    Serial.print(F("# ambient (outside sensor, 2 min mean): "));
    printFloatOrNA(ambient);
    Serial.print(F("  inside mean: "));
    printFloatOrNA(ambientInside);
    Serial.println();
    Serial.print(F("# settling criterion: +/-"));
    Serial.print(SETTLE_BAND, 1);
    Serial.print(F(" degC held for "));
    Serial.print(SETTLE_BAND_DWELL / 1000);
    Serial.println(F(" s"));
    Serial.print(F("# actuator: deadband="));
    Serial.print(outDeadband);
    Serial.print(F(" window_ms="));
    Serial.print(windowSize);
    Serial.print(F("  (identical to the PID harness)"));
    Serial.println();
    Serial.print(F("# host link: "));
    Serial.print(commandsRx);
    Serial.print(F(" commands received, "));
    Serial.print(staleSamples);
    Serial.println(F(" samples with a stale/absent command"));
    if (staleSamples > 0) {
        Serial.println(F("# WARNING: the host was not commanding for part of this run."
                         " Those stretches measure the enclosure, not the MPC."));
    }
    if (nanCount) {
        Serial.print(F("# WARNING: "));
        Serial.print(nanCount);
        Serial.println(F(" failed sensor reads during the session"));
    }
    if (aborted) Serial.println(F("# WARNING: session was ABORTED early"));

    Serial.println(F("#RESULT,test,target_C,start_C,settled,settle_s,first_in_band_s,"
                     "overshoot_C,undershoot_C,rms_C,max_dev_C,extreme_C,dur_s,heat_pct,fan_pct"));

    for (uint8_t i = 0; i < nResults; i++) {
        TestResult &r = results[i];
        float durS = r.durMs / 1000.0;
        Serial.print(F("RESULT,"));
        Serial.print(r.test);           Serial.print(',');
        Serial.print(r.target, 2);      Serial.print(',');
        printFloatOrNA(r.startTemp);    Serial.print(',');
        Serial.print(r.settled);        Serial.print(',');
        printSecs(r.settleMs);          Serial.print(',');
        Serial.print(r.firstInBandMs / 1000.0, 1); Serial.print(',');
        printFloatOrNA(r.overshoot);    Serial.print(',');
        printFloatOrNA(r.undershoot);   Serial.print(',');
        printFloatOrNA(r.rms);          Serial.print(',');
        printFloatOrNA(r.maxDev);       Serial.print(',');
        Serial.print(r.extreme, 2);     Serial.print(',');
        Serial.print(durS, 1);          Serial.print(',');
        Serial.print(durS > 0 ? (100.0 * r.heatMs / r.durMs) : 0.0, 1); Serial.print(',');
        Serial.println(durS > 0 ? (100.0 * r.fanMs / r.durMs) : 0.0, 1);
    }

    for (uint8_t i = 0; i < nResults; i++) {
        if (results[i].test == 3 && !results[i].settled) {
            Serial.print(F("# TEST 3: never settled at "));
            Serial.print(results[i].target, 2);
            Serial.print(F(" degC. Coldest reached "));
            Serial.print(results[i].extreme, 2);
            Serial.println(F(" degC — expected: the fan cannot cool below ambient."));
        }
    }

    for (uint8_t i = 0; i < nResults; i++) {
        if (results[i].test != 4) continue;
        Serial.print(F("# TEST 4: max deviation "));
        printFloatOrNA(results[i].maxDev);
        Serial.print(F(" degC, recovery "));
        printSecs(results[i].settleMs);
        Serial.println(F(" s (from lid reseal)"));
        if (isnan(results[i].maxDev) || results[i].maxDev < 0.3) {
            Serial.println(F("#   NO-DISTURBANCE: deviation never exceeded 0.3 degC. The lid was"
                             " probably not opened — rerun test 4 before reporting it."));
        }
    }

    uint32_t hSum = 0, fSum = 0, dSum = 0;
    for (uint8_t i = 0; i < nResults; i++) {
        hSum += results[i].heatMs;
        fSum += results[i].fanMs;
        dSum += results[i].durMs;
    }
    Serial.println(F("#DUTY,scope,heat_on_s,fan_on_s,total_s,heat_pct,fan_pct,combined_pct"));
    Serial.print(F("DUTY,tests_1_to_4,"));
    Serial.print(hSum / 1000.0, 1); Serial.print(',');
    Serial.print(fSum / 1000.0, 1); Serial.print(',');
    Serial.print(dSum / 1000.0, 1); Serial.print(',');
    if (dSum > 0) {
        Serial.print(100.0 * hSum / dSum, 1); Serial.print(',');
        Serial.print(100.0 * fSum / dSum, 1); Serial.print(',');
        Serial.println(100.0 * (hSum + fSum) / dSum, 1);
    } else {
        Serial.println(F("NA,NA,NA"));
    }
    Serial.print(F("# session totals: heat ON "));
    Serial.print(heatTotalMs / 1000.0, 1);
    Serial.print(F(" s, fan ON "));
    Serial.print(fanTotalMs / 1000.0, 1);
    Serial.print(F(" s, elapsed "));
    Serial.print((millis() - sessionStart) / 1000.0, 1);
    Serial.println(F(" s"));
    Serial.println(F("# =========================================="));
}

// -------------------- LCD --------------------
void updateLcd(unsigned long now)
{
    lcd.clear();
    lcd.setCursor(0, 0);

    if (phase == PH_T4_PREP || phase == PH_T4_OPEN) {
        if (phase == PH_T4_PREP) {
            lcd.print(F("GET READY: LID"));
            lcd.setCursor(0, 1);
            lcd.print(F("OPEN IN "));
            lcd.print((int)((LID_PREP_MS - (now - phaseStart)) / 1000) + 1);
            lcd.print(F("s"));
        } else {
            lcd.print(F("** OPEN LID **"));
            lcd.setCursor(0, 1);
            lcd.print(F("CLOSE IN "));
            lcd.print((int)((LID_OPEN_MS - (now - phaseStart)) / 1000) + 1);
            lcd.print(F("s"));
        }
        return;
    }

    lcd.print(F("T:"));
    lcd.print(temperature, 1);
    lcd.print((char)223);
    lcd.print(F("C S:"));
    lcd.print(setPoint, 1);

    lcd.setCursor(0, 1);
    if (overTempTrip) {
        lcd.print(F("!OVERTEMP CUTOFF"));
        return;
    }
    if (systemEnabled && !linkOk) {
        // The single most likely failure of this rig: MATLAB not running,
        // crashed, or holding a different port. Say so unmistakably.
        lcd.print(F("HOST LINK LOST!"));
        return;
    }
    lcd.print(phaseName(phase));
    lcd.print(' ');
    uint32_t el = (now - phaseStart) / 1000;
    if (el / 60 < 10) lcd.print('0');
    lcd.print(el / 60);
    lcd.print(':');
    if (el % 60 < 10) lcd.print('0');
    lcd.print(el % 60);
    if (!isnan(temperature) && fabs(setPoint - temperature) <= SETTLE_BAND) {
        lcd.print(F(" IB"));
    }
}

// -------------------- Serial input --------------------
// Lines starting with "U:" are host commands. A bare single character is
// treated as a console key, so the harness still works from a plain
// terminal when you are testing the rig without MATLAB attached.
void handleLine(char *line, uint8_t len, unsigned long now)
{
    if (len >= 2 && line[0] == 'U' && line[1] == ':') {
        double v = atof(line + 2);
        if (v > outMax) v = outMax;
        if (v < outMin) v = outMin;
        hostCommand   = v;
        lastCommandMs = now;
        commandsRx++;
        return;
    }
    if (len == 1) {
        char c = line[0];
        if (c == 'n' || c == 'N') {
            forceAdvance = true;
            banner(F("console: SKIP current phase"));
        } else if (c == 'a' || c == 'A') {
            aborted = true;
            banner(F("console: ABORT — summarising what we have"));
            enterPhase(PH_SUMMARY);
        } else if (c == 'l' || c == 'L') {
            if (phase == PH_T4_PREP) {
                forceAdvance = true;
                banner(F("console: start the lid window now"));
            }
        }
    }
}

void readSerial(unsigned long now)
{
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (rxLen > 0) {
                rxBuf[rxLen] = '\0';
                handleLine(rxBuf, rxLen, now);
                rxLen = 0;
            }
        } else if (rxLen < sizeof(rxBuf) - 1) {
            rxBuf[rxLen++] = c;
        } else {
            rxLen = 0;   // overlong line: drop it rather than truncate-parse
        }
    }
}

// -------------------- setup --------------------
void setup()
{
    pinMode(BTN_START, INPUT_PULLUP);
    pinMode(BTN_STOP,  INPUT_PULLUP);
    pinMode(BTN_MODE,  INPUT_PULLUP);

    setLed(LED_POWER, false);
    setLed(LED_HEAT,  false);
    setLed(LED_COOL,  false);
    pinMode(LED_POWER, OUTPUT);
    pinMode(LED_HEAT,  OUTPUT);
    pinMode(LED_COOL,  OUTPUT);

    digitalWrite(RELAY_HEAT, RELAY_OFF);
    digitalWrite(RELAY_FAN,  RELAY_OFF);
    pinMode(RELAY_HEAT, OUTPUT);
    pinMode(RELAY_FAN,  OUTPUT);
    allOff();

    Serial.begin(9600);
    dht.begin();
    dhtOut.begin();

    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("MPC TEST SUITE"));
    lcd.setCursor(0, 1);
    lcd.print(F("waiting host..."));

    sessionStart  = millis();
    lastDutyTick  = millis();
    lastCommandMs = millis();
    idleStep();

    Serial.println();
    Serial.println(F("# ============================================================"));
    Serial.println(F("# AUTOMATED MPC TEST PROTOCOL — control computed on the host"));
    Serial.println(F("# Requires MPC_HOST_TEST.m running in MATLAB on this port."));
    Serial.println(F("# Protocol/metrics identical to test_harness_pid.cpp."));
#if QUICK_MODE
    Serial.println(F("# *** QUICK_MODE — durations 10x short. RESULTS ARE NOT VALID. ***"));
#endif
#if START_AT_TEST == 4
    Serial.println(F("# Order: baseline -> T4 -> summary   (START_AT_TEST=4)"));
#elif START_AT_TEST == 3
    Serial.println(F("# Order: baseline -> T3 -> T4 -> summary   (START_AT_TEST=3)"));
#else
    Serial.println(F("# Order: baseline -> T1 -> T2 -> T3 -> T4 -> summary"));
#endif
    Serial.println(F("# STOP button aborts and prints the summary at any time."));
    Serial.println(F("# ============================================================"));
    Serial.println(F("t_s,phase,test,Tin_C,RH_pct,Tamb_C,SP_C,err_C,u,duty_pct,heat,fan,in_band,link_ok,event"));

    delay(2000);
    enterPhase(PH_BASELINE);
    banner(F("logging ambient with the system OFF (no commands sent yet)"));
}

// -------------------- loop --------------------
void loop()
{
    unsigned long now = millis();

    accumulateDuty(now);
    readSerial(now);

    if (digitalRead(BTN_STOP) == LOW && phase != PH_DONE) {
        aborted = true;
        systemEnabled = false;
        allOff();
        banner(F("STOP button — aborting"));
        enterPhase(PH_SUMMARY);
    }

    setLed(LED_POWER, systemEnabled);

    humidity    = dht.readHumidity();
    temperature = dht.readTemperature();
    if (!isnan(temperature)) lastValidTemp = temperature;
    outHum  = dhtOut.readHumidity();
    outTemp = dhtOut.readTemperature();

    overTempTrip = (!isnan(temperature) && temperature >= ABSOLUTE_MAX_TEMP);
    linkOk = (now - lastCommandMs) <= COMMAND_TIMEOUT_MS;

    //---------------- Actuation ----------------
    // The host's command is applied only while everything is sane. Safety
    // is enforced here, not on the PC, exactly as main_mcp.cpp intends:
    // the Arduino never trusts the host blindly.
    if (systemEnabled && !isnan(temperature) && !overTempTrip && linkOk) {
        appliedCommand = hostCommand;
        applyActuation(appliedCommand, now);
    } else {
        appliedCommand = 0;
        allOff();
    }

    //---------------- 1 Hz tick ----------------
    if (now - lastSampleMs >= SAMPLE_MS) {
        lastSampleMs = now;
        if (isnan(temperature)) nanCount++;

        // Send the control line FIRST so the host sees it before the CSV
        // row and its reply is not delayed by a line it will discard.
        //
        // Nothing is sent during BASELINE. The host's state estimator
        // advances on the assumption that the u it returned was applied;
        // asking it for commands while the relays are parked would leave
        // its internal state describing a plant that never ran.
        if (systemEnabled && !isnan(temperature)) {
            Serial.print(F("T:"));
            Serial.print(temperature, 2);
            Serial.print(F(",SP:"));
            Serial.print(setPoint, 2);
            Serial.print(F(",A:"));
            Serial.println(isnan(outTemp) ? lastValidTemp : outTemp, 2);
        }

        if (systemEnabled && !linkOk) staleSamples++;

        if (phase == PH_BASELINE) {
            if (!isnan(temperature)) { baseSumIn  += temperature; baseNIn++;  }
            if (!isnan(outTemp))     { baseSumOut += outTemp;     baseNOut++; }
        } else if (systemEnabled) {
            updateStep(now);
        }

        logRow(overTempTrip ? F("OVERTEMP") : (systemEnabled && !linkOk ? F("NOLINK") : NULL));
    }

    //---------------- Watchdogs ----------------
    if (phase != PH_SUMMARY && phase != PH_DONE && systemEnabled &&
        (now - lastCommandMs) > LINK_LOST_ABORT_MS) {
        // Without a controller there is no experiment. Better to stop and
        // say so than to spend two hours logging an unpowered box.
        aborted = true;
        banner(F("FATAL: no host command for 2 min — is MPC_HOST_TEST.m running?"));
        enterPhase(PH_SUMMARY);
    }

    if (phase != PH_SUMMARY && phase != PH_DONE &&
        (now - sessionStart) > SESSION_MAX_MS) {
        aborted = true;
        banner(F("session time limit hit — summarising"));
        enterPhase(PH_SUMMARY);
    }

    //---------------- Phase machine ----------------
    switch (phase) {

    case PH_BASELINE:
        systemEnabled = false;
        if ((now - phaseStart) >= BASELINE_MS || forceAdvance) {
            ambientInside = baseNIn  ? baseSumIn  / baseNIn  : NAN;
            ambient       = baseNOut ? baseSumOut / baseNOut : ambientInside;
            if (isnan(ambient)) {
                banner(F("FATAL: no valid sensor reading during baseline. Check wiring."));
                aborted = true;
                enterPhase(PH_SUMMARY);
                break;
            }
            Serial.print(F("# ambient = "));
            Serial.print(ambient, 2);
            Serial.print(F(" degC (outside), inside mean "));
            Serial.print(ambientInside, 2);
            Serial.println(F(" degC"));

            // Re-arm the link watchdog: nothing has been sent during
            // baseline, so lastCommandMs is stale by construction and
            // would otherwise trip the abort immediately.
            lastCommandMs = now;

#if START_AT_TEST == 4
            Serial.print(F("# STARTING AT TEST 4. Target = "));
            Serial.print(ambient + T4_OFFSET, 2);
            Serial.println(F(" degC"));
            systemEnabled = true;
            beginStep(ambient + T4_OFFSET, 0, false,
                      SETTLE_BAND_DWELL, STEP_TIMEOUT_MS);
            enterPhase(PH_T4_STABILIZE);
#elif START_AT_TEST == 3
            Serial.print(F("# STARTING AT TEST 3. Reheat to "));
            Serial.print(ambient + HEAT_OFFSET, 2);
            Serial.print(F(", then step to "));
            Serial.print(ambient + FLOOR_OFFSET, 2);
            Serial.print(F(", then T4 at "));
            Serial.println(ambient + T4_OFFSET, 2);
            Serial.println(F("# (tests 1-2 not run — their rows are in the earlier session's log)"));
            systemEnabled = true;
            beginStep(ambient + HEAT_OFFSET, 0, false,
                      SETTLE_BAND_DWELL, STEP_TIMEOUT_MS);
            enterPhase(PH_T3_REHEAT);
#else
            Serial.print(F("# targets: T1 = "));
            Serial.print(ambient + HEAT_OFFSET, 2);
            Serial.print(F(", T2 = "));
            Serial.print(ambient + COOL_OFFSET, 2);
            Serial.print(F(", T3 = "));
            Serial.print(ambient + FLOOR_OFFSET, 2);
            Serial.print(F(", T4 = "));
            Serial.println(ambient + T4_OFFSET, 2);

            systemEnabled = true;
            beginStep(ambient + HEAT_OFFSET, 1, true,
                      SETTLE_BAND_DWELL, STEP_TIMEOUT_MS);
            enterPhase(PH_T1);
#endif
        }
        break;

    case PH_T1:
        if (stepDone(now)) {
            finishStep(now);
            banner(step.settled ? F("test 1 settled") : F("test 1 TIMED OUT"));
            systemEnabled = true;
            beginStep(ambient + COOL_OFFSET, 2, true,
                      SETTLE_BAND_DWELL, STEP_TIMEOUT_MS);
            enterPhase(PH_T2);
        }
        break;

    case PH_T2:
        if (stepDone(now)) {
            finishStep(now);
            banner(step.settled ? F("test 2 settled") : F("test 2 TIMED OUT"));
            systemEnabled = true;
            beginStep(ambient + HEAT_OFFSET, 0, false,
                      SETTLE_BAND_DWELL, STEP_TIMEOUT_MS);
            enterPhase(PH_T3_REHEAT);
        }
        break;

    case PH_T3_REHEAT:
        if (stepDone(now)) {
            if (step.settled) {
                banner(F("stable at ambient+5 — holding 2 min before the step"));
            } else {
                banner(F("WARNING: never reached ambient+5 — test 3 does NOT start"
                         " from a settled setpoint as the protocol requires"));
            }
            enterPhase(PH_T3_HOLD);
        }
        break;

    case PH_T3_HOLD:
        if ((now - phaseStart) >= HOLD_MS || forceAdvance) {
            systemEnabled = true;
            beginStep(ambient + FLOOR_OFFSET, 3, true,
                      SETTLE_BAND_DWELL, STEP_TIMEOUT_MS);
            enterPhase(PH_T3_STEP);
        }
        break;

    case PH_T3_STEP:
        if (stepDone(now)) {
            finishStep(now);
            banner(step.settled ? F("test 3 settled at the floor")
                                : F("test 3 did not reach the floor (expected)"));
            systemEnabled = true;
            beginStep(ambient + T4_OFFSET, 0, false,
                      SETTLE_BAND_DWELL, STEP_TIMEOUT_MS);
            enterPhase(PH_T4_STABILIZE);
        }
        break;

    case PH_T4_STABILIZE:
        if (stepDone(now)) {
            if (step.settled) {
                banner(F("stable — TEST 4 needs you: open the lid when prompted"));
            } else {
                banner(F("WARNING: never settled — test 4 starts from an UNSTABLE"
                         " setpoint and its recovery time will not be meaningful"));
            }
            enterPhase(PH_T4_PREP);
        }
        break;

    case PH_T4_PREP: {
        static uint32_t lastTick = 0;
        uint32_t remain = (LID_PREP_MS - min(LID_PREP_MS, now - phaseStart)) / 1000;
        if (remain != lastTick) {
            lastTick = remain;
            Serial.print(F("# lid test in "));
            Serial.print(remain);
            Serial.println(F(" s"));
        }
        if ((now - phaseStart) >= LID_PREP_MS || forceAdvance) {
            banner(F(">>> OPEN THE ENCLOSURE NOW — hold it open 5 s <<<"));
            enterPhase(PH_T4_OPEN);
        }
        break;
    }

    case PH_T4_OPEN:
        setLed(LED_HEAT, ((now / 200) % 2) == 0);
        setLed(LED_COOL, ((now / 200) % 2) == 0);
        if ((now - phaseStart) >= LID_OPEN_MS) {
            banner(F(">>> CLOSE AND RESEAL NOW <<<"));
            beginStep(setPoint, 4, true, RECOVER_DWELL, STEP_TIMEOUT_MS);
            enterPhase(PH_T4_RECOVER);
        }
        break;

    case PH_T4_RECOVER:
        if (stepDone(now)) {
            finishStep(now);
            banner(F("test 4 done — protocol complete"));
            idleStep();
            enterPhase(PH_SUMMARY);
        }
        break;

    case PH_SUMMARY:
        systemEnabled = false;
        allOff();
        printSummary();
        enterPhase(PH_DONE);
        break;

    case PH_DONE:
        systemEnabled = false;
        allOff();
        setLed(LED_POWER, ((now / 1000) % 2) == 0);
        break;

    default:
        break;
    }

    //---------------- LCD ----------------
    if (now - lastLcdMs >= LCD_MS) {
        lastLcdMs = now;
        if (phase == PH_DONE) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("TESTS COMPLETE"));
            lcd.setCursor(0, 1);
            lcd.print(nResults);
            lcd.print(F(" results logged"));
        } else {
            updateLcd(now);
        }
    }

    delay(LOOP_MS);
}
