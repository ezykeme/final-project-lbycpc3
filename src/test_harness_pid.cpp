/* =====================================================================
   Automated PID characterisation harness
   Runs the protocol end to end, unattended, and prints both the raw 1 Hz
   log and the computed metrics over Serial.

   Covers tests 1-5: heating step, cooling step, mid-run setpoint step,
   disturbance rejection, and the duty-cycle/energy comparison computed
   from the first four. The repeatability check is not implemented — to
   get it, run this firmware three times and compare the summaries.

   RUN_TEST_4_ONLY (below) selects between the full protocol (0, the
   default) and the disturbance test alone (1), for when you only need to
   redo test 4 without repeating a two-hour session.

   WHY THIS IS A SEPARATE FIRMWARE, NOT A MODE INSIDE main.cpp
   -----------------------------------------------------------
   The tests need to own the setpoint (main.cpp takes it from the pot) and
   the enable state (main.cpp takes it from the buttons). Rather than bolt a
   test mode onto the production sketch, this is a second sketch that reuses
   the SAME control law and the SAME constants, and drives them from a
   scripted state machine. Build it with:

       pio run -e test_pid -t upload
       pio device monitor -e test_pid -f log2file

   The log2file filter writes .pio/build/test_pid/monitor-*.log — that file
   IS your dataset. Every line that does not start with '#' is CSV.

   !! KEEP IN SYNC WITH main.cpp !!
   The PID block below (gains, deadband, window, clamps, computePID(),
   applyActuation()) is copied verbatim from main.cpp. If you retune the
   controller there, copy the change here or the harness will characterise
   a controller you are no longer shipping. The two files are marked with
   PID-CORE-BEGIN / PID-CORE-END to make the diff mechanical.

   WHAT IS AUTOMATED AND WHAT IS NOT
   ---------------------------------
   Tests 1, 2, 3 and 5 are fully unattended: power the board, open the
   monitor, walk away. Test 4 needs a human to physically open the lid —
   no firmware can do that. The harness reduces it to "follow the prompt":
   it counts you in, tells you exactly when to open and when to reseal, and
   times the whole thing itself. If nobody is there, the test still runs to
   completion and is flagged NO-DISTURBANCE in the summary rather than
   silently reporting a meaningless recovery time.

   RUNTIME
   -------
   Expect 60-100 minutes. Test 3 dominates: its ambient-2 target is not
   reachable on this rig (see the COOL_OFFSET note below), so that phase
   almost always runs to its full 40 minute timeout. Shorten
   STEP_TIMEOUT_MS if you would rather cap it. Set QUICK_MODE to 1 for a
   ~10x faster dry run that exercises every branch of the state machine
   (useful to verify wiring and logging before committing to a real run —
   the numbers it produces are NOT valid results).
   ===================================================================== */

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ---------------------------------------------------------------------
// 0 = real protocol. 1 = all durations shortened 10x for a smoke test.
// ---------------------------------------------------------------------
#define QUICK_MODE 0

// ---------------------------------------------------------------------
// 1 = skip tests 1-3 and run ONLY the disturbance test: baseline, warm to
// the T4 setpoint, hold it, lid event, recover, summarise. ~25-35 min
// instead of ~2.5 h. Use this to redo test 4 without repeating a session
// whose other results you already have.
// ---------------------------------------------------------------------
#define RUN_TEST_4_ONLY 0

#if QUICK_MODE
  #define TSCALE 10UL
#else
  #define TSCALE 1UL
#endif

// -------------------- PID-CORE-BEGIN (mirror of main.cpp) --------------------

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

double Kp = 150.0;
double Ki = 0.2;
double Kd = 15.0;

double integralErr = 0;
double prevErr     = 0;
int    lastErrSign = 0;
double prevMeas    = 0;
bool   pidFirstRun = true;
double pidOutput   = 0;

const double outMin      = -255.0;
const double outMax      =  255.0;
const double integralMax = 0.15 * outMax;
const double outDeadband = 64.0;

unsigned long lastPidTime = 0;
const unsigned long Ts = 1000;

unsigned long windowStartTime = 0;
const unsigned long windowSize = 2000;
unsigned long onTimeSnapshot = 0;

double computePID(double setp, double meas, double dtSeconds)
{
    double error = setp - meas;

    int errSign = (error > 0) - (error < 0);
    if (errSign != 0) {
        if (lastErrSign != 0 && errSign != lastErrSign) {
            integralErr = 0;
        }
        lastErrSign = errSign;
    }

    integralErr += error * dtSeconds;

    double iTerm = Ki * integralErr;
    if (iTerm >  integralMax) { iTerm =  integralMax; integralErr = iTerm / Ki; }
    if (iTerm < -integralMax) { iTerm = -integralMax; integralErr = iTerm / Ki; }

    double derivative = 0;
    if (!pidFirstRun) {
        derivative = -(meas - prevMeas) / dtSeconds;
    }

    double u = Kp * error + iTerm + Kd * derivative;

    double uSat = u;
    if (uSat > outMax) uSat = outMax;
    if (uSat < outMin) uSat = outMin;

    if (u != uSat) {
        integralErr -= error * dtSeconds;
    }

    prevErr = error;
    prevMeas = meas;
    pidFirstRun = false;
    return uSat;
}

// -------------------- PID-CORE-END --------------------

// ---------------------------------------------------------------------
// Actuation, instrumented.
// Same duty logic as main.cpp, but every relay write goes through
// setRelays() so the harness can integrate ON-time for the test 5
// duty-cycle/energy comparison. main.cpp writes the pins directly; that is
// the only intentional difference from the mirrored core above.
// ---------------------------------------------------------------------
bool heatOn = false;
bool fanOn  = false;

uint32_t heatTotalMs = 0;      // integrated relay ON time, whole session
uint32_t fanTotalMs  = 0;
unsigned long lastDutyTick = 0;

void setRelays(bool h, bool f)
{
    heatOn = h;
    fanOn  = f;
    digitalWrite(RELAY_HEAT, h ? RELAY_ON : RELAY_OFF);
    digitalWrite(RELAY_FAN,  f ? RELAY_ON : RELAY_OFF);
}

// Integrate ON-time. Called every loop pass, so the resolution is the loop
// period (~50 ms) — an order of magnitude finer than the 500 ms minimum
// pulse the time-proportioning can command, which is what matters.
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

void resetPid(unsigned long now)
{
    integralErr = 0;
    prevErr     = 0;
    pidOutput   = 0;
    lastErrSign = 0;
    pidFirstRun = true;
    lastPidTime = now;
}

// ---------------------------------------------------------------------
// Protocol timing. Everything the test schedule depends on lives here.
// ---------------------------------------------------------------------
const unsigned long BASELINE_MS         =  120000UL / TSCALE;  // test 1 pre-roll, system off
const unsigned long SETTLE_BAND_DWELL   =  300000UL / TSCALE;  // 5 min inside band = settled
const unsigned long HOLD_MS             =  120000UL / TSCALE;  // test 3 dwell before the step
const unsigned long LID_PREP_MS         =   30000UL / TSCALE;  // countdown before "open"
const unsigned long LID_OPEN_MS         =    5000UL;           // NOT scaled: physical spec
const unsigned long RECOVER_DWELL       =   60000UL / TSCALE;  // test 4 "recovered" dwell
const unsigned long STEP_TIMEOUT_MS     = 2400000UL / TSCALE;  // give up on a step after 40 min
const unsigned long SESSION_MAX_MS      = 5UL * 3600000UL / TSCALE;

const float SETTLE_BAND   = 0.5;   // +/- degC, the protocol's settling criterion
const float HEAT_OFFSET   = 5.0;   // test 1 target = ambient + this

// Test 4 gets its own, lower setpoint. Measured on this rig (session
// 260812-064112): three separate attempts to hold ambient+5 sat at 100%
// heater duty for 40, 40 and 23 minutes and plateaued at ambient+3.2 to
// +3.3 without ever entering the band. At a saturated setpoint there is no
// closed loop left to disturb — the controller is pinned at 255 before the
// lid opens and pinned at 255 after, so "disturbance rejection" would be
// measuring the enclosure's passive time constant, not the PID.
// ambient+2.0 leaves roughly 1.2 degC of headroom, so the loop is actively
// modulating duty when the disturbance lands. Raise it only if you also
// raise the heater's authority.
const float T4_OFFSET     = 2.0;   // test 4 target = ambient + this
const float COOL_OFFSET   = 0.0;   // test 2 target = ambient + this (see note below)
const float FLOOR_OFFSET  = -2.0;  // test 3 target = ambient + this

const unsigned long SAMPLE_MS = 1000;   // protocol says log every 1 s
const unsigned long LCD_MS    = 500;
const unsigned long LOOP_MS   = 50;

// Test 2 targets ambient, not ambient-2. The cooling actuator on this rig is
// a fan pulling AMBIENT air through the enclosure, so ambient is a hard
// thermodynamic floor — asking for ambient-2 there would guarantee a timeout
// and produce no settling time at all. Test 3 deliberately DOES ask for
// ambient-2 (the protocol's "tested floor"), and the harness reports how
// close it actually got instead of pretending it settled.

// ---------------------------------------------------------------------
// Per-test results. All metrics are computed incrementally: an Uno cannot
// hold a 3000-sample trace, so nothing is stored that isn't a running
// statistic.
// ---------------------------------------------------------------------
struct TestResult {
    uint8_t  test;          // protocol test number (1..4)
    uint8_t  settled;       // reached and held the band
    uint8_t  crossed;       // ever reached the target at all
    float    target;
    float    startTemp;
    float    ambient;
    float    overshoot;     // past the target, in the direction of travel
    float    undershoot;    // back past the target, the other way
    float    rms;           // RMS error over the whole step
    float    maxDev;        // worst |error| after first reaching the target
    float    extreme;       // hottest (heating) / coldest (cooling) point seen
    uint32_t settleMs;      // step start -> start of the window it held
    uint32_t firstInBandMs; // step start -> first touched the band
    uint32_t durMs;
    uint32_t heatMs;
    uint32_t fanMs;
};

const uint8_t MAX_RESULTS = 4;      // one per protocol test
TestResult results[MAX_RESULTS];
uint8_t nResults = 0;

// ---------------------------------------------------------------------
// The step tracker: one active measurement window.
// ---------------------------------------------------------------------
struct StepState {
    float    target;
    float    startTemp;
    bool     heatingStep;     // direction of travel
    bool     crossed;         // has the temperature reached the target yet
    bool     inBand;
    bool     settled;
    bool     record;          // store a TestResult when it finishes
    uint8_t  test;
    uint32_t startMs;
    uint32_t bandEnterMs;
    uint32_t settleMs;
    uint32_t firstInBandMs;
    uint32_t dwellRequired;
    uint32_t timeoutMs;
    uint32_t heatMs0;
    uint32_t fanMs0;
    float    postMax;         // extremes AFTER first crossing the target
    float    postMin;
    float    extreme;         // extreme over the whole step, crossing or not
    float    maxDev;
    uint64_t sumSqE4;         // sum of err^2 * 10000, integer to avoid the
    uint32_t nSamples;        // float accumulator swallowing small errors
};
StepState step;

// ---------------------------------------------------------------------
// Session state
// ---------------------------------------------------------------------
enum Phase : uint8_t {
    PH_BOOT,
    PH_BASELINE,
    PH_T1,
    PH_T2,
    PH_T3_REHEAT,
    PH_T3_HOLD,
    PH_T3_STEP,
    PH_T4_STABILIZE,
    PH_T4_PREP,
    PH_T4_OPEN,
    PH_T4_RECOVER,
    PH_SUMMARY,
    PH_DONE
};

Phase    phase        = PH_BOOT;
uint32_t phaseStart   = 0;
uint32_t sessionStart = 0;
bool     forceAdvance = false;  // 'n' on the serial console skips a phase
bool     aborted      = false;

float ambient      = NAN;       // measured during baseline, drives every target
float ambientInside = NAN;
float setPoint     = 25.0;
bool  systemEnabled = false;

float temperature = NAN;
// Last reading that wasn't NaN. Step boundaries are decided from this rather
// than from `temperature`: a single dropped DHT read at the instant a step
// begins would otherwise set the direction of travel wrong, and every
// overshoot/undershoot number for that test would come out mirrored.
float lastValidTemp = NAN;
float humidity    = NAN;
float outTemp     = NAN;
float outHum      = NAN;
uint16_t nanCount = 0;
bool  overTempTrip = false;

// Baseline averaging
float    baseSumIn = 0, baseSumOut = 0;
uint16_t baseNIn = 0, baseNOut = 0;

// Test 4 bookkeeping
uint32_t lidCloseMs = 0;
float    lidPeakDev = 0;

unsigned long lastSampleMs = 0;
unsigned long lastLcdMs    = 0;

// ---------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------
const __FlashStringHelper *phaseName(Phase p)
{
    switch (p) {
        case PH_BOOT:          return F("BOOT");
        case PH_BASELINE:      return F("BASELINE");
        case PH_T1:            return F("T1_HEAT");
        case PH_T2:            return F("T2_COOL");
        case PH_T3_REHEAT:     return F("T3_REHEAT");
        case PH_T3_HOLD:       return F("T3_HOLD");
        case PH_T3_STEP:       return F("T3_STEP");
        case PH_T4_STABILIZE:  return F("T4_STAB");
        case PH_T4_PREP:       return F("T4_PREP");
        case PH_T4_OPEN:       return F("T4_OPEN");
        case PH_T4_RECOVER:    return F("T4_RECOV");
        case PH_SUMMARY:       return F("SUMMARY");
        default:               return F("DONE");
    }
}

void printTemp(float v)
{
    if (isnan(v)) Serial.print(F("NaN"));
    else          Serial.print(v, 2);
}

// One CSV row per second. Column order is fixed; '#' lines are commentary
// and can be stripped (pandas: read_csv(f, comment='#')).
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
    Serial.print(pidOutput, 1);
    Serial.print(',');
    Serial.print(fabs(pidOutput) > outDeadband ? (int)(100.0 * fabs(pidOutput) / outMax) : 0);
    Serial.print(',');
    Serial.print(heatOn ? 1 : 0);
    Serial.print(',');
    Serial.print(fanOn ? 1 : 0);
    Serial.print(',');
    Serial.print((!isnan(temperature) && fabs(setPoint - temperature) <= SETTLE_BAND) ? 1 : 0);
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

// ---------------------------------------------------------------------
// Step machinery
// ---------------------------------------------------------------------
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

    // A disturbance test starts AT the setpoint, so the temperature has
    // already "crossed" — every excursion from here counts as deviation.
    step.crossed = (!isnan(t0) && fabs(target - t0) <= SETTLE_BAND);

    setPoint = target;
    resetPid(now);

    Serial.print(F("# --- step: target "));
    Serial.print(target, 2);
    Serial.print(F(" degC, from "));
    printTemp(step.startTemp);
    Serial.print(F(" degC, direction "));
    Serial.println(step.heatingStep ? F("HEAT") : F("COOL"));
}

// Called once per sample tick (1 Hz) while a step is active.
void updateStep(unsigned long now)
{
    if (isnan(temperature)) return;

    float err = step.target - temperature;
    float aerr = fabs(err);

    // RMS accumulator. err^2 peaks near 25 for a 5 degC step; *10000 keeps
    // 0.01 degC resolution and an integer sum stays exact for the whole run,
    // where a float accumulator would quietly drop the small late-run errors
    // that the metric is actually about.
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

    // Overshoot only means anything once the target has been reached; before
    // that, "distance from target" is just the approach, not an excursion.
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
        // Left the band: the dwell restarts from scratch.
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
        r.overshoot  = NAN;         // never reached the target; no excursion exists
        r.undershoot = NAN;
    } else if (step.heatingStep) {
        r.overshoot  = max(0.0f, step.postMax - step.target);
        r.undershoot = max(0.0f, step.target - step.postMin);
    } else {
        r.overshoot  = max(0.0f, step.target - step.postMin);   // past it, downward
        r.undershoot = max(0.0f, step.postMax - step.target);
    }
}

// ---------------------------------------------------------------------
// Phase control
// ---------------------------------------------------------------------
void enterPhase(Phase p)
{
    phase = p;
    phaseStart = millis();
    forceAdvance = false;
    Serial.print(F("# ===== PHASE "));
    Serial.print(phaseName(p));
    Serial.println(F(" ====="));
}

void idleStep()  // keeps the CSV's test column meaningful between steps
{
    step.test = 0;
}

// ---------------------------------------------------------------------
// Summary
// ---------------------------------------------------------------------
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
    Serial.print(F("# gains Kp="));  Serial.print(Kp);
    Serial.print(F(" Ki="));         Serial.print(Ki);
    Serial.print(F(" Kd="));         Serial.print(Kd);
    Serial.print(F(" deadband="));   Serial.print(outDeadband);
    Serial.print(F(" window_ms="));  Serial.println(windowSize);
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

    // ---- Test 3 caveat: the floor may simply not be reachable ----
    for (uint8_t i = 0; i < nResults; i++) {
        if (results[i].test == 3 && !results[i].settled) {
            Serial.print(F("# TEST 3: never settled at "));
            Serial.print(results[i].target, 2);
            Serial.print(F(" degC. Coldest reached "));
            Serial.print(results[i].extreme, 2);
            Serial.println(F(" degC — expected: the fan cannot cool below ambient."));
            Serial.print(F("#   Report the floor as the tested limit; recovery time to "));
            Serial.println(F("within the band is NA by construction."));
        }
    }

    // ---- Test 4: disturbance rejection ----
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

    // ---- Test 5: duty cycle / energy comparison ----
    // Summed over the four recorded tests. The unrecorded reheat/stabilise
    // phases between them are harness overhead, not part of any controller
    // comparison, so their duty is excluded — it appears only in the session
    // totals printed below.
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

// ---------------------------------------------------------------------
// LCD
// ---------------------------------------------------------------------
void updateLcd(unsigned long now)
{
    lcd.clear();
    lcd.setCursor(0, 0);

    if (phase == PH_T4_PREP || phase == PH_T4_OPEN) {
        // The one phase a human has to act on gets the whole display.
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

// ---------------------------------------------------------------------
// Console commands. Nothing here is required for an unattended run — they
// exist so a run can be rescued rather than restarted.
// ---------------------------------------------------------------------
void handleConsole()
{
    while (Serial.available() > 0) {
        int c = Serial.read();
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

// ---------------------------------------------------------------------
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
    lcd.print(F("PID TEST SUITE"));
    lcd.setCursor(0, 1);
    lcd.print(F("starting..."));

    sessionStart = millis();
    lastDutyTick = millis();
    idleStep();

    Serial.println();
    Serial.println(F("# ============================================================"));
    Serial.println(F("# AUTOMATED PID TEST PROTOCOL — controller: PID (main.cpp core)"));
    Serial.println(F("# Runs unattended. Test 4 prompts you to open the lid for 5 s."));
    Serial.println(F("# Console keys: n=skip phase, a=abort+summary, l=start lid window"));
#if QUICK_MODE
    Serial.println(F("# *** QUICK_MODE — durations 10x short. RESULTS ARE NOT VALID. ***"));
#endif
    Serial.println(F("# Order: baseline -> T1 -> T2 -> T3 -> T4 -> summary"));
    Serial.println(F("# Expect 60-100 min. Metrics are computed on-board; the summary"));
    Serial.println(F("# block at the end is the deliverable, the CSV is the evidence."));
    Serial.println(F("# ============================================================"));
    Serial.println(F("t_s,phase,test,Tin_C,RH_pct,Tamb_C,SP_C,err_C,u,duty_pct,heat,fan,in_band,event"));

    delay(2000);
    enterPhase(PH_BASELINE);
    banner(F("logging ambient with the system OFF"));
}

void loop()
{
    unsigned long now = millis();

    accumulateDuty(now);
    handleConsole();

    // Emergency stop stays live for the whole session. This sketch drives a
    // heater unattended for over an hour; a physical kill that does not
    // depend on the console being attached is not optional.
    if (digitalRead(BTN_STOP) == LOW && phase != PH_DONE) {
        aborted = true;
        systemEnabled = false;
        allOff();
        banner(F("STOP button — aborting"));
        enterPhase(PH_SUMMARY);
    }

    setLed(LED_POWER, systemEnabled);

    //---------------- Sensors ----------------
    // The DHT library serves a cached sample if polled faster than its 2 s
    // minimum, so the 1 Hz protocol log genuinely updates every other row.
    // That is a sensor limit, not a logging bug — noted here because it puts
    // a 2 s floor under every settling time this harness can resolve.
    humidity    = dht.readHumidity();
    temperature = dht.readTemperature();
    if (!isnan(temperature)) lastValidTemp = temperature;
    outHum  = dhtOut.readHumidity();
    outTemp = dhtOut.readTemperature();

    overTempTrip = (!isnan(temperature) && temperature >= ABSOLUTE_MAX_TEMP);

    //---------------- Control ----------------
    if (systemEnabled && !isnan(temperature) && !overTempTrip) {
        if (now - lastPidTime >= Ts) {
            double dtSeconds = (now - lastPidTime) / 1000.0;
            pidOutput = computePID(setPoint, temperature, dtSeconds);
            lastPidTime = now;
        }
        applyActuation(pidOutput, now);
    } else {
        allOff();
        if (!systemEnabled) integralErr = 0;
    }

    //---------------- 1 Hz sample tick ----------------
    if (now - lastSampleMs >= SAMPLE_MS) {
        lastSampleMs = now;
        if (isnan(temperature)) nanCount++;

        if (phase == PH_BASELINE) {
            if (!isnan(temperature)) { baseSumIn  += temperature; baseNIn++;  }
            if (!isnan(outTemp))     { baseSumOut += outTemp;     baseNOut++; }
        } else if (systemEnabled) {
            updateStep(now);
        }

        logRow(overTempTrip ? F("OVERTEMP") : NULL);
    }

    //---------------- Session watchdog ----------------
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
                // Nothing measurable — refuse to invent an ambient and run a
                // 90 minute test against a made-up setpoint.
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
#if RUN_TEST_4_ONLY
            Serial.print(F("# TEST 4 ONLY. Target = "));
            Serial.print(ambient + T4_OFFSET, 2);
            Serial.println(F(" degC (ambient + 2.0)"));
            systemEnabled = true;
            beginStep(ambient + T4_OFFSET, 0, false,
                      SETTLE_BAND_DWELL, STEP_TIMEOUT_MS);
            enterPhase(PH_T4_STABILIZE);
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
            // Test 2 starts immediately, per the protocol.
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
            // Test 3 needs a stable ambient+5 again before it can step down.
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
            // Say which of the two happened. The old wording claimed "stable"
            // on a timeout as well, which is exactly the case where test 4's
            // premise ("once stable at any setpoint") does NOT hold — and the
            // previous session hit it three times without the log saying so.
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
        // Count the operator in. Nothing about the measurement depends on
        // them reacting perfectly: the clock that matters starts at reseal.
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
        // Blink both indicators so the prompt is visible from the bench with
        // no monitor in view.
        setLed(LED_HEAT, ((now / 200) % 2) == 0);
        setLed(LED_COOL, ((now / 200) % 2) == 0);
        if ((now - phaseStart) >= LID_OPEN_MS) {
            banner(F(">>> CLOSE AND RESEAL NOW <<<"));
            lidCloseMs = now;
            lidPeakDev = 0;
            // Recovery is measured from the reseal, against the setpoint the
            // system was already holding — so the step tracker is restarted
            // with the same target and a shorter dwell.
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
        setLed(LED_POWER, ((now / 1000) % 2) == 0);   // slow blink = finished
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
