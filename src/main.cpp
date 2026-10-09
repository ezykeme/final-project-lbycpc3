/* =====================================================================
   Smart Building Temperature Control — PID version
   Ported from: HVAC_PID_VS_MPC.m (MATLAB)

   What changed vs. the original sketch:
     - Original: simple ON/OFF hysteresis (heater/fan fully on or off).
     - Now: a real discrete-time PID loop (same structure as the MATLAB
       script's manual PID loop, incl. anti-windup) computes a continuous
       "power demand" every sample period, which is then applied to the
       relays using TIME-PROPORTIONING (software PWM for mechanical
       relays), since relays can't be driven with analogWrite().

   Note on MPC: the MATLAB script also builds an MPC controller using the
   Model Predictive Control Toolbox. That controller solves a constrained
   QP every step using a preview of future disturbances (outdoor temp,
   occupancy, solar gain). An Arduino Uno has neither the memory nor the
   floating-point throughput to run that optimization online, and this
   sketch has no outdoor/occupancy/solar sensors to preview anyway. If you
   need the MPC side running against real hardware, the usual pattern is:
   run the MPC on a Pi/ESP32/PC (in MATLAB or Python), and have it send a
   setpoint or duty-cycle command to this Arduino over Serial — happy to
   wire that up if useful.
   ===================================================================== */

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// -------------------- Pins --------------------
#define DHTPIN      2       // zone sensor, INSIDE the enclosure (control input)
#define DHTTYPE     DHT22

// Ambient/reference sensor, OUTSIDE the enclosure near the fan. This rig is
// not a real HVAC zone — the "outdoor" air the fan pulls in is what actually
// bounds how far cooling can go — so this reading is logged as the reference
// against which the inside temperature should be judged. It is NOT fed into
// the PID: control still runs off the inside sensor only.
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

// -------------------- Relay polarity --------------------
// Most cheap opto-isolated relay boards (SRD-05VDC / blue boards with a
// JD-VCC jumper) are ACTIVE LOW: the coil energizes when the Arduino pin is
// pulled LOW. Writing HIGH/LOW literally, as the old code did, then means
// the heater is energized in every state EXCEPT the heating window — which
// looks like "the heater relay never turns off, even while cooling".
// Set to 0 if your board energizes on HIGH instead.
#define RELAY_ACTIVE_LOW 1

#if RELAY_ACTIVE_LOW
  #define RELAY_ON   LOW
  #define RELAY_OFF  HIGH
#else
  #define RELAY_ON   HIGH
  #define RELAY_OFF  LOW
#endif

// -------------------- LED polarity --------------------
// Same trap as the relays, and worth making explicit rather than assuming.
// 0 = LED wired cathode -> resistor -> GND, anode -> Arduino pin. HIGH lights
//     it. This is the normal "LED straight into a digital pin" wiring.
// 1 = LED wired anode -> +5V, cathode -> resistor -> Arduino pin (common
//     anode, and how most prewired 3-LED indicator modules are built). The
//     pin SINKS the current, so LOW lights it and HIGH turns it off.
// If the LEDs behave inverted (lit when they should be dark), flip this.
#define LED_ACTIVE_LOW 0

#if LED_ACTIVE_LOW
  #define LED_ON   LOW
  #define LED_OFF  HIGH
#else
  #define LED_ON   HIGH
  #define LED_OFF  LOW
#endif

// Every LED write goes through here so the polarity switch above is the only
// place the wiring is encoded. Writing HIGH/LOW literally, as the old code
// did, is what makes a polarity mistake look like "the LEDs never turn on".
inline void setLed(uint8_t pin, bool on)
{
    digitalWrite(pin, on ? LED_ON : LED_OFF);
}

// -------------------- Objects --------------------
DHT dht(DHTPIN, DHTTYPE);
DHT dhtOut(DHTPIN_OUT, DHTTYPE_OUT);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// -------------------- System state --------------------
bool systemEnabled = false;
bool autoMode = true;              // AUTO = PID, MANUAL = everything off (as before)
bool lastModeButton = HIGH;

float temperature = 0;
float humidity = 0;
float setPoint = 25.0;

// Ambient reference, Serial-logged only — never used by the controller and
// never shown on the 16x2 LCD (no room for it without crowding out T/S).
float outTemperature = NAN;
float outHumidity = NAN;

// -------------------- Hard safety limit --------------------
// Independent of the setpoint and of the PID: no matter what the pot is
// dialled to, the heater is cut once the zone reaches this temperature.
// This is the backstop for a PTC with no heatsink — a slow sensor, poor
// airflow, or a hot spot the DHT can't see would otherwise leave the PID
// calling for heat indefinitely.
const float ABSOLUTE_MAX_TEMP = 40.0;   // [degC]
bool overTempTrip = false;              // latched-per-sample state, for the LCD

// -------------------- PID gains --------------------
// Tuned on the actual rig, NOT from MATLAB. The pidtune() gains were sized for
// a fitted RC room model (R=2.0 degC/kW, C=2.5 kWh/degC) on a 60 s sample and do
// not transfer to a small enclosure with a PTC and a fan on a 1 s sample.
//
// Kp is NOT free here. With time-proportioning, the shortest pulse the loop can
// command is (outDeadband/outMax)*windowSize, and that pulse must not move the
// box further than the band it is holding:
//
//     P_heater * (D/outMax) * windowSize / C  <=  D / Kp
//     => Kp * P_heater * windowSize  <=  outMax * C          (D cancels)
//
// This 24x19x24 cm insulated enclosure holds ~11 L of air, so C is only about
// 60 J/K counting the PTC mass and inner wall skin. With a ~50 W PTC that caps
// Kp*windowSize at ~306. Kp=150 with windowSize=2 s sits just inside it.
//
// Note what cancels: widening outDeadband does NOT buy tolerance, because it
// lengthens the minimum pulse by exactly as much as it widens the band. Only
// heater power, thermal mass, or the window can move this limit.
double Kp = 150.0;  // relay "power units" per degC of error
double Ki = 0.2;    // relay "power units" per (degC*s) of accumulated error
                    // 0.5 wound up too fast; 0.1 unwound too slowly (0.01/s at a
                    // 0.1 degC error). With the crossing reset actually working,
                    // the integral no longer has to bleed off a large debt.
double Kd = 15.0;   // relay "power units" per (degC/s) of measurement rate

// -------------------- PID working variables --------------------
double integralErr = 0;
double prevErr = 0;
int    lastErrSign = 0;             // sign of the last NON-ZERO error, for crossing detection
double prevMeas = 0;                // for derivative-on-measurement
bool   pidFirstRun = true;          // suppresses the derivative kick on the first step
double pidOutput = 0;               // signed "power" demand, + = heat, - = cool

const double outMin = -255.0;       // symmetric actuator limits (like Qmin/Qmax in MATLAB)
const double outMax =  255.0;

// Hard cap on how much authority the integral alone may claim. The integral was
// the source of two separate faults on this rig: it reached -184 (35% cooling)
// while the box sat BELOW setpoint and took ~30 min to unwind, and later held
// +27.7 (11% heat) at dead-on setpoint and drove the box straight past it. With
// Kp now at 255 the proportional term dominates and this is only a small trim.
const double integralMax = 0.15 * outMax;   // ~38 units, i.e. 15% duty

// Minimum |output| that closes a relay — sets BOTH the idle band and the smallest
// duty the actuators are ever asked for:
//   idle band  = outDeadband / Kp     = 64/150  = +/-0.43 degC around setpoint
//   first duty = outDeadband / outMax = 64/255  = 25% = 500 ms of a 2 s window
// 500 ms is about the least that gets a mechanical relay closed and a fan moving
// air. 1.0 (0.4% duty) meant the relays twitched and did nothing.
const double outDeadband = 64.0;

unsigned long lastPidTime = 0;
const unsigned long Ts = 1000;      // PID sample time [ms]  (== MATLAB's Ts concept)

// -------------------- Time-proportioning (software PWM for relays) ------
unsigned long windowStartTime = 0;
const unsigned long windowSize = 2000;  // relay duty-cycle window [ms]
// 10 s was chosen to spare the relay contacts, but on an 11 L enclosure it is far
// too coarse: the shortest commandable pulse (25% of the window) was 2.5 s of PTC
// = ~125 J = ~2 degC, eight times the band the loop was trying to hold. At 2 s the
// minimum pulse is ~500 ms = ~25 J = ~0.4 degC, which matches the band.
// Cost: the relay may now switch up to ~30 times/minute instead of ~6. That is
// well inside mechanical life but spends electrical life faster — acceptable for
// bench work, and the reason a MOSFET on the PTC is the real answer.
unsigned long onTimeSnapshot = 0;       // ON duration for the CURRENT window, latched at
                                         // window start so the duty cycle can't shift
                                         // mid-window when the PID updates

// -------------------- LCD refresh (independent of PID/relay timing) -----
unsigned long lastLcdTime = 0;
const unsigned long lcdInterval = 500;

// --------------------------------------------------

void allOff()
{
    digitalWrite(RELAY_HEAT, RELAY_OFF);
    digitalWrite(RELAY_FAN, RELAY_OFF);
    setLed(LED_HEAT, false);
    setLed(LED_COOL, false);
}

// Discrete PID step. Structurally the MATLAB loop from HVAC_PID_VS_MPC.m, with
// three departures forced by what the hardware actually does (see the serial
// capture that motivated them):
//   1. integral term hard-clamped, not just back-calculated on saturation
//   2. integral dumped when the error changes sign
//   3. derivative taken on the MEASUREMENT, not on the error
double computePID(double setp, double meas, double dtSeconds)
{
    double error = setp - meas;

    // (2) Zero-crossing reset. Once the temperature crosses the setpoint, an
    // integral accumulated while approaching from the other side is obsolete —
    // it only buys overshoot. Dumping it is what actually releases the relay at
    // the setpoint instead of 45 minutes later. Costs a small bump and a looser
    // steady state; worth it when the actuator is a relay anyway.
    //
    // Compare against the last NON-ZERO error, not the previous sample. The DHT
    // quantizes to 0.1 degC, so the error parks at exactly 0.0 for long stretches
    // and the sequence is (+ -> 0 -> -). Testing prevErr directly sees 0 on the
    // step that matters and never fires — measured: the integral held 27.68
    // (11% heat duty) at dead-on setpoint and drove the box past it.
    int errSign = (error > 0) - (error < 0);        // -1, 0, or +1
    if (errSign != 0) {
        if (lastErrSign != 0 && errSign != lastErrSign) {
            integralErr = 0;
        }
        lastErrSign = errSign;
    }

    integralErr += error * dtSeconds;

    // (1) Clamp the integral CONTRIBUTION, then back out the state so the
    // stored integral can never drift beyond what the clamp allows.
    double iTerm = Ki * integralErr;
    if (iTerm >  integralMax) { iTerm =  integralMax; integralErr = iTerm / Ki; }
    if (iTerm < -integralMax) { iTerm = -integralMax; integralErr = iTerm / Ki; }

    // (3) Derivative on measurement. With a constant setpoint this is identical
    // to d(error)/dt, but it ignores setpoint steps completely — which matters
    // because the pot dithers by one ADC count (0.15 degC) and each wobble was
    // injecting a Kd*0.1 = 1.5 unit kick straight into the output.
    double derivative = 0;
    if (!pidFirstRun) {
        derivative = -(meas - prevMeas) / dtSeconds;
    }

    double u = Kp * error + iTerm + Kd * derivative;

    double uSat = u;
    if (uSat > outMax) uSat = outMax;
    if (uSat < outMin) uSat = outMin;

    // Classic back-calculation, still worth keeping for the case where the
    // proportional term alone saturates the output.
    if (u != uSat) {
        integralErr -= error * dtSeconds;
    }

    prevErr = error;
    prevMeas = meas;
    pidFirstRun = false;
    return uSat;
}

// Apply the signed PID output to the relays using time-proportioning.
// |pidOutput|/outMax fraction of each `windowSize` window is spent ON.
// The ON duration is latched once per window (textbook time-proportioning):
// applyActuation() runs every ~100 ms loop but the PID only updates once per
// second, so recomputing onTime continuously would let the duty cycle of a
// window drift after it had already started.
void applyActuation(double u, unsigned long now)
{
    double magnitude = fabs(u);

    if (now - windowStartTime >= windowSize) {
        windowStartTime = now;
        onTimeSnapshot = (unsigned long)((magnitude / outMax) * windowSize);
    }
    unsigned long elapsed = now - windowStartTime;

    bool heating = (u > 0);
    // Deadband uses the LIVE magnitude so a collapsing demand releases the
    // relay immediately instead of riding out the rest of the window.
    bool demanding = (magnitude > outDeadband);
    bool relayOn   = demanding && (elapsed < onTimeSnapshot);

    // Relays follow the duty PULSE; the LEDs follow the DEMAND.
    //
    // These used to be the same signal, which made the LEDs look dead. At the
    // smallest duty the loop can command the relay is closed for only 500 ms
    // of every 2 s window, so a pulse-driven LED is dark 75% of the time and
    // reads as "off" at a glance — and it went fully dark for minutes at a
    // time whenever the error sat inside the deadband, which is the NORMAL
    // holding state. The relay has to pulse, because that is how a mechanical
    // relay does proportional output. An indicator LED has no such excuse:
    // it should mean "the controller is calling for heat/cool", which is
    // exactly what the LCD already reports. Now they agree.
    digitalWrite(RELAY_HEAT, (relayOn &&  heating) ? RELAY_ON : RELAY_OFF);
    digitalWrite(RELAY_FAN,  (relayOn && !heating) ? RELAY_ON : RELAY_OFF);

    setLed(LED_HEAT, demanding &&  heating);
    setLed(LED_COOL, demanding && !heating);
}

void setup()
{
    pinMode(BTN_START, INPUT_PULLUP);
    pinMode(BTN_STOP, INPUT_PULLUP);
    pinMode(BTN_MODE, INPUT_PULLUP);

    // Drive the pins to the OFF level BEFORE making them outputs: a fresh
    // output pin starts LOW, which on an active-low board would click the
    // relays on (or light the LEDs) for a moment during boot.
    setLed(LED_POWER, false);
    setLed(LED_HEAT, false);
    setLed(LED_COOL, false);
    pinMode(LED_POWER, OUTPUT);
    pinMode(LED_HEAT, OUTPUT);
    pinMode(LED_COOL, OUTPUT);

    digitalWrite(RELAY_HEAT, RELAY_OFF);
    digitalWrite(RELAY_FAN, RELAY_OFF);
    pinMode(RELAY_HEAT, OUTPUT);
    pinMode(RELAY_FAN, OUTPUT);

    allOff();
    setLed(LED_POWER, false);

    Serial.begin(9600);
    dht.begin();
    dhtOut.begin();

    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Smart Temp Ctrl");
    lcd.setCursor(0, 1);
    lcd.print("PID Init...");

    // LED self-test, run during the splash so it costs no extra boot time.
    // POWER -> HEAT -> COOL light in turn, then all three together, then off.
    // This is here to make the failure mode diagnosable: if a LED stays dark
    // through this sweep, the controller is not involved at all — it is the
    // wiring, a dead LED, a missing/oversized series resistor, or LED_ACTIVE_LOW
    // set the wrong way. If they all sweep correctly but stay dark in
    // operation, the fault is upstream in the control logic instead.
    const uint8_t testLeds[3] = { LED_POWER, LED_HEAT, LED_COOL };
    for (uint8_t i = 0; i < 3; i++) {
        setLed(testLeds[i], true);
        delay(400);
    }
    delay(400);
    for (uint8_t i = 0; i < 3; i++) {
        setLed(testLeds[i], false);
    }

    lastPidTime = millis();
    windowStartTime = millis();
}

void loop()
{
    unsigned long now = millis();

    //---------------- Buttons ----------------
    if (digitalRead(BTN_START) == LOW) {
        systemEnabled = true;
        delay(250);
    }

    if (digitalRead(BTN_STOP) == LOW) {
        systemEnabled = false;
        // Reset PID state on stop so it doesn't "remember" a stale integral
        integralErr = 0;
        prevErr = 0;
        pidOutput = 0;
        lastErrSign = 0;
        pidFirstRun = true;   // no derivative kick off a stale prevMeas
        // Re-base the sample clock too: without this, the first dt after a
        // restart is the whole time the system spent switched off, which
        // skews that step's derivative and integral.
        lastPidTime = millis();
        delay(250);
    }

    bool currentMode = digitalRead(BTN_MODE);
    if (lastModeButton == HIGH && currentMode == LOW) {
        autoMode = !autoMode;
        // Same reasoning as STOP: MANUAL parks the actuators, so the PID
        // must not resume on AUTO with the integral it had accumulated
        // before, nor with a dt spanning the whole MANUAL period.
        integralErr = 0;
        prevErr = 0;
        pidOutput = 0;
        lastErrSign = 0;
        pidFirstRun = true;
        lastPidTime = millis();
        delay(250);
    }
    lastModeButton = currentMode;

    //---------------- Power LED ----------------
    setLed(LED_POWER, systemEnabled);

    //---------------- Read Sensors ----------------
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    // Ambient reference near the fan. The DHT library serves a cached value
    // if polled faster than its 2 s minimum interval, so calling it every
    // loop costs nothing extra. Reference only — see note at DHTPIN_OUT.
    outHumidity = dhtOut.readHumidity();
    outTemperature = dhtOut.readTemperature();

    //---------------- Potentiometer -> Setpoint ----------------
    // One ADC count spans 0.15 degC over this 20.0-35.0 range, so the pot's
    // normal +/-1 count of noise flips the setpoint between adjacent tenths.
    // The serial capture showed it bouncing 25.50 <-> 25.40, and each bounce
    // hit the output with a Kp*0.1 = 4 unit step. Only accept a real move.
    int pot = analogRead(POT_PIN);
    float potSetPoint = map(pot, 0, 1023, 200, 350) / 10.0;
    if (fabs(potSetPoint - setPoint) >= 0.2) {
        setPoint = potSetPoint;
    }

    //---------------- Control (PID @ fixed Ts) ----------------
    // Hard ceiling, evaluated before anything is actuated. NaN compares
    // false here, but a NaN reading already falls through to allOff() below.
    overTempTrip = (!isnan(temperature) && temperature >= ABSOLUTE_MAX_TEMP);

    if (systemEnabled && autoMode && !isnan(temperature)) {
        if (now - lastPidTime >= Ts) {
            double dtSeconds = (now - lastPidTime) / 1000.0;
            pidOutput = computePID(setPoint, temperature, dtSeconds);
            lastPidTime = now;
        }
        if (overTempTrip) {
            allOff();          // safety wins over the controller, unconditionally
        } else {
            applyActuation(pidOutput, now);
        }
    } else {
        allOff();
        if (!systemEnabled) {
            integralErr = 0; // don't let error build up while off
        }
    }

    //---------------- LCD (throttled, not on every loop) ----------------
    if (now - lastLcdTime >= lcdInterval) {
        lastLcdTime = now;
        lcd.clear();

        lcd.setCursor(0, 0);
        lcd.print("T:");
        lcd.print(temperature, 1);
        lcd.print((char)223);
        lcd.print("C S:");
        lcd.print(setPoint, 1);

        lcd.setCursor(0, 1);
        if (overTempTrip) {
            // Say so explicitly — otherwise the display would still read
            // "PID HEAT 80%" while the safety cutoff holds everything off.
            lcd.print("!OVERTEMP CUTOFF");
        } else if (!systemEnabled) {
            lcd.print("SYSTEM OFF");
        } else if (!autoMode) {
            lcd.print("MANUAL/OFF");
        } else {
            lcd.print("PID ");
            // Same threshold the relays use, so the display never claims
            // HEAT/COOL while the deadband is holding both relays open.
            if (pidOutput > outDeadband) {
                lcd.print("HEAT ");
                lcd.print((int)(100.0 * fabs(pidOutput) / outMax));
                lcd.print("%");
            } else if (pidOutput < -outDeadband) {
                lcd.print("COOL ");
                lcd.print((int)(100.0 * fabs(pidOutput) / outMax));
                lcd.print("%");
            } else {
                lcd.print("IDLE");
            }
        }
    }

    //---------------- Serial (diagnostics) ----------------
    Serial.print("Temp: ");
    Serial.print(temperature);
    Serial.print("  Humidity: ");
    Serial.print(humidity);
    Serial.print("  OutTemp: ");
    if (isnan(outTemperature)) Serial.print("--"); else Serial.print(outTemperature);
    Serial.print("  OutHum: ");
    if (isnan(outHumidity)) Serial.print("--"); else Serial.print(outHumidity);
    Serial.print("  dT(in-out): ");
    if (isnan(outTemperature) || isnan(temperature)) Serial.print("--");
    else Serial.print(temperature - outTemperature);
    Serial.print("  SetPoint: ");
    Serial.print(setPoint);
    Serial.print("  PIDout: ");
    Serial.print(pidOutput);
    Serial.print("  Enabled: ");
    Serial.print(systemEnabled);
    Serial.print("  Mode: ");
    Serial.print(autoMode ? "AUTO-PID" : "MANUAL");
    Serial.println(overTempTrip ? "  [OVERTEMP CUTOFF]" : "");

    delay(100); // faster loop than before since relay timing is now handled
                // by applyActuation()/windowSize, not this delay
}
