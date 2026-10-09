/* =====================================================================
   Smart Building Temperature Control — MPC (host-driven) version
   The Arduino does NOT compute MPC. It:
     1) reads temperature + setpoint
     2) sends them to a PC over Serial
     3) receives a duty command back from the PC (which is running the
        actual MPC optimization, e.g. in MATLAB using mpcmove())
     4) applies that command to the relays via time-proportioning
     5) independently enforces safety cutoffs, regardless of whether the
        PC is connected, responding, or sending sane values

   This keeps the Arduino "dumb but safe" — it never trusts the host
   blindly. If serial goes quiet, or a command looks unsafe, it shuts
   the actuators off on its own.
   ===================================================================== */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// -------------------- Pins --------------------
#define DHTPIN      2
#define DHTTYPE     DHT22

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

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool systemEnabled = false;
bool autoMode = true;
bool lastModeButton = HIGH;

float temperature = 0;
float humidity = 0;
float setPoint = 25.0;

// -------------------- Command from host --------------------
double hostCommand = 0;             // last duty command received, -255..255
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT_MS = 5000; // if no fresh command in this long, go safe

const double outMin = -255.0;
const double outMax =  255.0;

// -------------------- Independent safety --------------------
const float ABSOLUTE_MAX_TEMP = 40.0;  // hard ceiling regardless of host commands

// -------------------- Time-proportioning --------------------
unsigned long windowStartTime = 0;
const unsigned long windowSize = 4000; // shorter window than the PID version, since
                                        // MPC's Ts should be close to your identified
                                        // plant's sample time, not a fixed 10s guess
unsigned long onTimeSnapshot = 0;

// -------------------- Sample timing (matches host's Ts) --------------------
unsigned long lastSampleTime = 0;
const unsigned long Ts = 1000; // 1s sample period -- MUST match the Ts used to build
                                // the MPC object on the host side

unsigned long lastLcdTime = 0;
const unsigned long lcdInterval = 500;

void allOff()
{
    digitalWrite(RELAY_HEAT, RELAY_OFF);
    digitalWrite(RELAY_FAN, RELAY_OFF);
    digitalWrite(LED_HEAT, LOW);
    digitalWrite(LED_COOL, LOW);
}

void applyActuation(double u, unsigned long now)
{
    if (now - windowStartTime >= windowSize) {
        windowStartTime = now;
        onTimeSnapshot = (unsigned long)((fabs(u) / outMax) * windowSize);
    }
    unsigned long elapsed = now - windowStartTime;

    bool heating = (u > 0);
    bool active = (elapsed < onTimeSnapshot) && (fabs(u) > 1.0);

    if (active && heating) {
        digitalWrite(RELAY_HEAT, RELAY_ON);
        digitalWrite(RELAY_FAN, RELAY_OFF);
        digitalWrite(LED_HEAT, HIGH);
        digitalWrite(LED_COOL, LOW);
    } else if (active && !heating) {
        digitalWrite(RELAY_HEAT, RELAY_OFF);
        digitalWrite(RELAY_FAN, RELAY_ON);
        digitalWrite(LED_HEAT, LOW);
        digitalWrite(LED_COOL, HIGH);
    } else {
        allOff();
    }
}

// Parses a line like "U:180" or "U:-90" from the host.
// Returns true if a valid command was parsed.
bool parseHostCommand(String line, double &outVal)
{
    line.trim();
    if (!line.startsWith("U:")) return false;
    String numPart = line.substring(2);
    outVal = numPart.toFloat();
    if (outVal > outMax) outVal = outMax;
    if (outVal < outMin) outVal = outMin;
    return true;
}

void setup()
{
    pinMode(BTN_START, INPUT_PULLUP);
    pinMode(BTN_STOP, INPUT_PULLUP);
    pinMode(BTN_MODE, INPUT_PULLUP);

    pinMode(LED_POWER, OUTPUT);
    pinMode(LED_HEAT, OUTPUT);
    pinMode(LED_COOL, OUTPUT);

    digitalWrite(RELAY_HEAT, RELAY_OFF);
    digitalWrite(RELAY_FAN, RELAY_OFF);
    pinMode(RELAY_HEAT, OUTPUT);
    pinMode(RELAY_FAN, OUTPUT);

    allOff();
    digitalWrite(LED_POWER, LOW);

    Serial.begin(9600);
    dht.begin();

    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Smart Temp Ctrl");
    lcd.setCursor(0, 1);
    lcd.print("MPC (host) Init");
    delay(2000);

    lastSampleTime = millis();
    windowStartTime = millis();
    lastCommandTime = millis(); // start the timeout clock now, not at 0
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
        hostCommand = 0;
        delay(250);
    }
    bool currentMode = digitalRead(BTN_MODE);
    if (lastModeButton == HIGH && currentMode == LOW) {
        autoMode = !autoMode;
        delay(250);
    }
    lastModeButton = currentMode;

    digitalWrite(LED_POWER, systemEnabled);

    //---------------- Sensors ----------------
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    int pot = analogRead(POT_PIN);
    setPoint = map(pot, 0, 1023, 200, 350) / 10.0;

    //---------------- Read any incoming command from host ----------------
    while (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        double cmd;
        if (parseHostCommand(line, cmd)) {
            hostCommand = cmd;
            lastCommandTime = now;
        }
    }

    //---------------- Send sensor data to host at the sample rate ----------------
    if (now - lastSampleTime >= Ts) {
        lastSampleTime = now;
        if (!isnan(temperature)) {
            Serial.print("T:");
            Serial.print(temperature, 2);
            Serial.print(",SP:");
            Serial.println(setPoint, 2);
        }
    }

    //---------------- Safety-gated actuation ----------------
    bool commandStale = (now - lastCommandTime) > COMMAND_TIMEOUT_MS;
    bool overTemp = (!isnan(temperature)) && (temperature >= ABSOLUTE_MAX_TEMP);

    if (systemEnabled && autoMode && !isnan(temperature) && !commandStale && !overTemp) {
        applyActuation(hostCommand, now);
    } else {
        allOff();
        if (commandStale) hostCommand = 0;
    }

    //---------------- LCD ----------------
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
        if (!systemEnabled) {
            lcd.print("SYSTEM OFF");
        } else if (overTemp) {
            lcd.print("OVERTEMP CUTOFF");
        } else if (commandStale) {
            lcd.print("HOST LINK LOST");
        } else if (!autoMode) {
            lcd.print("MANUAL/OFF");
        } else {
            lcd.print("MPC ");
            if (hostCommand > 1.0) {
                lcd.print("HEAT ");
                lcd.print((int)(100.0 * fabs(hostCommand) / outMax));
                lcd.print("%");
            } else if (hostCommand < -1.0) {
                lcd.print("COOL ");
                lcd.print((int)(100.0 * fabs(hostCommand) / outMax));
                lcd.print("%");
            } else {
                lcd.print("IDLE");
            }
        }
    }
}
