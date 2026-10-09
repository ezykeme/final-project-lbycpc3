#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_AIP31068_I2C.h>
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

// -------------------- Objects --------------------
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_AIP31068_I2C lcd(0x3E, 16, 2);

// -------------------- Variables --------------------
bool systemEnabled = false;
bool autoMode = true;

bool lastModeButton = HIGH;

float temperature = 0;
float humidity = 0;
float setPoint = 25.0;

const float hysteresis = 1.0;

// --------------------------------------------------

void heaterOn()
{
    digitalWrite(RELAY_HEAT, HIGH);
    digitalWrite(RELAY_FAN, LOW);

    digitalWrite(LED_HEAT, HIGH);
    digitalWrite(LED_COOL, LOW);
}

void fanOn()
{
    digitalWrite(RELAY_HEAT, LOW);
    digitalWrite(RELAY_FAN, HIGH);

    digitalWrite(LED_HEAT, LOW);
    digitalWrite(LED_COOL, HIGH);
}

void allOff()
{
    digitalWrite(RELAY_HEAT, LOW);
    digitalWrite(RELAY_FAN, LOW);

    digitalWrite(LED_HEAT, LOW);
    digitalWrite(LED_COOL, LOW);
}

void setup()
{
    pinMode(BTN_START, INPUT_PULLUP);
    pinMode(BTN_STOP, INPUT_PULLUP);
    pinMode(BTN_MODE, INPUT_PULLUP);

    pinMode(LED_POWER, OUTPUT);
    pinMode(LED_HEAT, OUTPUT);
    pinMode(LED_COOL, OUTPUT);

    pinMode(RELAY_HEAT, OUTPUT);
    pinMode(RELAY_FAN, OUTPUT);

    allOff();

    digitalWrite(LED_POWER, LOW);

    Serial.begin(9600);

    dht.begin();

    // Wait for the LCD controller to power up before initializing it.
    // The library's own power-on wait uses delayMicroseconds(50000), which
    // overflows AVR's 16-bit argument and only waits ~0.85ms instead of 40ms+.
    delay(1000);

    lcd.init();

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Smart Temp Ctrl");
    lcd.setCursor(0,1);
    lcd.print("Initializing");

    delay(2000);
}

void loop()
{
    //---------------- Buttons ----------------

    if(digitalRead(BTN_START)==LOW)
    {
        systemEnabled=true;
        delay(250);
    }

    if(digitalRead(BTN_STOP)==LOW)
    {
        systemEnabled=false;
        delay(250);
    }

    bool currentMode = digitalRead(BTN_MODE);

    if(lastModeButton==HIGH && currentMode==LOW)
    {
        autoMode=!autoMode;
        delay(250);
    }

    lastModeButton=currentMode;

    //---------------- Power LED ----------------

    digitalWrite(LED_POWER, systemEnabled);

    //---------------- Read Sensors ----------------

    humidity=dht.readHumidity();
    temperature=dht.readTemperature();

    //---------------- Potentiometer ----------------

    int pot=analogRead(POT_PIN);

    setPoint=map(pot,0,1023,200,350)/10.0;

    //---------------- Control ----------------

    if(systemEnabled && autoMode)
    {
        if(temperature < setPoint-hysteresis)
        {
            heaterOn();
        }
        else if(temperature > setPoint+hysteresis)
        {
            fanOn();
        }
        else
        {
            allOff();
        }
    }
    else
    {
        allOff();
    }

    //---------------- LCD ----------------

    lcd.clear();

    lcd.setCursor(0,0);

    lcd.print("T:");
    lcd.print(temperature,1);

    lcd.print((char)223);
    lcd.print("C ");

    lcd.print("S:");
    lcd.print(setPoint,1);

    lcd.setCursor(0,1);

    if(!systemEnabled)
    {
        lcd.print("SYSTEM OFF");
    }
    else
    {
        if(autoMode)
            lcd.print("AUTO ");
        else
            lcd.print("MAN ");

        if(digitalRead(RELAY_HEAT))
            lcd.print("HEATING");

        else if(digitalRead(RELAY_FAN))
            lcd.print("COOLING");

        else
            lcd.print("IDLE");
    }

    //---------------- Serial ----------------

    Serial.print("Temp: ");
    Serial.print(temperature);

    Serial.print("  Humidity: ");
    Serial.print(humidity);

    Serial.print("  SetPoint: ");
    Serial.print(setPoint);

    Serial.print("  Enabled: ");
    Serial.print(systemEnabled);

    Serial.print("  Mode: ");

    if(autoMode)
        Serial.println("AUTO");
    else
        Serial.println("MANUAL");

    delay(500);
}