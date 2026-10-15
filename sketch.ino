#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ============================================================
// OLED
// ============================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C

Adafruit_SSD1306 display(
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    &Wire,
    -1
);

// ============================================================
// PIN DEFINITIONS
// ============================================================

// Analog inputs
#define TEMP_PIN        PB0
#define BATTERY_PIN     PB1
#define RPM_PIN         PA0

// Outputs
#define FAN_PIN         PA1

// Buttons / fault inputs
#define IGNITION_PIN    PB12
#define OVER_TEMP_PIN   PB13
#define LOW_BAT_PIN     PB14
#define OVERSPEED_PIN   PB15
#define SENSOR_PIN      PA8

// LEDs
#define NORMAL_LED      PC13
#define WARNING_LED     PA11
#define FAULT_LED       PA12

// ============================================================
// ECU STATES
// ============================================================

enum ECUState
{
    ECU_OFF,
    ECU_SELF_CHECK,
    ECU_READY,
    ECU_RUNNING,
    ECU_FAULT
};

ECUState ecuState = ECU_OFF;

// ============================================================
// SENSOR VARIABLES
// ============================================================

int tempADC = 0;
int batteryADC = 0;

float temperature = 0.0;
float batteryVoltage = 0.0;
float rpm = 0.0;

int fanDuty = 0;

// ============================================================
// FAULT FLAGS
// ============================================================

bool overTemperatureFault = false;
bool lowBatteryFault = false;
bool overspeedFault = false;
bool sensorFault = false;

// ============================================================
// RPM VARIABLES
// ============================================================

volatile unsigned long lastRPMTime = 0;
volatile unsigned long rpmPeriod = 0;


// ============================================================
// TIMERS
// ============================================================

unsigned long lastSensorRead = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastSerialUpdate = 0;

const unsigned long SENSOR_INTERVAL = 200;
const unsigned long DISPLAY_INTERVAL = 500;
const unsigned long SERIAL_INTERVAL = 500;

// ============================================================
// RPM INTERRUPT
// ============================================================

void RPM_ISR()
{
    unsigned long currentTime = micros();

    if (lastRPMTime != 0)
    {
        unsigned long period = currentTime - lastRPMTime;

        // Ignore unrealistically short pulses
        if (period >= 1000)
        {
            rpmPeriod = period;
        }
    }

    lastRPMTime = currentTime;
}

// ============================================================
// ECU STATE NAME
// ============================================================

const char* GetStateName()
{
    switch (ecuState)
    {
        case ECU_OFF:
            return "OFF";

        case ECU_SELF_CHECK:
            return "SELF CHECK";

        case ECU_READY:
            return "READY";

        case ECU_RUNNING:
            return "RUNNING";

        case ECU_FAULT:
            return "FAULT";

        default:
            return "UNKNOWN";
    }
}

// ============================================================
// READ SENSORS
// ============================================================

void ReadSensors()
{
    tempADC = analogRead(TEMP_PIN);
    batteryADC = analogRead(BATTERY_PIN);

    // -------------------------------
    // RPM
    // -------------------------------

    unsigned long period;
    unsigned long lastPulse;

    noInterrupts();

    period = rpmPeriod;
    lastPulse = lastRPMTime;

    interrupts();

    unsigned long now = micros();

    // If no pulse has arrived for 1 second,
    // consider engine stopped.
    if (lastPulse == 0 ||
        (unsigned long)(now - lastPulse) > 1000000UL)
    {
        rpm = 0.0;
    }
    else if (period > 0)
    {
        rpm = 60000000.0 / (float)period;
    }
    else
    {
        rpm = 0.0;
    }

    // -------------------------------
    // Other sensors
    // -------------------------------

    CalculateTemperature();
    CalculateBattery();
}

// ============================================================
// CALCULATE TEMPERATURE
// Wokwi NTC 3950
// ============================================================

void CalculateTemperature()
{
    if (tempADC <= 0 || tempADC >= 4095)
    {
        sensorFault = true;
        return;
    }

    float resistanceRatio =
        4095.0 / (float)tempADC - 1.0;

    if (resistanceRatio <= 0)
    {
        sensorFault = true;
        return;
    }

    temperature =
        1.0 /
        (
            log(1.0 / resistanceRatio) / 3950.0
            +
            1.0 / 298.15
        )
        - 273.15;

    if (temperature < -40.0 ||
        temperature > 150.0)
    {
        sensorFault = true;
    }
    else
    {
        sensorFault = false;
    }
}

// ============================================================
// CALCULATE BATTERY VOLTAGE
// Potentiometer represents 0 - 14.2 V
// ============================================================

void CalculateBattery()
{
    float adcVoltage =
        ((float)batteryADC * 3.3) / 4095.0;

    batteryVoltage =
        adcVoltage * (14.2 / 3.3);
}

// ============================================================
// CHECK FAULTS
// ============================================================

void CheckFaults()
{
    // Automatic faults
    overTemperatureFault =
        (temperature >= 100.0);

    lowBatteryFault =
        (batteryVoltage <= 2.5);

    overspeedFault =
        (rpm >= 3000.0);

    // Manual fault buttons
    // Buttons are active LOW

    if (digitalRead(OVER_TEMP_PIN) == LOW)
    {
        overTemperatureFault = true;
    }

    if (digitalRead(LOW_BAT_PIN) == LOW)
    {
        lowBatteryFault = true;
    }

    if (digitalRead(OVERSPEED_PIN) == LOW)
    {
        overspeedFault = true;
    }

    if (digitalRead(SENSOR_PIN) == LOW)
    {
        sensorFault = true;
    }
}

// ============================================================
// CHECK IF ANY FAULT EXISTS
// ============================================================

bool AnyFault()
{
    return
        overTemperatureFault ||
        lowBatteryFault ||
        overspeedFault ||
        sensorFault;
}

// ============================================================
// ECU STATE MACHINE
// ============================================================

void UpdateECUState()
{
    bool ignition =
        (digitalRead(IGNITION_PIN) == LOW);

    switch (ecuState)
    {
        // ----------------------------------------------------
        // OFF
        // ----------------------------------------------------

        case ECU_OFF:

            if (ignition)
            {
                ecuState = ECU_SELF_CHECK;
            }

            break;


        // ----------------------------------------------------
        // SELF CHECK
        // ----------------------------------------------------

        case ECU_SELF_CHECK:

            if (!ignition)
            {
                ecuState = ECU_OFF;
            }
            else if (AnyFault())
            {
                ecuState = ECU_FAULT;
            }
            else
            {
                ecuState = ECU_READY;
            }

            break;


        // ----------------------------------------------------
        // READY
        // ----------------------------------------------------

        case ECU_READY:

            if (!ignition)
            {
                ecuState = ECU_OFF;
            }
            else if (AnyFault())
            {
                ecuState = ECU_FAULT;
            }
            else if (rpm > 100.0)
            {
                ecuState = ECU_RUNNING;
            }

            break;


        // ----------------------------------------------------
        // RUNNING
        // ----------------------------------------------------

        case ECU_RUNNING:

            if (!ignition)
            {
                ecuState = ECU_OFF;
            }
            else if (AnyFault())
            {
                ecuState = ECU_FAULT;
            }

            break;


        // ----------------------------------------------------
        // FAULT
        // ----------------------------------------------------

        case ECU_FAULT:

            if (!ignition)
            {
                ecuState = ECU_OFF;
            }
            else if (!AnyFault())
            {
                ecuState = ECU_READY;
            }

            break;
    }
}

// ============================================================
// FAN CONTROL
// ============================================================

void UpdateFan()
{
    // ECU OFF
    if (ecuState == ECU_OFF)
    {
        fanDuty = 0;
    }

    // Critical faults
    else if (overTemperatureFault ||
             sensorFault)
    {
        fanDuty = 100;
    }

    // Temperature based control
    else if (temperature >= 90.0)
    {
        fanDuty = 100;
    }

    else if (temperature >= 80.0)
    {
        fanDuty = 60;
    }

    else if (temperature >= 70.0)
    {
        fanDuty = 30;
    }

    else
    {
        fanDuty = 0;
    }

    // Convert percentage to PWM
    int pwmValue =
        map(fanDuty, 0, 100, 0, 255);

    analogWrite(FAN_PIN, pwmValue);
}

// ============================================================
// LED CONTROL
//
// LEDs are assumed to be wired:
//
// 3.3V -> LED -> resistor -> GPIO
//
// Therefore LEDs are ACTIVE LOW.
// ============================================================

void UpdateLEDs()
{
    // Turn everything OFF
    digitalWrite(NORMAL_LED, HIGH);
    digitalWrite(WARNING_LED, HIGH);
    digitalWrite(FAULT_LED, HIGH);

    switch (ecuState)
    {
        // ----------------------------------------------------
        // OFF
        // ----------------------------------------------------

        case ECU_OFF:
            break;


        // ----------------------------------------------------
        // SELF CHECK
        // ----------------------------------------------------

        case ECU_SELF_CHECK:

            digitalWrite(WARNING_LED, LOW);

            break;


        // ----------------------------------------------------
        // READY
        // ----------------------------------------------------

        case ECU_READY:

            digitalWrite(NORMAL_LED, LOW);

            break;


        // ----------------------------------------------------
        // RUNNING
        // ----------------------------------------------------

        case ECU_RUNNING:

            digitalWrite(NORMAL_LED, LOW);

            if (temperature >= 80.0)
            {
                digitalWrite(WARNING_LED, LOW);
            }

            break;


        // ----------------------------------------------------
        // FAULT
        // ----------------------------------------------------

        case ECU_FAULT:

            digitalWrite(FAULT_LED, LOW);

            break;
    }
}

// ============================================================
// OLED DISPLAY
// ============================================================

void UpdateOLED()
{
    display.clearDisplay();

    display.setTextColor(SSD1306_WHITE);

    // --------------------------------------------------------
    // Line 1 - ECU STATE
    // --------------------------------------------------------

    display.setTextSize(1);
    display.setCursor(0, 0);

    display.print("ECU: ");
    display.println(GetStateName());

    // Separator
    display.drawLine(
        0, 10,
        127, 10,
        SSD1306_WHITE
    );

    // --------------------------------------------------------
    // RPM
    // --------------------------------------------------------

    display.setCursor(0, 15);

    display.print("RPM : ");
    display.println((int)rpm);

    // --------------------------------------------------------
    // Temperature
    // --------------------------------------------------------

    display.setCursor(0, 27);

    display.print("TEMP: ");
    display.print(temperature, 1);
    display.println(" C");

    // --------------------------------------------------------
    // Battery
    // --------------------------------------------------------

    display.setCursor(0, 39);

    display.print("BATT: ");
    display.print(batteryVoltage, 1);
    display.println(" V");

    // --------------------------------------------------------
    // Fan
    // --------------------------------------------------------

    display.setCursor(0, 51);

    display.print("FAN : ");
    display.print(fanDuty);
    display.println("%");

    display.display();
}

// ============================================================
// SERIAL DIAGNOSTICS
// ============================================================

void PrintSerialData()
{
    Serial1.println();
    Serial1.println("==============================");

    Serial1.print("ECU STATE : ");
    Serial1.println(GetStateName());

    Serial1.print("RPM       : ");
    Serial1.println(rpm, 1);

    Serial1.print("TEMP      : ");
    Serial1.print(temperature, 1);
    Serial1.println(" C");

    Serial1.print("BATTERY   : ");
    Serial1.print(batteryVoltage, 2);
    Serial1.println(" V");

    Serial1.print("FAN       : ");
    Serial1.print(fanDuty);
    Serial1.println("%");

    Serial1.println();

    Serial1.print("FAULTS    : ");

    if (!AnyFault())
    {
        Serial1.println("NONE");
    }
    else
    {
        if (overTemperatureFault)
            Serial1.print("OVER_TEMP ");

        if (lowBatteryFault)
            Serial1.print("LOW_BAT ");

        if (overspeedFault)
            Serial1.print("OVERSPEED ");

        if (sensorFault)
            Serial1.print("SENSOR ");

        Serial1.println();
    }

    // Button diagnostics
    Serial1.print("BUTTONS   : ");

    Serial1.print(
        digitalRead(OVER_TEMP_PIN)
    );

    Serial1.print(",");

    Serial1.print(
        digitalRead(LOW_BAT_PIN)
    );

    Serial1.print(",");

    Serial1.print(
        digitalRead(OVERSPEED_PIN)
    );

    Serial1.print(",");

    Serial1.println(
        digitalRead(SENSOR_PIN)
    );

    Serial1.println("==============================");
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    // --------------------------------------------------------
    // Serial
    // --------------------------------------------------------

    Serial1.begin(115200);

    delay(500);

    Serial1.println();
    Serial1.println("================================");
    Serial1.println(" STM32 MINI AUTOMOTIVE ECU");
    Serial1.println("================================");


    // --------------------------------------------------------
    // Pin modes
    // --------------------------------------------------------

    pinMode(TEMP_PIN, INPUT_ANALOG);
    pinMode(BATTERY_PIN, INPUT_ANALOG);

    pinMode(RPM_PIN, INPUT);

    pinMode(FAN_PIN, OUTPUT);

    pinMode(
        IGNITION_PIN,
        INPUT_PULLUP
    );

    pinMode(
        OVER_TEMP_PIN,
        INPUT_PULLUP
    );

    pinMode(
        LOW_BAT_PIN,
        INPUT_PULLUP
    );

    pinMode(
        OVERSPEED_PIN,
        INPUT_PULLUP
    );

    pinMode(
        SENSOR_PIN,
        INPUT_PULLUP
    );

    pinMode(NORMAL_LED, OUTPUT);
    pinMode(WARNING_LED, OUTPUT);
    pinMode(FAULT_LED, OUTPUT);


    // --------------------------------------------------------
    // Initial outputs
    // --------------------------------------------------------

    digitalWrite(NORMAL_LED, HIGH);
    digitalWrite(WARNING_LED, HIGH);
    digitalWrite(FAULT_LED, HIGH);

    analogWrite(FAN_PIN, 0);


    // --------------------------------------------------------
    // RPM interrupt
    // --------------------------------------------------------

    attachInterrupt(
        digitalPinToInterrupt(RPM_PIN),
        RPM_ISR,
        RISING
    );


    // --------------------------------------------------------
    // I2C
    // --------------------------------------------------------

    Wire.setSDA(PB7);
    Wire.setSCL(PB6);
    Wire.begin();


    // --------------------------------------------------------
    // OLED
    // --------------------------------------------------------

    Serial1.println("Initializing OLED...");

    if (!display.begin(
            SSD1306_SWITCHCAPVCC,
            OLED_ADDRESS))
    {
        Serial1.println("OLED FAILED!");

        while (1)
        {
            // Stop here if OLED fails
        }
    }

    Serial1.println("OLED OK!");


    // --------------------------------------------------------
    // OLED startup screen
    // --------------------------------------------------------

    display.clearDisplay();

    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(2);

    display.setCursor(5, 5);
    display.println("ECU");

    display.setTextSize(1);

    display.setCursor(5, 32);
    display.println("Initializing...");

    display.setCursor(5, 48);
    display.println("STM32F103C8T6");

    display.display();

    delay(1500);


    // --------------------------------------------------------
    // Initial sensor reading
    // --------------------------------------------------------

    ReadSensors();

    CheckFaults();

    UpdateECUState();

    UpdateFan();

    UpdateLEDs();

    UpdateOLED();


    Serial1.println("ECU INITIALIZED");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    unsigned long currentTime = millis();


    // ========================================================
    // SENSOR UPDATE
    // ========================================================

    if (currentTime - lastSensorRead >= SENSOR_INTERVAL)
    {
        lastSensorRead = currentTime;

        ReadSensors();

        CheckFaults();

        UpdateECUState();

        UpdateFan();

        UpdateLEDs();
    }


    // ========================================================
    // OLED UPDATE
    // ========================================================

    if (currentTime - lastDisplayUpdate >= DISPLAY_INTERVAL)
    {
        lastDisplayUpdate = currentTime;

        UpdateOLED();
    }


    // ========================================================
    // SERIAL UPDATE
    // ========================================================

    if (currentTime - lastSerialUpdate >= SERIAL_INTERVAL)
    {
        lastSerialUpdate = currentTime;

        PrintSerialData();
    }
}