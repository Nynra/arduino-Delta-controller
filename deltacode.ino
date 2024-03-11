/*
Arduino file for controlling the Delta Elektronics ES030-5 power supply using the analog pins on the PSU

This code uses the analog pins on the PSU to control the voltage and current. It is easiest to see the
PSU as an Operational Amplifier with a gain of 6 (maps 0 - 5 V to 0 - 30V) that can be current or voltage controlled.

ES030-5 pin mapping
1. ??
2. I monitor (0-5 V)
3. I programming (0-5 V)
4. CC status (0 = VC, 1 = CC)
5. Remote shutdown (0 = on, 1 = off)
6. NC
7. 12V output (can be used to power microcontrollers)
8. ??
9. Reference voltage (5.1 V)
10. V monitor (0-5 V)
11. V programming (0-5 V)
12. NC
13. NC
14. NC
15. NC

.. attention:
    All commands expect a carrige return ("\r") at the end of the command.

COMMAND SET
----------------
General command set:
- r (Turn the power supply on)
- s (Turn the power supply off)
- sm int (Set the mode, 0 = PSU mode, 1 = Function generator mode)
- gm (Get the mode, 0 = PSU mode, 1 = Function generator mode)

PSU mode command set:
- sv int (Set the voltage, in mV)
- sc int (Set the current, in mA)
- gv (Get the voltage, in mV)
- gc (Get the current, in mA)

Function generator mode command set:
- sf int (Set the frequency, in Hz)
- sa int (Set the amplitude, in mV)
- so int (Set the offset, in mV)
- sw int (Set the waveform, 0 = sine, 1 = square, 2 = triangle)
- gf (Get the frequency, in Hz)
- ga (Get the amplitude, in mV)
- go (Get the offset, in mV)
- gw (Get the waveform, 0 = sine, 1 = square, 2 = triangle)

ERROR CODES
----------------
- err: 0 (Unknown command)
- err: 1 (Value out of bounds)
- err: 2 (Current limit reached)
*/
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 voltage_dac;
Adafruit_MCP4725 current_dac;

// IMPORTANT RUNTIME CONFIGURATION
const bool CONFIRMATION = true;           // Set to true to enable confirmation messages
const String CONFIRMATION_MESSAGE = "OK"; // Message to send when a command is executed
const bool USE_CORRECTION_FACTOR = false; // Set to true to use the correction factor
const int BAUDRATE = 9600;

// Pin definitions (analog)
const int V_MON_PIN = 2; // Analog pin A2
const int I_MON_PIN = 3; // Analog pin A3
const int REF_PIN = 4;   // Analog pin A4

// Pin definitions (digital)
const int RSD_PIN = 2;     // Remote shutdown pin
const int CC_MODE_PIN = 3; // Constant current mode pin

/* PSU mode settings

Internally the code only uses voltage and current ranges between 0 and 5 V.
This is because this is also the range the PSU monitors and sets the values with.
When interacting with the user the values are converted to the correct range. This
means the user can still use the full range of the PSU (0 - 30 V, 0 - 5 A).
*/
const int VOLTAGE_MIN = 0;        // Minimum voltage in mV
const int VOLTAGE_MAX = 30000;    // Maximum voltage in mV
const int CURRENT_MIN = 0;        // Minimum current in mV
const int CURRENT_MAX = 5000;     // Maximum current in Mv
const int DAC_MAX = 19540 / 5000; // Value to give the dac and get 5V

/* Function generator mode settings

This PSU can also be used as a DIY function generator. This mode is not very accurate
but it good enough for some low frewuency applications. One of the main advantages of
this mode is that it can be used to generate waveforms with significanly more power than
a normal function generator.
*/
const int FREQ_MIN = 0;       // Minimum frequency
const int FREQ_MAX = 50;     // Maximum frequency
const int AMP_MIN = 0;        // Minimum amplitude
const int AMP_MAX = 30000;    // Maximum amplitude in mV
const int OFFSET_MIN = 0;     // Minimum offset in mV
const int OFFSET_MAX = 30000; // Maximum offset in mV
const int WAVEFORM_MIN = 0;   // Minimum waveform
const int WAVEFORM_MAX = 3;   // Maximum waveform

// RUNTIME VARIABLES
int MODE = 0;              // 0 = PSU mode, 1 = Function generator mode
int REFERENCE_VOLTAGE = 0; // Reference voltage in mV
int CORRECTION_FACTOR = 0; // Correction factor for the reference voltage
int MEASURED_VOLTAGE = 0;  // Measured voltage in mV
int LAST_VOLTAGE = 0;      // Voltage in mV
int VOLTAGE = 0;           // Voltage in mV
int MEASURED_CURRENT = 0;  // Measured current in mA
int LAST_CURRENT = 0;      // Current in mA
int CURRENT = 0;           // Current in mA
int LAST_WAVEFORM = 0;     // 0 = sine, 1 = square, 2 = triangle
int WAVEFORM = 0;          // 0 = sine, 1 = square, 2 = triangle
int LAST_FREQ = 0;         // Frequency in Hz
int FREQ = 10;             // Frequency in Hz
int LAST_AMP = 0;          // Amplitude in mV
int AMP = 1000;            // Amplitude in mV
int LAST_OFFSET = 0;       // Offset in mV
int OFFSET = 0;            // Offset in mV

void setup()
{
    // Set baudrate
    Serial.begin(BAUDRATE);

    // Set pin modes
    pinMode(RSD_PIN, OUTPUT);
    pinMode(CC_MODE_PIN, INPUT);

    // Set default values
    digitalWrite(RSD_PIN, 0); // Off by default

    voltage_dac.begin(0x60);
    current_dac.begin(0x61);

    if (USE_CORRECTION_FACTOR)
    {
        // Calculate the correction factor
        REFERENCE_VOLTAGE = analogRead(REF_PIN) / 1024 * 5;
        CORRECTION_FACTOR = REFERENCE_VOLTAGE / 5;
    }

    // Set the initial voltage and current to 0
    voltage_dac.setVoltage(0, false);
    current_dac.setVoltage(0, false);
    Serial.println("Successfully initiated the controller");
}

float map_input_voltage(int input)
{
    return map(input, VOLTAGE_MIN, VOLTAGE_MAX, 0, 4095);
}

float map_input_current(int input)
{
    return map(input, CURRENT_MIN, CURRENT_MAX, 0, 4095);
}

float map_output_voltage(int output)
{
    return map(output, 0, 4095, VOLTAGE_MIN, VOLTAGE_MAX);
}

float map_output_current(int output)
{
    return map(output, 0, 4095, CURRENT_MIN, CURRENT_MAX);
}

void read_commands()
{
    // Read the serial input
    if (Serial.available() > 0)
    {
        // Read the input
        String input = Serial.readStringUntil('\n');
        // Split the input into a command and a value
        String command = input.substring(0, input.indexOf(' '));
        String value = input.substring(input.indexOf(' ') + 1);

        // Execute the command
        if (command == "r")
        {
            digitalWrite(RSD_PIN, HIGH);
        }
        else if (command == "s")
        {
            digitalWrite(RSD_PIN, LOW);
        }
        else if (command == "sv")
        {
            // Remap the value form 0-5 to 0-30
            int new_voltage = value.toInt();
            if (new_voltage < VOLTAGE_MIN || new_voltage > VOLTAGE_MAX)
            {
                Serial.println("err: 1 Voltage out of bounds");
            }
            else
            {
                VOLTAGE = map_input_voltage(new_voltage);
            }
        }
        else if (command == "sc")
        {
            int new_current = value.toInt();
            if (new_current < CURRENT_MIN || new_current > CURRENT_MAX)
            {
                Serial.println("err: 1 Current out of bounds");
            }
            else
            {
                CURRENT = map_input_current(new_current);
            }
        }
        else if (command == "gv")
        {
            Serial.println(map_output_voltage(LAST_VOLTAGE));
        }
        else if (command == "gc")
        {
            Serial.println(map_output_current(LAST_CURRENT));
        }
        else if (command == "sm")
        {
            int new_mode = value.toInt();
            if (new_mode < 0 || new_mode > 1)
            {
                Serial.println("err: 1 Mode out of bounds");
            }
            else
            {
                MODE = new_mode;
            }
        }
        else if (command == "gm")
        {
            Serial.println(MODE);
        }
        else if (command == "sf")
        {
            int new_freq = value.toInt();
            if (new_freq < FREQ_MIN || new_freq > FREQ_MAX)
            {
                Serial.println("err: 1 Frequency out of bounds");
            }
            else
            {
                FREQ = new_freq;
            }
        }
        else if (command == "gf")
        {
            Serial.println(LAST_FREQ);
        }
        else if (command == "sa")
        {
            int new_amp = map_input_voltage(value.toInt());
            if (new_amp + OFFSET > 4095)
            {
                Serial.println("err: 1 Amplitude + Offset is greater than 30 V");
            }
            else
            {
                AMP = new_amp;
            }
        }
        else if (command == "ga")
        {
            Serial.println(map_output_voltage(LAST_AMP));
        }
        else if (command == "so")
        {
            int new_offset = map_input_voltage(value.toInt());
            if (new_offset + AMP > 4095)
            {
                Serial.println("err: 1 Amplitude + Offset is greater than 30 V");
            }
            else
            {
                OFFSET = new_offset;
            }
        }
        else if (command == "go")
        {
            Serial.println(map_output_voltage(LAST_OFFSET));
        }
        else if (command == "sw")
        {
            int waveform = value.toInt();
            if (waveform < WAVEFORM_MIN || waveform > WAVEFORM_MAX)
            {
                Serial.println("err: 1 Waveform out of bounds");
            }
            else
            {
                WAVEFORM = waveform;
            }
        }
        else if (command == "gw")
        {
            Serial.println(WAVEFORM);
        }
        else
        {
            Serial.println("err: Unknown command " + command + "!");
        }

        if (CONFIRMATION)
        {
            Serial.println("" + command + " " + value + " " + CONFIRMATION_MESSAGE);
        }
    }
}

void update_monitor_readings()
{
    // Measure the current and voltage
    MEASURED_VOLTAGE = analogRead(V_MON_PIN) / 1024 * 5;
    MEASURED_CURRENT = analogRead(I_MON_PIN) / 1024 * 5;
}

void execute_psu_mode()
{
    update_monitor_readings();

    // Check if the voltage changed
    if (VOLTAGE != LAST_VOLTAGE)
    {
        // Calculate the needed output
        int output = VOLTAGE; 
        if (USE_CORRECTION_FACTOR)
        {
            output = output * CORRECTION_FACTOR;
        }
        voltage_dac.setVoltage(output, false);
        LAST_VOLTAGE = VOLTAGE;
    }
    // Check if the current changed
    if (CURRENT != LAST_CURRENT)
    {
        // Calculate the needed output
        int output = CURRENT;
        if (USE_CORRECTION_FACTOR)
        {
            output = output * CORRECTION_FACTOR;
        }
        current_dac.setVoltage(output, false);
        LAST_CURRENT = CURRENT;
    }
}

float cal_sine(int freq, int amplitude, int offset)
{
    float output = sin(2 * PI * millis() * 1e-3 * freq) * amplitude + offset;
    return output;
}

float cal_square(int freq, int amplitude, int offset)
{
    float output = 0;
    if (sin(2 * PI * millis() * 1e-3 * freq) > 0)
    {
        output = amplitude + offset;
    }
    else
    {
        output = offset;
    }
    return output;
}

float cal_triangle(int freq, int amplitude, int offset)
{
    float output = 0;
    output = (2 * amplitude / PI) * asin(sin(2 * PI * millis() * 1e-3 * freq)) + offset;
    return output;
}

float cal_sawtooth(int freq, int amplitude, int offset)
{
    float output = 0;
    output = (2 * amplitude / PI) * atan(tan(2 * PI * millis() * 1e-3 * freq)) + offset;
    return output;
}

void execute_function_generator_mode()
{
    update_monitor_readings();

    // Update the offset, frequency and amplitude
    if (LAST_FREQ != FREQ || LAST_AMP != AMP || LAST_OFFSET != OFFSET || LAST_WAVEFORM != WAVEFORM)
    {
        LAST_FREQ = FREQ;
        LAST_AMP = AMP;
        LAST_OFFSET = OFFSET;
        LAST_WAVEFORM = WAVEFORM;
    } 

    // Calculate the output
    int output = 0;
    switch (WAVEFORM)
    {
    case 0:
        output = cal_sine(FREQ, AMP, OFFSET);
//        Serial.println();
        break;
    case 1:
        output = cal_square(FREQ, AMP, OFFSET);
        break;
    case 2:
        output = cal_triangle(FREQ, AMP, OFFSET);
        break;
    case 3:
        output = cal_sawtooth(FREQ, AMP, OFFSET);
        break;
    }
    if (USE_CORRECTION_FACTOR)
    {
        output = output * CORRECTION_FACTOR;
    }
    // Set the output
    voltage_dac.setVoltage(output, false);

    // Check if the output is current limited
    if (digitalRead(CC_MODE_PIN) == 1)
    {
        Serial.println("err: 2 Current limit reached");
    }
}

void loop()
{
    read_commands();
    if (MODE == 0)
    {
        execute_psu_mode();
    }
    else if (MODE == 1)
    {
        execute_function_generator_mode();
    }
    else
    {
        Serial.println("err: 0 Unknown mode");
    }
}
