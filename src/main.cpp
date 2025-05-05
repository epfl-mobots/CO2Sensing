/* Cyril Monette, September 2024, modified by Romain Lattion for 6 SCD30 and ENV-Pro, May 2025
This code is for the M5StickCPlus to read data from 6 SCD30 sensors at Bassenges and an ENV-Pro unit,
display it on the LCD, and send it to the RPi for logging.

  Recognized commands from serial:
    - "Init"          : Resets the serial buffer and initializes sensor communication
    - "Get data"      : Prints the latest CO2, temperature, and humidity measurements for all sensors in the format [SCD30, sensor_nb, {CO2, temp, RH}]
    - "CalibrateAll"  : Calibrates all SCD30 sensors to 400 ppm CO2
    - "Calibrate sensor x"  : Calibrates SCD30 sensor x to 400 ppm CO2

  M5StickCPlus button commands:
    - M5 button (A) press            : Navigate through sensors 1–6 (highlight selected one)
    - Right button (B) press         : Toggle between main display (all sensor states) and zoomed display (selected sensor)
    - M5 button hold for 2 sec       : Cycle through display modes in main view (CO2, Temperature, Humidity, Weather Station)
    - Right button hold for 3.5 sec  : In main mode, calibrate all SCD30 sensors; in zoomed mode, calibrate the selected SCD30 sensor

  Sensor Mapping:
    - 6 SCD30 sensors connected to TCA9548A I2C multiplexer:
        • Sensor 1: Channel 0
        • Sensor 2: Channel 1
        • Sensor 3: Channel 2
        • Sensor 4: Channel 3
        • Sensor 5: Channel 4
        • Sensor 6: Channel 5
    - ENV-Pro (BME688) connected directly to I2C bus (address 0x77)

  Display:
    - Main mode: Shows states of all sensors (CO2, Temp, Humidity, or Weather Station based on display mode), with the selected sensor highlighted in yellow. Includes button instructions at the bottom:
        • "M5: Navigate"
        • "Right: Select"
        • "Hold M5 (2s): Cycle through states"
        • "Hold Right (5s): Calib. all sensors"
    - Zoomed mode: Shows CO2, temperature, and humidity for the selected SCD30 sensor, with button instructions:
        • "M5: Navigate"
        • "Right: Exit"
        • "Hold M5 (2s): Cycle through states"
        • "Hold Right (5s): Calib. sel. sensor"
    - Weather Station mode: Shows Pressure, Temperature, Humidity, and CO2 Equivalent from the ENV-Pro unit.

  Notes:
    - Loop runs every 20ms to check buttons for responsiveness
    - SCD30 sensors are polled every SENSING_INTERVAL seconds (one sensor every ~SENSING_INTERVAL*1000/NUM_SENSORS ms)
    - ENV-Pro data is updated continuously via BSEC2 library
    - Display is updated every ACTUATING_INTERVAL seconds after all SCD30 sensors are polled
    - Serial output is printed every SERIAL_INTERVAL seconds
*/

#include "M5StickCPlus.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "bme68xLibrary.h"
#include <bsec2.h>

#define NUM_SENSORS 6 // Number of SCD30 sensors connected to the TCA9548A
#define SENSING_INTERVAL 8 // Seconds
#define ACTUATING_INTERVAL 8 // Seconds
#define SERIAL_INTERVAL 10 // Seconds

#define ADDRESS 0x71 // I2C address of the TCA9548A
#define BAUD_RATE 115200

#define DEBUG
const static bool display = true;

static SCD30 sensors[NUM_SENSORS]; // Array of SCD30 sensors
static Bsec2 envSensor; // ENV-Pro sensor (BME688-based)

// Variables for SCD30 sensors
static uint16_t co2_meas[NUM_SENSORS]={0}; // Array to store the last measurements of the SCD30 sensors
static float rh_meas[NUM_SENSORS]={0};
static float temp_meas[NUM_SENSORS]={0};

// Variables for Weather Station (ENV-Pro) sensing
static float env_pressure = 0.0; // hPa
static float env_humidity = 0.0; // %
static float env_temperature = 0.0; // °C
static float env_co2_equivalent = 0.0; // ppm

// LCD dimensions: 135x240 pixels
int rectWidth = 120;
int rectHeight = 60;
int rectX = (135 - rectWidth) / 2;
int rectY = 7;

// Button and display state
static uint8_t selectedSensor = 1; // Current sensor (1 to 6)
static bool isZoomed = false; // Whether zoomed display is active
static uint8_t displayMode = 0; // 0: CO2, 1: Temperature, 2: Humidity, 3: Weather Station

void clearSerialBuffer() {
    while (Serial.available() > 0) {
        Serial.read();
    }
    Serial.flush();
}

// Display STARTING screen with loading animation
void displayStartingScreen() {
    if (!display) return;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.drawRect(rectX, rectY, rectWidth, rectHeight, WHITE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);

    int textX = rectX + (rectWidth - 7 * 12) / 2;
    int textY1 = rectY + (rectHeight - 2 * 16) / 2;
    int textY2 = textY1 + 16;

    M5.Lcd.setCursor(textX - 4, textY1);
    M5.Lcd.print("STARTING");
    M5.Lcd.setCursor(textX + 3, textY2);
    delay(500);
    M5.Lcd.print("  .");
    delay(500);
    M5.Lcd.print(".");
    delay(500);
    M5.Lcd.print(".");
    delay(500);
}

// Display initialization status for a sensor
void displayInitStatus(uint8_t sensor_nb, bool success) {
    if (!display) return;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.drawRect(rectX, rectY, rectWidth, rectHeight, WHITE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);

    int textX = rectX + (rectWidth - 7 * 12) / 2;
    int textY1 = rectY + (rectHeight - 2 * 16) / 2;
    int textY2 = textY1 + 16;

    M5.Lcd.setCursor(textX - 4, textY1);
    M5.Lcd.print("SENSOR ");
    M5.Lcd.print(sensor_nb + 1);
    M5.Lcd.setCursor(textX - 4, textY2);
    M5.Lcd.setTextColor(success ? GREEN : RED);
    M5.Lcd.print(success ? "   OK" : "   FAIL");
    delay(1000);
}

// Display calibration status for a sensor
void displayCalibrationStatus(uint8_t sensor_nb, bool success) {
    if (!display) return;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.drawRect(rectX, rectY, rectWidth, rectHeight, WHITE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);

    int textX = rectX + (rectWidth - 7 * 12) / 2;
    int textY1 = rectY + (rectHeight - 2 * 16) / 2;
    int textY2 = textY1 + 16;

    M5.Lcd.setCursor(textX - 4, textY1);
    M5.Lcd.print("SENSOR ");
    M5.Lcd.print(sensor_nb + 1);
    M5.Lcd.setCursor(textX - 4, textY2);
    M5.Lcd.setTextColor(success ? GREEN : RED);
    M5.Lcd.print(success ? "   OK" : "   FAIL");
    delay(1000);
}

// Display zoomed view of selected sensor
void displayZoomedSensor() {
    if (!display) return;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.drawRect(rectX, rectY, rectWidth, rectHeight/2+rectHeight/5, WHITE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);

    int textX = rectX + (rectWidth - 7 * 12) / 2;
    int textY1 = rectY + (rectHeight - 2 * 16) / 2;
    int textY2 = textY1 + 16;

    M5.Lcd.setTextColor(YELLOW);
    M5.Lcd.setCursor(textX - 4, textY1);
    M5.Lcd.print("SENSOR ");
    M5.Lcd.println(selectedSensor);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\n\n\n ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("CO2:");
    M5.Lcd.print(co2_meas[selectedSensor - 1]);
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\n                  ppm");
    M5.Lcd.print("\n\n ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("Temp:");
    M5.Lcd.print(temp_meas[selectedSensor - 1], 1);
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\n                    C");
    M5.Lcd.print("\n\n ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("RH:");
    M5.Lcd.print(rh_meas[selectedSensor - 1], 1);
    M5.Lcd.setTextSize(1);
    M5.Lcd.println("\n                  %\n");

    M5.Lcd.print("\n");
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("  BUTTONS");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\n\n\n    M5");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":Navigate");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("    Right");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":Exit");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("    Hold M5 (2s)");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":\n Cycle through states");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("    Hold Right (3.5s)");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":\n Calib. sel. sensor");
}

// Display all sensor measurements for the current display mode (SCD30 sensors)
void displaySensorStates() {
    if (!display) return;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.drawRect(rectX, rectY, rectWidth, rectHeight, WHITE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);

    int textX = rectX + (rectWidth - 6 * 12) / 2 - 4;
    int textY1 = rectY + (rectHeight - 2 * 16) / 2;
    int textY2 = textY1 + 16;

    M5.Lcd.setCursor(textX-7, textY1);
    switch (displayMode) {
        case 0: M5.Lcd.print("  CO2"); break;
        case 1: M5.Lcd.print(" TEMP."); break;
        case 2: M5.Lcd.print("HUMIDITY"); break;
        default: break; // Weather Station mode handled separately
    }
    M5.Lcd.setCursor(textX + 3, textY2);
    M5.Lcd.print("STATES\n\n");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\n");

    for (int i = 0; i < NUM_SENSORS; i++) {
        M5.Lcd.setTextColor(i + 1 == selectedSensor ? YELLOW : WHITE);
        M5.Lcd.print("   Sensor ");
        M5.Lcd.print(i + 1);
        M5.Lcd.print(": ");
        switch (displayMode) {
            case 0:
                M5.Lcd.print(co2_meas[i]);
                M5.Lcd.println(" ppm");
                break;
            case 1:
                M5.Lcd.print(temp_meas[i], 1);
                M5.Lcd.println(" C");
                break;
            case 2:
                M5.Lcd.print(rh_meas[i], 1);
                M5.Lcd.println(" %");
                break;
        }
    }
    M5.Lcd.print("\n");
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("  BUTTONS");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\n\n\n    M5");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":Navigate");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("    Right");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":Select");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("    Hold M5 (2s)");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":\n Cycle through states");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("    Hold Right (3.5s)");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":\n Calib. all sensors");
}

// Display Weather Station data (ENV-Pro) with CO2 equivalent
void displayWeatherStation() {
    if (!display) return;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.drawRect(rectX, rectY, rectWidth, rectHeight/2 + rectHeight/5, WHITE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(YELLOW);

    int textX = rectX + (rectWidth - 7 * 12) / 2;
    int textY1 = rectY + (rectHeight - 2 * 16) / 2;

    M5.Lcd.setCursor(textX - 4, textY1);
    M5.Lcd.print("WEATHER DATA");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\n\n\n ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("Press:");
    M5.Lcd.print(env_pressure, 1);
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\n                  hPa");
    M5.Lcd.print("\n\n ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("Temp:");
    M5.Lcd.print(env_temperature, 1);
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\n                    C");
    M5.Lcd.print("\n\n ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("RH:");
    M5.Lcd.print(env_humidity, 1);
    M5.Lcd.setTextSize(1);
    M5.Lcd.println("\n                  %\n");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("CO2 eq:");
    M5.Lcd.print(env_co2_equivalent, 1);
    M5.Lcd.setTextSize(1);
    M5.Lcd.println("\n                  ppm\n");

    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("  BUTTONS");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\n\n\n    M5");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":Navigate");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("    Right");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":Select");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("    Hold M5 (2s)");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":\n Cycle through states");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("    Hold Right (3.5s)");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":\n Calib. all sensors");
}

// Function to select the TCA (I2C hub) channel
void tcaselect(uint8_t i) {
    if (i > NUM_SENSORS-1) {
        char error_msg[50];
        sprintf(error_msg, "Error: Channel %d is higher than SENSORS_NB", i);
        #ifdef DEBUG
        Serial.println(error_msg);
        #endif
        delay(2000);
        return;
    }

    Wire.beginTransmission(ADDRESS);
    Wire.write(1 << i); // Select channel i
    Wire.endTransmission();
}

// Function to parametrise the SCD30 sensors
static void parametriseSensors(int number) {
    sensors[number].setMeasurementInterval(SENSING_INTERVAL);
    delay(200);
    int interval = sensors[number].getMeasurementInterval();
    #ifdef DEBUG
    char msg[50];
    sprintf(msg, "Measurement Interval: %d", interval);
    Serial.println(msg);
    #endif
    sensors[number].setAltitudeCompensation(400);
    delay(200);
}

// Initialize an SCD30 sensor
static void init_sensor(uint8_t sensor_nb) {
    sensors[sensor_nb] = SCD30();
    tcaselect(sensor_nb);
    bool success = sensors[sensor_nb].begin(Wire);
    displayInitStatus(sensor_nb, success);
    while (!success) {
        #ifdef DEBUG
        char error_msg[50];
        sprintf(error_msg, "Error: Could not connect to sensor %d", sensor_nb);
        Serial.println(error_msg);
        #endif
        delay(500);
        tcaselect(sensor_nb);
        success = sensors[sensor_nb].begin(Wire);
        displayInitStatus(sensor_nb, success);
    }
    #ifdef DEBUG
    char success_msg[50];
    sprintf(success_msg, "Connected to sensor %d", sensor_nb);
    Serial.println(success_msg);
    #endif
    parametriseSensors(sensor_nb);
}

// Function to calibrate an SCD30 sensor
static void calibrate_sensor(uint8_t sensor_nb) {
    tcaselect(sensor_nb);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.drawRect(rectX, rectY, rectWidth, rectHeight, WHITE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);
    int textX = rectX + (rectWidth - 7 * 12) / 2;
    int textY1 = rectY + (rectHeight - 2 * 16) / 2;
    M5.Lcd.setCursor(textX - 4, textY1);
    M5.Lcd.print("SENSOR ");
    M5.Lcd.print(sensor_nb + 1);
    M5.Lcd.setCursor(textX - 7, textY1 + 16);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(ORANGE);
    M5.Lcd.print("CALIBRAT.");
    delay(500);

    // Attempt calibration
    sensors[sensor_nb].setForcedRecalibrationFactor(400);
    delay(200); // Allow time for the command to process

    // Verify by reading back CO2 to check if calibration took effect
    bool success = false;
    if (sensors[sensor_nb].dataAvailable()) {
        uint16_t co2 = sensors[sensor_nb].getCO2();
        // Check if CO2 reading is reasonable after calibration
        success = (co2 > 0); // Basic check; adjust if needed
    }

    displayCalibrationStatus(sensor_nb, success);
    #ifdef DEBUG
    if (success) {
        char success_msg[50];
        sprintf(success_msg, "Calibrated sensor %d to 400 ppm", sensor_nb + 1);
        Serial.println(success_msg);
    } else {
        char error_msg[50];
        sprintf(error_msg, "Error: Calibration failed for sensor %d", sensor_nb + 1);
        Serial.println(error_msg);
    }
    #endif
}

// Function to scan all I2C addresses and print the addresses of the devices found
static void scanI2CAddresses() {
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println(" !");
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }

    if (nDevices == 0) {
        Serial.println("No I2C devices found\n");
    } else {
        Serial.println("Done\n");
    }
}

// Function to format the latest SCD30 measurements to a log
String format_log(uint8_t sensor_nb) {
    String output = "[SCD30, ";
    output = output + String(sensor_nb + 1) + ", ";
    output = output + "{";
    output = output + String(co2_meas[sensor_nb]) + ", ";
    output = output + String(temp_meas[sensor_nb]) + ", ";
    output = output + String(rh_meas[sensor_nb]) + "}";
    output = output + "]";
    return output;
}

// Function to format the ENV-Pro data to a log with CO2 equivalent
String format_weather_log() {
    String output = "[WeatherStation, {";
    output += "Pressure: " + String(env_pressure, 1) + " hPa, ";
    output += "Temperature: " + String(env_temperature, 1) + " C, ";
    output += "Humidity: " + String(env_humidity, 1) + " %, ";
    output += "CO2 Equivalent: " + String(env_co2_equivalent, 1) + " ppm}";
    return output;
}

// Function to update sensor display when new SCD30 data is available
void displayLatestSensorData() {
    static uint16_t prev_co2[NUM_SENSORS]={0};
    bool changed = false;

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (co2_meas[i] != prev_co2[i]) {
            changed = true;
            prev_co2[i] = co2_meas[i];
        }
    }

    if (changed) {
        if (isZoomed) {
            displayZoomedSensor();
        } else {
            if (displayMode == 3) {
                displayWeatherStation();
            } else {
                displaySensorStates();
            }
        }
    }
}

// Function to get data from an SCD30 sensor
void get_data_from_sensor(int number) {
    static unsigned long sensor_wait_start[NUM_SENSORS] = {0};
    static bool waiting[NUM_SENSORS] = {false};
    static uint16_t temp_co2[NUM_SENSORS] = {0};
    static float temp_temp[NUM_SENSORS] = {0};
    static float temp_rh[NUM_SENSORS] = {0};

    if (!waiting[number]) {
        tcaselect(number);
        sensor_wait_start[number] = millis();
        waiting[number] = true;
    }

    if (waiting[number] && millis() - sensor_wait_start[number] >= 100) {
        if (sensors[number].dataAvailable()) {
            // Store temporary data
            temp_co2[number] = sensors[number].getCO2();
            temp_temp[number] = sensors[number].getTemperature();
            temp_rh[number] = sensors[number].getHumidity();
            #ifdef DEBUG
            Serial.print("New data polled for sensor: ");
            Serial.println(number + 1);
            Serial.printf("Temp CO2: %d ppm, Temp: %.1f C, RH: %.1f%%\n", 
                         temp_co2[number], temp_temp[number], temp_rh[number]);
            #endif
        }
        waiting[number] = false;
    }
}

// ENV-Pro: Check BSEC status and display errors on LCD
void checkBsecStatus(Bsec2 bsec) {
    if (bsec.status < BSEC_OK) {
        Serial.println("BSEC error code: " + String(bsec.status));
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.drawRect(rectX, rectY, rectWidth, rectHeight, WHITE);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(RED);
        int textX = rectX + (rectWidth - 7 * 12) / 2;
        int textY1 = rectY + (rectHeight - 2 * 16) / 2;
        M5.Lcd.setCursor(textX - 4, textY1);
        M5.Lcd.print("BSEC ERROR");
        delay(2000);
    } else if (bsec.status > BSEC_OK) {
        Serial.println("BSEC warning code: " + String(bsec.status));
    }

    if (bsec.sensor.status < BME68X_OK) {
        Serial.println("BME68X error code: " + String(bsec.sensor.status));
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.drawRect(rectX, rectY, rectWidth, rectHeight, WHITE);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(RED);
        int textX = rectX + (rectWidth - 7 * 12) / 2;
        int textY1 = rectY + (rectHeight - 2 * 16) / 2;
        M5.Lcd.setCursor(textX - 4, textY1);
        M5.Lcd.print("BME68X ERROR");
        delay(2000);
    } else if (bsec.sensor.status > BME68X_OK) {
        Serial.println("BME68X warning code: " + String(bsec.sensor.status));
    }
}

// ENV-Pro: Callback function to handle new data
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec) {
    if (!outputs.nOutputs) return;

    for (uint8_t i = 0; i < outputs.nOutputs; i++) {
        const bsecData output = outputs.output[i];
        switch (output.sensor_id) {
            case BSEC_OUTPUT_RAW_TEMPERATURE:
                env_temperature = output.signal;
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
                env_pressure = output.signal / 100.0; // Convert Pa to hPa
                break;
            case BSEC_OUTPUT_RAW_HUMIDITY:
                env_humidity = output.signal;
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
                env_co2_equivalent = output.signal;
                break;
            default:
                break;
        }
    }
}

// ENV-Pro: Initialize the ENV-Pro sensor with CO2 equivalent
void init_env_pro() {
    /* Desired subscription list of BSEC2 outputs */
    bsecSensor sensorList[] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_CO2_EQUIVALENT
    };

    /* Initialize the library and interfaces */
    if (!envSensor.begin(BME68X_I2C_ADDR_HIGH, Wire)) {
        checkBsecStatus(envSensor);
        return;
    }

    /* Subscribe to the desired BSEC2 outputs */
    if (!envSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_LP)) {
        checkBsecStatus(envSensor);
        return;
    }

    /* Attach the callback for new data */
    envSensor.attachCallback(newDataCallback);

    Serial.println("BSEC library version " + String(envSensor.version.major) +
                   "." + String(envSensor.version.minor) + "." +
                   String(envSensor.version.major_bugfix) + "." +
                   String(envSensor.version.minor_bugfix));
}

// ENV-Pro: Read and update data from the ENV-Pro sensor
void read_env_pro_data() {
    if (!envSensor.run()) {
        checkBsecStatus(envSensor);
    }
}

void command_handler(String command) {
    command.trim();
    #ifdef DEBUG
    Serial.print("Command was: ");
    Serial.println(command);
    #endif
    if (command == "Init") {
        clearSerialBuffer();
        #ifdef DEBUG
        Serial.println("Buffer successfully reset, switches turned on");
        #endif
    }
    else if (command == "Get data") {
        for (int i = 0; i < NUM_SENSORS; i++) {
            Serial.println(format_log(i));
        }
        Serial.println(format_weather_log());
    }
    else if (command == "CalibrateAll") {
        #ifdef DEBUG
        Serial.println("Calibrating all SCD30 sensors");
        #endif
        for (int i = 0; i < NUM_SENSORS; i++) {
            calibrate_sensor(i);
            delay(SENSING_INTERVAL*1000/(NUM_SENSORS+1)); // Small delay to maintain responsiveness
        }
        if (isZoomed) {
            displayZoomedSensor();
        } else {
            if (displayMode == 3) {
                displayWeatherStation();
            } else {
                displaySensorStates();
            }
        }
    }
    else if (command.startsWith("Calibrate sensor ")) {
        String sensor_num_str = command.substring(16); // Extract number after "Calibrate sensor "
        sensor_num_str.trim();
        int sensor_num = sensor_num_str.toInt();
        if (sensor_num >= 1 && sensor_num <= NUM_SENSORS) {
            #ifdef DEBUG
            Serial.printf("Calibrating SCD30 sensor %d\n", sensor_num);
            #endif
            calibrate_sensor(sensor_num - 1); // Convert to 0-based index
            if (isZoomed) {
                displayZoomedSensor();
            } else {
                if (displayMode == 3) {
                    displayWeatherStation();
                } else {
                    displaySensorStates();
                }
            }
        } else {
            #ifdef DEBUG
            Serial.printf("Error: Invalid sensor number %s (must be 1 to %d)\n", sensor_num_str.c_str(), NUM_SENSORS);
            #endif
        }
    }
    else {
        #ifdef DEBUG
        Serial.println("Command not recognized");
        #endif
    }
}

void setup() {
    // Initialize M5StickCPlus
    M5.begin();
    displayStartingScreen();
    Serial.begin(BAUD_RATE);
    Serial.println("M5StickC started");
    Wire.begin(21, 22); // M5StickCPlus I2C pins: SDA=21, SCL=22
    delay(50);

    #ifdef DEBUG
    scanI2CAddresses();
    #endif

    // Initialize SCD30 sensors
    for (int i = 0; i < NUM_SENSORS; i++) {
        init_sensor(i);
        delay(SENSING_INTERVAL*1000/(NUM_SENSORS+1));
    }

    // Initialize ENV-Pro
    init_env_pro();

    delay(200);
    clearSerialBuffer();
    if (displayMode == 3) {
        displayWeatherStation();
    } else {
        displaySensorStates();
    }
}

void loop() {
    static unsigned long last_loop = millis();
    static unsigned long last_sensor_poll = 0;
    static unsigned long last_display_update = 0;
    static unsigned long last_serial_update = 0;
    static uint8_t current_sensor = 0;
    static uint8_t sensors_polled = 0;
    static uint16_t temp_co2[NUM_SENSORS] = {0};
    static float temp_temp[NUM_SENSORS] = {0};
    static float temp_rh[NUM_SENSORS] = {0};

    // Run loop every 20ms for button responsiveness
    if (millis() - last_loop >= 20) {
        last_loop = millis();

        M5.update(); // Update button states

        // Handle Button A (navigate sensors)
        if (M5.BtnA.wasReleased()) {
            selectedSensor = (selectedSensor % NUM_SENSORS) + 1;
            #ifdef DEBUG
            Serial.printf("Selected Sensor %d\n", selectedSensor);
            #endif
            if (isZoomed) {
                displayZoomedSensor();
            } else {
                if (displayMode == 3) {
                    displayWeatherStation();
                } else {
                    displaySensorStates();
                }
            }
        }

        // Handle Button B (toggle zoomed view)
        if (M5.BtnB.wasReleased()) {
            isZoomed = !isZoomed;
            if (isZoomed) {
                displayZoomedSensor();
            } else {
                if (displayMode == 3) {
                    displayWeatherStation();
                } else {
                    displaySensorStates();
                }
            }
        }

        // Handle Button A long press (cycle display mode)
        if (M5.BtnA.wasReleasefor(1000)) {
            displayMode = (displayMode + 1) % 4; // Cycle through CO2, Temp, Humidity, Weather Station
            #ifdef DEBUG
            Serial.printf("Display mode changed to %d\n", displayMode);
            #endif
            if (!isZoomed) {
                if (displayMode == 3) {
                    displayWeatherStation();
                } else {
                    displaySensorStates();
                }
            }
        }

        // Handle Button B long press (calibrate SCD30 sensors)
        if (M5.BtnB.wasReleasefor(3500)) {
            if (isZoomed) {
                #ifdef DEBUG
                Serial.printf("Calibrating selected SCD30 sensor %d\n", selectedSensor);
                #endif
                calibrate_sensor(selectedSensor - 1);
                displayZoomedSensor();
            } else {
                #ifdef DEBUG
                Serial.println("Calibrating all SCD30 sensors");
                #endif
                for (int i = 0; i < NUM_SENSORS; i++) {
                    calibrate_sensor(i);
                    unsigned long start = millis();
                    while (millis() - start < SENSING_INTERVAL*1000/(NUM_SENSORS+1)) { M5.update(); }
                }
                if (displayMode == 3) {
                    displayWeatherStation();
                } else {
                    displaySensorStates();
                }
            }
        }

        // Poll one SCD30 sensor every ~SENSING_INTERVAL*1000/NUM_SENSORS ms
        if (millis() - last_sensor_poll >= (SENSING_INTERVAL * 1000 / NUM_SENSORS)) {
            get_data_from_sensor(current_sensor);
            // Store data temporarily
            if (sensors[current_sensor].dataAvailable()) {
                temp_co2[current_sensor] = sensors[current_sensor].getCO2();
                temp_temp[current_sensor] = sensors[current_sensor].getTemperature();
                temp_rh[current_sensor] = sensors[current_sensor].getHumidity();
                sensors_polled++;
            }
            current_sensor = (current_sensor + 1) % NUM_SENSORS;
            last_sensor_poll = millis();

            // If all SCD30 sensors have been polled, update global data and display
            if (sensors_polled >= NUM_SENSORS && millis() - last_display_update >= ACTUATING_INTERVAL * 1000) {
                for (int i = 0; i < NUM_SENSORS; i++) {
                    co2_meas[i] = temp_co2[i];
                    temp_meas[i] = temp_temp[i];
                    rh_meas[i] = temp_rh[i];
                }
                // Update ENV-Pro data
                read_env_pro_data();
                displayLatestSensorData();
                sensors_polled = 0; // Reset counter
                last_display_update = millis();
            }
        }

        // Update Serial output every SERIAL_INTERVAL seconds
        if (millis() - last_serial_update >= SERIAL_INTERVAL * 1000) {
            Serial.println("Serial data output for all sensors:");
            for (int i = 0; i < NUM_SENSORS; i++) {
                String output = format_log(i);
                Serial.println(output);
            }
            Serial.println(format_weather_log());
            last_serial_update = millis();
        }

        // Handle serial input
        static String inputString = "";
        static bool stringComplete = false;
        while (Serial.available()) {
            char inChar = (char)Serial.read();
            inputString += inChar;
            if (inChar == '\n') {
                stringComplete = true;
                break;
            }
        }

        if (stringComplete) {
            command_handler(inputString);
            inputString = "";
            stringComplete = false;
        }
    }
}