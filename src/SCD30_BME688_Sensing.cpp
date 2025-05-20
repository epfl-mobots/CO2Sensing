/* Cyril Monette, September 2024, modified by Romain Lattion for 6 SCD30, May 2025
This code is for the M5StickCPlus to read data from the SCD30 sensors at Bassenges and send it to the RPi for logging,
with additional BME688 sensor for weather data.

  Recognized commands from serial:
    - "Init"          : Resets the serial buffer and initializes sensor communication
    - "Get data"      : Prints the latest CO2, temperature, and humidity measurements for all SCD30 sensors and BME688 data
    - "CalibrateAll"  : Calibrates all SCD30 sensors to 400 ppm CO2
    - "Calibrate sensor x"  : Calibrates SCD30 sensor x to 400 ppm CO2
    - "Get name"      : Returns the device name "M5Stick2"

  M5StickCPlus button commands:
    - M5 button (A) press            : Navigate through SCD30 sensors 1–6 (highlight selected one) in CO2, Temp, Humidity modes
    - Right button (B) press         : Toggle between main display and zoomed display for CO2, Temp, Humidity modes; no effect in Weather mode
    - M5 button hold for 2 sec       : Cycle through display modes (CO2, Temperature, Humidity, Weather)
    - Right button hold for 5 sec    : In main mode, initialize all sensors; in zoomed mode, initialize the selected SCD30 sensor

  Sensor Mapping:
    - 6 SCD30 sensors connected to TCA9548A I2C multiplexer:
        • Sensor 1: Channel 0
        • Sensor 2: Channel 1
        • Sensor 3: Channel 2
        • Sensor 4: Channel 3
        • Sensor 5: Channel 4
        • Sensor 6: Channel 5
    - 1 BME688 sensor connected directly to I2C bus at address 0x77

  Display:
    - Main mode: Shows states of all SCD30 sensors (CO2, Temp, or Humidity) or BME688 data (Weather mode: Temp, Press, RH, Gas if enabled), with the selected SCD30 sensor highlighted in yellow. Includes button instructions.
    - Zoomed mode: Shows CO2, temperature, and humidity for the selected SCD30 sensor in CO2, Temp, Humidity modes; not available in Weather mode.

  Notes:
    - Loop runs every 20ms for button responsiveness.
    - SCD30 sensors are polled every SENSING_INTERVAL seconds (one sensor every ~SENSING_INTERVAL*1000/NUM_SENSORS ms).
    - BME688 sensor is polled every SENSING_INTERVAL seconds.
    - Serial output is printed every SERIAL_INTERVAL seconds if SERIAL_DISPLAY is defined, or on "Get data" command.

  Output ("Get data") style : 
[SCD30 1, {<CO2_ppm>, <temperature_C>, <humidity_%>}]
[SCD30 2, {<CO2_ppm>, <temperature_C>, <humidity_%>}]
[SCD30 3, {<CO2_ppm>, <temperature_C>, <humidity_%>}]
[SCD30 4, {<CO2_ppm>, <temperature_C>, <humidity_%>}]
[SCD30 5, {<CO2_ppm>, <temperature_C>, <humidity_%>}]
[SCD30 6, {<CO2_ppm>, <temperature_C>, <humidity_%>}]
[BME688, {<temperature_C>, <pressure_Pa>, <humidity_%>}]
*/

#include "M5StickCPlus.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include "bme68xLibrary.h"
/**
 * Copyright (C) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 */

// Unique device name for this M5StickC Plus
const char* DEVICE_NAME = "M5Stick2";

#define NUM_SENSORS 6 // Number of SCD30 sensors connected to the TCA9548A
#define SENSING_INTERVAL 8 // Seconds
#define ACTUATING_INTERVAL 8 // Seconds
#define SERIAL_INTERVAL 10 // Seconds

#define ADDRESS 0x74 // I2C address of the TCA9548A
#define BAUD_RATE 115200

//#define DEBUG
//#define SERIAL_DISPLAY // Enable serial output every SERIAL_INTERVAL
// [BME688, {<temperature_C>, <pressure_Pa>, <humidity_%>, <gas_resistance_ohm>, <status_HEX>, <gas_index>}]

const static bool display = true;

static SCD30 sensors[NUM_SENSORS]; // Array of SCD30 sensors
static uint16_t co2_meas[NUM_SENSORS]={0}; // SCD30 CO2 measurements
static float rh_meas[NUM_SENSORS]={0}; // SCD30 humidity measurements
static float temp_meas[NUM_SENSORS]={0}; // SCD30 temperature measurements

// BME688 sensor
#define NEW_GAS_MEAS (BME688_GASM_VALID_MSK | BME688_HEAT_STAB_MSK | BME688_NEW_DATA_MSK)
// #define GasSensing
static Bme68x bme;
static float bme_temp = 0;
static float bme_press = 0;
static float bme_rh = 0;
static bool bme_data_valid = false; // Track valid BME688 data
#ifdef GasSensing
static float bme_gas_res = 0;
static uint8_t bme_status = 0;
static uint8_t bme_gas_idx = 0;
#endif

// LCD dimensions: 135x240 pixels
int rectWidth = 120;
int rectHeight = 60;
int rectX = (135 - rectWidth) / 2;
int rectY = 7;

// Button and display state
static uint8_t selectedSensor = 1; // Current SCD30 sensor (1 to 6)
static bool isZoomed = false; // Whether zoomed display is active
static uint8_t displayMode = 0; // 0: CO2, 1: Temperature, 2: Humidity, 3: Weather

void clearSerialBuffer() {
    while (Serial.available() > 0) {
        Serial.read();
    }
    Serial.flush();
}

// Reset I2C bus
void resetI2C() {
    Wire.end();
    delay(10);
    Wire.begin();
    delay(50);
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
void displayInitStatus(uint8_t sensor_nb, bool success, bool isBme68x = false) {
    if (!display) return;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.drawRect(rectX, rectY, rectWidth, rectHeight, WHITE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);

    int textX = rectX + (rectWidth - 7 * 12) / 2;
    int textY1 = rectY + (rectHeight - 2 * 16) / 2;
    int textY2 = textY1 + 16;

    M5.Lcd.setCursor(textX - 4, textY1);
    if (isBme68x) {
        M5.Lcd.print(" BME688");
    } else {
        M5.Lcd.print(" SCD30 ");
        M5.Lcd.print(sensor_nb + 1);
    }
    M5.Lcd.setCursor(textX - 4, textY2);
    M5.Lcd.setTextColor(success ? GREEN : RED);
    M5.Lcd.print(success ? "   OK" : "   FAIL");
    delay(1000);
}

// Display calibration status for an SCD30 sensor
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
    M5.Lcd.print(" SCD30 ");
    M5.Lcd.print(sensor_nb + 1);
    M5.Lcd.setCursor(textX - 4, textY2);
    M5.Lcd.setTextColor(success ? GREEN : RED);
    M5.Lcd.print(success ? "   OK" : "   FAIL");
    delay(1000);
}

// Display zoomed view of selected SCD30 sensor (CO2, Temp, Humidity modes only)
void displayZoomedSensor() {
    if (!display || displayMode == 3) return;
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.drawRect(rectX, rectY, rectWidth, rectHeight/2+rectHeight/5, WHITE);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);

    int textX = rectX + (rectWidth - 7 * 12) / 2;
    int textY1 = rectY + (rectHeight - 2 * 16) / 2;

    M5.Lcd.setTextColor(YELLOW);
    M5.Lcd.setCursor(textX - 4, textY1);
    M5.Lcd.print("SCD30 ");
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
    M5.Lcd.println(":\n  Cycle through states");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("    Hold Right (5s)");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":\n  Calib. sel. SCD30");
}

// Display all sensor measurements for the current display mode
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
        case 3: M5.Lcd.print(" WEATHER"); break;
    }
    M5.Lcd.setCursor(textX + 3, textY2);
    M5.Lcd.print("STATES\n\n");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\n");

    if (displayMode == 3) {
        M5.Lcd.setTextColor(WHITE);
        if (bme_data_valid) {
            M5.Lcd.print("   Temp.: ");
            M5.Lcd.print(bme_temp, 1);
            M5.Lcd.println(" C");
            M5.Lcd.print("   Pressure: ");
            M5.Lcd.print(bme_press, 0);
            M5.Lcd.println(" Pa");
            M5.Lcd.print("   Humidity: ");
            M5.Lcd.print(bme_rh, 1);
            M5.Lcd.println(" %");
#ifdef GasSensing
            M5.Lcd.print("   Gas: ");
            M5.Lcd.print(bme_gas_res, 0);
            M5.Lcd.println(" ohm");
#endif
        } else {
            M5.Lcd.print("   BME688: No data");
        }
    } else {
        for (int i = 0; i < NUM_SENSORS; i++) {
            M5.Lcd.setTextColor(i + 1 == selectedSensor ? YELLOW : WHITE);
            M5.Lcd.print("   SCD30 ");
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
    }
    M5.Lcd.print("\n");
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("  BUTTONS");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\n\n\n    M5");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(displayMode == 3 ? ":None" : ":Navigate");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("    Right");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(displayMode == 3 ? ":None" : ":Select");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("    Hold M5 (2s)");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":\n  Cycle through states");
    M5.Lcd.setTextColor(RED);
    M5.Lcd.print("    Hold Right (5s)");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":\n    Calib. all SCD30");
}

// Function to select the TCA (I2C hub) channel
void tcaselect(uint8_t i) {
    if (i > NUM_SENSORS-1) {
        #ifdef DEBUG
        Serial.printf("Error: Channel %d is higher than SENSORS_NB (%d)\n", i, NUM_SENSORS);
        #endif
        return;
    }

    Wire.beginTransmission(ADDRESS);
    uint8_t error = Wire.write(1 << i); // select channel i
    error |= Wire.endTransmission();
    if (error) {
        #ifdef DEBUG
        Serial.printf("Error: Failed to select TCA9548A channel %d\n", i);
        #endif
        resetI2C();
    }
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
    while(!success) {
        char error_msg[50];
        sprintf(error_msg, "Error: Could not connect to SCD30 %d", sensor_nb+1);
        Serial.println(error_msg);
        delay(500);
        tcaselect(sensor_nb);
        success = sensors[sensor_nb].begin(Wire);
        displayInitStatus(sensor_nb, success);
    }
    char success_msg[50];
    sprintf(success_msg, "Connected to SCD30 %d", sensor_nb+1);
    Serial.println(success_msg);
    parametriseSensors(sensor_nb);
}

// Initialize the BME688 sensor
static void init_bme688() {
    bme.begin(0x77, Wire);
    delay(1000);
    bool success = !bme.checkStatus();
    if (!success) {
        if (bme.checkStatus() == BME68X_ERROR) {
            #ifdef DEBUG
            Serial.println("BME688 error: " + bme.statusString());
            #endif
        } else if (bme.checkStatus() == BME68X_WARNING) {
            #ifdef DEBUG
            Serial.println("BME688 Warning: " + bme.statusString());
            #endif
        }
    }
    displayInitStatus(0, success, true);
    while (!success) {
        Serial.println("Error: Could not connect to BME688");
        delay(500);
        bme.begin(0x77, Wire);
        delay(1000);
        success = !bme.checkStatus();
        displayInitStatus(0, success, true);
    }

    Serial.println("Connected to BME688");

    bme.setTPH();
#ifdef GasSensing
    uint16_t tempProf[10] = { 100, 200, 320 };
    uint16_t durProf[10] = { 150, 150, 150 };
    bme.setHeaterProf(tempProf, durProf, 3);
#endif

    bme.setSeqSleep(BME68X_ODR_1000_MS); // set to 1000ms
    bme.setOpMode(BME68X_SEQUENTIAL_MODE);
    bme_data_valid = false; // Reset data validity
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
    M5.Lcd.print(" SCD30 ");
    M5.Lcd.print(sensor_nb + 1);
    M5.Lcd.setCursor(textX - 7, textY1 + 16);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(ORANGE);
    M5.Lcd.print("CALIBRAT.");
    delay(500);

    sensors[sensor_nb].setForcedRecalibrationFactor(400);
    delay(200);

    bool success = false;
    if (sensors[sensor_nb].dataAvailable()) {
        uint16_t co2 = sensors[sensor_nb].getCO2();
        success = (co2 > 0);
    }

    displayCalibrationStatus(sensor_nb, success);
    if (success) {
        char success_msg[50];
        sprintf(success_msg, "Calibrated SCD30 %d to 400 ppm", sensor_nb + 1);
        Serial.println(success_msg);
    } else {
        char error_msg[50];
        sprintf(error_msg, "Error: Calibration failed for SCD30 %d", sensor_nb + 1);
        Serial.println(error_msg);
    }
}

// Function to scan all I2C addresses
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

// Format SCD30 log
String format_log(uint8_t sensor_nb) {
    String output = "[SCD30, ";
    output = output + String(sensor_nb + 1) + ", ";
    if (co2_meas[sensor_nb] == 0 && temp_meas[sensor_nb] == 0 && rh_meas[sensor_nb] == 0) {
        output = output + "No data";
    } else {
        output = output + "{";
        output = output + String(co2_meas[sensor_nb]) + ", ";
        output = output + String(temp_meas[sensor_nb], 1) + ", ";
        output = output + String(rh_meas[sensor_nb], 1) + "}";
    }
    output = output + "]";
    return output;
}

// Format BME68x log
String format_bme688_log() {
    String output = "[BME688, ";
    if (!bme_data_valid) {
        output = output + "No data]";
    } else {
        output = output + "{";
        output = output + String(bme_temp, 1) + ", ";
        output = output + String(bme_press, 0) + ", ";
        output = output + String(bme_rh, 1);
#ifdef GasSensing
        output = output + ", ";
        output = output + String(bme_gas_res, 0) + ", ";
        output = output + String(bme_status, HEX) + ", ";
        output = output + String(bme_gas_idx);
#endif
        output = output + "}]";
    }
    return output;
}

// Update sensor display when new data is available
void displayLatestSensorData() {
    static uint16_t prev_co2[NUM_SENSORS]={0};
    static float prev_bme_temp = 0;
    bool changed = false;

    for (int i = 0; i < NUM_SENSORS; i++) {
        if (co2_meas[i] != prev_co2[i]) {
            changed = true;
            prev_co2[i] = co2_meas[i];
        }
    }
    if (bme_data_valid && bme_temp != prev_bme_temp) {
        changed = true;
        prev_bme_temp = bme_temp;
    }

    if (changed) {
        if (isZoomed && displayMode != 3) {
            displayZoomedSensor();
        } else {
            displaySensorStates();
        }
    }
}

// Get data from an SCD30 sensor
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
            temp_co2[number] = sensors[number].getCO2();
            temp_temp[number] = sensors[number].getTemperature();
            temp_rh[number] = sensors[number].getHumidity();
            #ifdef DEBUG
            Serial.print("New data polled for sensor: ");
            Serial.println(number + 1);
            Serial.printf("Temp CO2: %d ppm, Temp: %.1f C, RH: %.1f%%\n",
                         temp_co2[number], temp_temp[number], temp_rh[number]);
            #endif
        } else if (millis() - sensor_wait_start[number] >= 1000) {
            #ifdef DEBUG
            Serial.printf("Error: No data available for SCD30 %d\n", number + 1);
            #endif
            resetI2C();
            waiting[number] = false;
        }
    }
}

// Get data from BME688 sensor
void get_data_from_bme688() {
    static unsigned long wait_start = 0;
    static bool waiting = false;
    bme68xData data;
    uint8_t nFieldsLeft = 0;

    if (!waiting) {
        if (bme.fetchData()) {
            wait_start = millis();
            waiting = true;
        }
    }

    if (waiting && millis() - wait_start >= 150) {
        bool new_data = false;
        do {
            nFieldsLeft = bme.getData(data);
            if (data.temperature != 0 || data.pressure != 0 || data.humidity != 0) {
                bme_temp = data.temperature;
                bme_press = data.pressure;
                bme_rh = data.humidity;
                bme_data_valid = true;
                new_data = true;
#ifdef GasSensing
                bme_gas_res = data.gas_resistance;
                bme_status = data.status;
                bme_gas_idx = data.gas_index;
                if (data.gas_index == 2) delay(250);
#endif
            }
        } while (nFieldsLeft);
        if (new_data) {
            #ifdef DEBUG
            Serial.print("New BME688 data: ");
            Serial.print(bme_temp, 1);
            Serial.print(" °C, ");
            Serial.print(bme_press, 0);
            Serial.print(" Pa, ");
            Serial.print(bme_rh, 1);
            Serial.print(" %");
#ifdef GasSensing
            Serial.print(", ");
            Serial.print(bme_gas_res, 0);
            Serial.print(" ohm, status: ");
            Serial.print(bme_status, HEX);
            Serial.print(", idx: ");
            Serial.print(bme_gas_idx);
#endif
            Serial.println();
            #endif
        } else if (millis() - wait_start >= 1000) {
            #ifdef DEBUG
            Serial.println("Error: No valid BME688 data");
            #endif
            resetI2C();
        }
        waiting = false;
    }
}

void command_handler(String command) {
    command.trim();
    #ifdef DEBUG
    Serial.print("Command was: ");
    Serial.println(command);
    #endif
    if (command == "Get name") {
        Serial.println(DEVICE_NAME);
    } else if (command == "Init") {
        clearSerialBuffer();
        #ifdef DEBUG
        Serial.println("Buffer successfully reset, switches turned on");
        #endif
        for (int i = 0; i < NUM_SENSORS; i++) {
            init_sensor(i);
            delay(SENSING_INTERVAL*1000/(NUM_SENSORS+1));
        }
        init_bme688();
    } else if (command == "Get data") {
        for (int i = 0; i < NUM_SENSORS; i++) {
            Serial.println(format_log(i));
        }
        Serial.println(format_bme688_log());
    } else if (command == "CalibrateAll") {
        #ifdef DEBUG
        Serial.println("Calibrating all SCD30 sensors");
        #endif
        for (int i = 0; i < NUM_SENSORS; i++) {
            calibrate_sensor(i);
            delay(SENSING_INTERVAL*1000/(NUM_SENSORS+1));
        }
        if (isZoomed && displayMode != 3) {
            displayZoomedSensor();
        } else {
            displaySensorStates();
        }
    } else if (command.startsWith("Calibrate SCD30 ")) {
        String sensor_num_str = command.substring(16);
        sensor_num_str.trim();
        int sensor_num = sensor_num_str.toInt();
        if (sensor_num >= 1 && sensor_num <= NUM_SENSORS) {
            #ifdef DEBUG
            Serial.printf("Calibrating SCD30 %d\n", sensor_num);
            #endif
            calibrate_sensor(sensor_num - 1);
            if (isZoomed && displayMode != 3) {
                displayZoomedSensor();
            } else {
                displaySensorStates();
            }
        } else {
            #ifdef DEBUG
            Serial.printf("Error: Invalid SCD30 number %s (must be 1 to %d)\n", sensor_num_str.c_str(), NUM_SENSORS);
            #endif
        }
    } else {
        Serial.println("Command not recognized");
        Serial.println("Recognized commands:");
        Serial.println("  - \"Init\": Resets the serial buffer and initializes SCD30 communication");
        Serial.println("  - \"Get data\": Prints the latest CO2, temperature, and humidity measurements for all SCD30 sensors and BME688 data");
        Serial.println("  - \"CalibrateAll\": Calibrates all SCD30 sensors to 400 ppm CO2");
        Serial.println("  - \"Calibrate SCD30 x\": Calibrates SCD30 sensor x to 400 ppm CO2");
        Serial.println("  - \"Get name\": Returns the device name");
    }
}

void setup() {
    M5.begin();
    displayStartingScreen();
    Serial.begin(BAUD_RATE);
    Serial.println("M5StickC-Plus started");
    Wire.begin();
    delay(50);

    #ifdef DEBUG
    scanI2CAddresses();
    #endif

    for (int i = 0; i < NUM_SENSORS; i++) {
        init_sensor(i);
        delay(SENSING_INTERVAL*1000/(NUM_SENSORS+1));
    }
    init_bme688();

    delay(200);
    clearSerialBuffer();
    Serial.println("M5StickC-Plus ready");
    displaySensorStates();
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

    if (millis() - last_loop >= 20) {
        last_loop = millis();
        M5.update();

        if (M5.BtnA.wasReleased() && displayMode != 3) {
            selectedSensor = (selectedSensor % NUM_SENSORS) + 1;
            #ifdef DEBUG
            Serial.printf("Selected SCD30 %d\n", selectedSensor);
            #endif
            if (isZoomed) {
                displayZoomedSensor();
            } else {
                displaySensorStates();
            }
        }

        if (M5.BtnB.wasReleased() && displayMode != 3) {
            isZoomed = !isZoomed;
            if (isZoomed) {
                displayZoomedSensor();
            } else {
                displaySensorStates();
            }
        }

        if (M5.BtnA.wasReleasefor(1000)) {
            displayMode = (displayMode + 1) % 4; // Cycle through CO2, Temp, Humidity, Weather
            isZoomed = false; // Exit zoomed mode when changing display mode
            #ifdef DEBUG
            Serial.printf("Display mode changed to %d\n", displayMode);
            #endif
            displaySensorStates();
        }

        if (M5.BtnB.wasReleasefor(3500)) {
            if (isZoomed && displayMode != 3) {
                #ifdef DEBUG
                Serial.printf("Calibrating selected SCD30 %d\n", selectedSensor);
                #endif
                calibrate_sensor(selectedSensor - 1);
                displayZoomedSensor();
            } else {
                #ifdef DEBUG
                Serial.println("Calibrating all sensors");
                #endif
                for (int i = 0; i < NUM_SENSORS; i++) {
                    calibrate_sensor(i);
                    unsigned long start = millis();
                    while (millis() - start < SENSING_INTERVAL*1000/(NUM_SENSORS+1)) { M5.update(); }
                }
                isZoomed = false;
                displaySensorStates();
            }
        }

        if (millis() - last_sensor_poll >= (SENSING_INTERVAL * 1000 / (NUM_SENSORS + 1))) {
            if (current_sensor < NUM_SENSORS) {
                get_data_from_sensor(current_sensor);
                if (sensors[current_sensor].dataAvailable()) {
                    temp_co2[current_sensor] = sensors[current_sensor].getCO2();
                    temp_temp[current_sensor] = sensors[current_sensor].getTemperature();
                    temp_rh[current_sensor] = sensors[current_sensor].getHumidity();
                    sensors_polled++;
                }
            } else {
                get_data_from_bme688();
                sensors_polled++;
            }
            current_sensor = (current_sensor + 1) % (NUM_SENSORS + 1);
            last_sensor_poll = millis();

            if (sensors_polled >= NUM_SENSORS + 1 && millis() - last_display_update >= ACTUATING_INTERVAL * 1000) {
                for (int i = 0; i < NUM_SENSORS; i++) {
                    co2_meas[i] = temp_co2[i];
                    temp_meas[i] = temp_temp[i];
                    rh_meas[i] = temp_rh[i];
                }
                displayLatestSensorData();
                sensors_polled = 0;
                last_display_update = millis();
            }
        }

#ifdef SERIAL_DISPLAY
        if (millis() - last_serial_update >= SERIAL_INTERVAL * 1000) {
            Serial.println("Serial data output for all sensors:");
            for (int i = 0; i < NUM_SENSORS; i++) {
                Serial.println(format_log(i));
            }
            Serial.println(format_bme688_log());
            last_serial_update = millis();
        }
#endif

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