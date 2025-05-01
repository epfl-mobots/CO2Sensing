/* Cyril Monette, September 2024, modified by Romain Lattion for 6 SCD30, May 2025
This code is for the M5StickCPlus to read data from the SCD30 sensors at Bassenges and send it to the RPi for logging

  Recognized commands from serial:
    - "Init"          : Resets the serial buffer and initializes sensor communication.
    - "Get data"      : Prints the latest CO2, temperature, and humidity measurements for all sensors in the format [SCD30, sensor_nb, {CO2, temp, RH}].
 
  M5StickCPlus button commands:
    - M5 button (A) press            : Navigate through sensors 1–6 (highlight selected one)
    - Right button (B) press         : Toggle between main display (all sensor states) and zoomed display (selected sensor)
    - M5 button hold for 2 sec       : Cycle through display modes in main view (CO2, Temperature, Humidity)
    - Right button hold for 5 sec    : In main mode, initialize all sensors; in zoomed mode, initialize the selected sensor
 
  Sensor Mapping:
    - 6 SCD30 sensors connected to TCA9548A I2C multiplexer:
        • Sensor 1: Channel 0
        • Sensor 2: Channel 1
        • Sensor 3: Channel 2
        • Sensor 4: Channel 3
        • Sensor 5: Channel 4
        • Sensor 6: Channel 5
 
  Display:
    - Main mode: Shows states of all sensors (CO2, Temp, or Humidity based on display mode), with the selected sensor highlighted in yellow. Includes button instructions at the bottom:
        • "M5: Navigate"
        • "Right: Select"
        • "Hold M5 (2s): Cycle through states"
        • "Hold Right (5s): Init. all sensors"
    - Zoomed mode: Shows CO2, temperature, and humidity for the selected sensor, with button instructions:
        • "M5: Navigate"
        • "Right: Exit"
        • "Hold M5 (2s): Cycle through states"
        • "Hold Right (5s): Init. selected sensor"
 
  Notes:
    - Loop runs every 20ms to check buttons for responsiveness.
    - Sensors are polled every SENSING_INTERVAL seconds (one sensor every ~SENSING_INTERVAL*1000/NUM_SENSORS ms)
    - Display is updated every ACTUATING_INTERVAL seconds (>= SENSING_INTERVAL) after all sensors are polled.
    - Serial output is printed every SERIAL_INTERVAL seconds (> SENSING_INTERVAL).

*/
#include "M5StickCPlus.h"
#include "SparkFun_SCD30_Arduino_Library.h"

#define NUM_SENSORS 6 // Number of sensors connected to the TCA9548A
#define SENSING_INTERVAL 8 // Seconds
#define ACTUATING_INTERVAL SENSING_INTERVAL // Seconds, must be >= SENSING_INTERVAL
#define SERIAL_INTERVAL 10 // Seconds, must be > SENSING_INTERVAL

#define ADDRESS 0x70 // I2C address of the TCA9548A
#define BAUD_RATE 115200

#define DEBUG
const static bool display = true;

static SCD30 sensors[NUM_SENSORS]; // Create an array of SCD30 sensors

static uint16_t co2_meas[NUM_SENSORS]={0}; // Array to store the last measurements of the sensors
static float rh_meas[NUM_SENSORS]={0}; // Array to store the last measurements of the sensors
static float temp_meas[NUM_SENSORS]={0}; // Array to store the last measurements of the sensors

// LCD dimensions: 135x240 pixels
int rectWidth = 120;
int rectHeight = 60;
int rectX = (135 - rectWidth) / 2;
int rectY = 7;

// Button and display state
static uint8_t selectedSensor = 1; // Current sensor (1 to 6)
static bool isZoomed = false; // Whether zoomed display is active
static uint8_t displayMode = 0; // 0: CO2, 1: Temperature, 2: Humidity

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
    M5.Lcd.print("    Hold Right (5s)");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":\n Init. selected sensor");
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
    M5.Lcd.print("    Hold Right (5s)");
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.println(":\n   Init. all sensors");
}

// Function to select the TCA (I2C hub) channel 
void tcaselect(uint8_t i) {
    if (i > NUM_SENSORS-1){
        char error_msg[50];
        sprintf(error_msg, "Error: Channel %d is higher than SENSORS_NB", i);
        #ifdef DEBUG
        Serial.println(error_msg);
        #endif
        delay(2000);
        return;
    }
 
    Wire.beginTransmission(ADDRESS); 
    Wire.write(1 << i); // select channel i 
    Wire.endTransmission();  
}

// Function to parametrise the sensors
// Parameters: sensingInterval: the interval between measurements in seconds, between 2 and 100 seconds
static void parametriseSensors(int number){
    sensors[number].setMeasurementInterval(SENSING_INTERVAL);//Change number of seconds between measurements: 2 to 1800 (30 minutes), stored in non-volatile memory of SCD30
    delay(200);
    int interval = sensors[number].getMeasurementInterval();
    char msg[50];
    sprintf(msg, "Measurement Interval: %d", interval);
    #ifdef DEBUG
    Serial.println(msg);
    #endif

    //The lab is ~400m above sealevel
    sensors[number].setAltitudeCompensation(400);
    delay(200);
}

static void init_sensor(uint8_t sensor_nb){
    sensors[sensor_nb] = SCD30();
    tcaselect(sensor_nb);
    bool success = sensors[sensor_nb].begin(Wire);
    displayInitStatus(sensor_nb, success);
    while(!success){
        char error_msg[50];
        sprintf(error_msg, "Error: Could not connect to sensor %d", sensor_nb);
        #ifdef DEBUG
        Serial.println(error_msg);
        #endif 
        delay(500);
        tcaselect(sensor_nb);
        success = sensors[sensor_nb].begin(Wire);
        displayInitStatus(sensor_nb, success);
    }
 
    char success_msg[50];
    sprintf(success_msg, "Connected to sensor %d", sensor_nb);
    #ifdef DEBUG
    Serial.println(success_msg);
    #endif
    parametriseSensors(sensor_nb);
}

// Function to scan all I2C addresses and print the addresses of the devices found or an error message
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

// Function to format the latest measurements of the sensors to a log
// Example of formatted log: "[SCD30, 2, {410, 23.2, 53.0}]"
String format_log(uint8_t sensor_nb){
    String output = "[SCD30, ";
    output = output + String(sensor_nb + 1) + ", ";
    output = output + "{";
    output = output + String(co2_meas[sensor_nb]) + ", ";
    output = output + String(temp_meas[sensor_nb]) + ", ";
    output = output + String(rh_meas[sensor_nb]) + "}";
    output = output + "]";
    return output;
}

// Function to update sensor display when new data is available
void displayLatestSensorData(){
    static uint16_t prev_co2[NUM_SENSORS]={0};
    bool changed = false;

    for (int i = 0; i < NUM_SENSORS; i++){
        if (co2_meas[i] != prev_co2[i]){
            changed = true;
            prev_co2[i] = co2_meas[i];
        }
    }

    if (changed) {
        if (isZoomed) {
            displayZoomedSensor();
        } else {
            displaySensorStates();
        }
    }
}

// Function to get data from the sensor
void get_data_from_sensor(int number){
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

void command_handler(String command){
    command.trim();
    #ifdef DEBUG
    Serial.print("Command was: ");
    Serial.println(command);
    #endif
    if (command == "Init"){
        clearSerialBuffer();
        #ifdef DEBUG
        Serial.println("Buffer successfully reset, switches turned on");
        #endif
    }
    else if (command == "Get data"){
        for (int i = 0; i < NUM_SENSORS; i++){
            Serial.println(format_log(i));
        }
    }
    else {
        #ifdef DEBUG
        Serial.println("Command not recognized");
        #endif
    }
}

void setup() {
    M5.begin();
    displayStartingScreen();
    Serial.begin(BAUD_RATE);
    Serial.println("M5StickC started");
    Wire.begin();
    delay(50);
    
    #ifdef DEBUG
    scanI2CAddresses();
    #endif

    for (int i = 0; i < NUM_SENSORS; i++){
        init_sensor(i);
        delay(SENSING_INTERVAL*1000/(NUM_SENSORS+1));
    }

    delay(200);
    clearSerialBuffer();
    displaySensorStates();
}

void loop(){
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
                displaySensorStates();
            }
        }

        // Handle Button B (toggle zoomed view)
        if (M5.BtnB.wasReleased()) {
            isZoomed = !isZoomed;
            if (isZoomed) {
                displayZoomedSensor();
            } else {
                displaySensorStates();
            }
        }

        // Handle Button A long press (cycle display mode)
        if (M5.BtnA.wasReleasefor(1000)) {
            displayMode = (displayMode + 1) % 3; // Cycle through CO2, Temp, Humidity
            #ifdef DEBUG
            Serial.printf("Display mode changed to %d\n", displayMode);
            #endif
            if (!isZoomed) {
                displaySensorStates();
            }
        }

        // Handle Button B long press (initialize sensors)
        if (M5.BtnB.wasReleasefor(3500)) {
            if (isZoomed) {
                #ifdef DEBUG
                Serial.printf("Initializing selected sensor %d\n", selectedSensor);
                #endif
                init_sensor(selectedSensor - 1);
                displayZoomedSensor();
            } else {
                #ifdef DEBUG
                Serial.println("Initializing all sensors");
                #endif
                for (int i = 0; i < NUM_SENSORS; i++) {
                    init_sensor(i);
                    unsigned long start = millis();
                    while (millis() - start < SENSING_INTERVAL*1000/(NUM_SENSORS+1)) { M5.update(); }
                }
                displaySensorStates();
            }
        }

        // Poll one sensor every ~SENSING_INTERVAL*1000/NUM_SENSORS ms
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

            // If all sensors have been polled, update global data and display
            if (sensors_polled >= NUM_SENSORS && millis() - last_display_update >= ACTUATING_INTERVAL * 1000) {
                for (int i = 0; i < NUM_SENSORS; i++) {
                    co2_meas[i] = temp_co2[i];
                    temp_meas[i] = temp_temp[i];
                    rh_meas[i] = temp_rh[i];
                }
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