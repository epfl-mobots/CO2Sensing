// Cyril Monette, September 2024
// This code is for the M5StickC+ to read data from the SCD30 sensors at Bassenges and send it to the RPi for logging

#include "M5StickCPlus.h"
#include "SparkFun_SCD30_Arduino_Library.h"

#define NUM_SENSORS 2 // Number of sensors connected to the TCA9548A
#define SENSING_INTERVAL 4 // Seconds

#define ADDRESS 0x70 // I2C address of the TCA9548A
#define BAUD_RATE 115200

//#define DEBUG
const static bool display = true;

static SCD30 sensors[NUM_SENSORS]; // Create an array of SCD30 sensors

static uint16_t co2_meas[NUM_SENSORS]={0}; // Array to store the last measurements of the sensors
static float rh_meas[NUM_SENSORS]={0}; // Array to store the last measurements of the sensors
static float temp_meas[NUM_SENSORS]={0}; // Array to store the last measurements of the sensors


void clearSerialBuffer() {
    while (Serial.available() > 0) {
        Serial.read();
    }
    Serial.flush();
}

void displayTextLCD(const char *text, uint16_t pos_x, uint16_t pos_y, uint16_t color,bool additive=false){
    if (display){
        if(!additive) M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(pos_x, pos_y);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(color);
        M5.Lcd.printf(text);
    }
}

// Function to select the TCA (I2C hub) channel 
// parameters: i: the channel number to select, between 0 and 5
void tcaselect(uint8_t i) {
    if (i > NUM_SENSORS-1){
        // Display error msg on the LCD screen
        char error_msg[50];
        sprintf(error_msg, "Error: Channel %d is higher than SENSORS_NB", i);
        displayTextLCD(error_msg, 0, 0,RED);    

        #ifdef DEBUG
        Serial.println(error_msg);
        #endif
        delay(2000);    // Ensure the error message is displayed for at least 2 seconds
        return;
    }
 
    Wire.beginTransmission(ADDRESS); 
    Wire.write(1 << i); // select channel i 
    Wire.endTransmission();  
}

// Function to parametrise the sensors
// Parameters: sensingInterval: the interval between measurements in seconds, between 2 and 100 seconds
static void parametriseSensors(int number){
    sensors[number].setMeasurementInterval(SENSING_INTERVAL); //Change number of seconds between measurements: 2 to 1800 (30 minutes), stored in non-volatile memory of SCD30
     
    //While the setting is recorded, it is not immediately available to be read.
    delay(200);
    int interval = sensors[number].getMeasurementInterval(); //Get the measurment interval from the sensor
    char msg[50];
    sprintf(msg, "Measurement Interval: %d", interval);
    displayTextLCD(msg, 0, 0,GREEN);
    #ifdef DEBUG
    Serial.println(msg);
    #endif

    //The lab is ~400m above sealevel
    sensors[number].setAltitudeCompensation(400); //Set altitude of the sensor in m, stored in non-volatile memory of SCD30
    delay(200);
}

static void init_sensor(uint8_t sensor_nb){
    sensors[sensor_nb] = SCD30();

    tcaselect(sensor_nb); //Select the desired channel
    while(!sensors[sensor_nb].begin(Wire)){ //Pass the Wire port to the .begin() function
        char error_msg[50];
        sprintf(error_msg, "Error: Could not connect to sensor %d", sensor_nb);
        displayTextLCD(error_msg, 0, 0,RED);
        #ifdef DEBUG
        Serial.println(error_msg);
        #endif 
        delay(500);
    }
    char success_msg[50];
    sprintf(success_msg, "Connected to sensor %d", sensor_nb);
    displayTextLCD(success_msg, 0, 0,GREEN);
    delay(1000);
    #ifdef DEBUG
    Serial.println(success_msg);
    #endif
    parametriseSensors(sensor_nb); // Set the sensing interval
}

// Function to scan all I2C addresses and print the addresses of the devices found or an error message
// helps for debug
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
    output = output + String(sensor_nb) + ", ";
    output = output + "{";
    output = output + String(co2_meas[sensor_nb]) + ", ";
    output = output + String(temp_meas[sensor_nb]) + ", ";
    output = output + String(rh_meas[sensor_nb]) + "}";
    output = output + "]";
    return output;
}

static void displayLatestCO2(){
    static uint16_t prev_co2[NUM_SENSORS]={0};
    //Check if any of the values have changed
    bool changed = false;
    for (int i = 0; i < NUM_SENSORS; i++){
        if (co2_meas[i] != prev_co2[i]){
            changed = true;
            prev_co2[i] = co2_meas[i];
        }
    }
    if(changed){
        // Clear the screen
        if(display) M5.Lcd.fillScreen(BLACK);
        // Prepare the data to be displayed on the LCD screen
        char msg[50];
        for(int i = 0; i < NUM_SENSORS; i++){
            sprintf(msg, "CO2 %d:\n %d ppm", i, co2_meas[i]);
            displayTextLCD(msg, 20, 30 + 60*i, YELLOW,true);
        }
    }
}

// Function to get data from the sensor
// Parameters: number: the sensor number to get data from, between 0 and 5
void get_data_from_sensor(int number){
    tcaselect(number); //Select the desired channel
    delay(100); // Wait for the sensor to be ready

    // Check if the sensor has new data available
    if(sensors[number].dataAvailable()){
        co2_meas[number] = sensors[number].getCO2();
        temp_meas[number] = sensors[number].getTemperature();
        rh_meas[number] = sensors[number].getHumidity();

        displayLatestCO2();

        #ifdef DEBUG
        Serial.print("New data available for sensor: ");
        Serial.println(number);
        // Prepare data in the following format: [SCD30, sensor_nb, {temp, rh, co2}]
        String output = format_log(number);
        Serial.println(output);
        #endif
    }
    #ifdef DEBUG
    else{
        Serial.print("No data available for sensor: ");
        Serial.println(number);
    }
    #endif
}

void command_handler(String command){
    command.trim();
    #ifdef DEBUG
    Serial.print("Command was: ");
    Serial.println(command);
    #endif
    if (command == "Init"){ // clear the serial buffer  
        clearSerialBuffer();
        #ifdef DEBUG
        Serial.println("Buffer successfully reset, switches turned on");
        #endif
    }
    else if (command == "Get data"){
        // Get data from all sensors
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
    delay(2000);
    Serial.println("M5StickC started");
    Wire.begin();
    delay(50);
    
    #ifdef DEBUG
    scanI2CAddresses(); // Scan all I2C addresses to find the sensors and to see if there are any errors
    #endif

    // Initialise all sensors
    for (int i = 0; i < NUM_SENSORS; i++){
        init_sensor(i);
        delay(SENSING_INTERVAL*1000/(NUM_SENSORS+1)); // To space out the sensor measurements equally
        //sensors[i].setForcedRecalibrationFactor(400); //Uncomment to recalibrate the sensor
    }

    delay(200);
    clearSerialBuffer();
}

// Loop function
void loop(){
    static unsigned long start_time = millis();
    if (millis() - start_time >= 200){
        start_time = millis();
        // Get data from sensors
        for (int i = 0; i < NUM_SENSORS; i++){
            get_data_from_sensor(i);
        }
    }

    static String inputString = "";  
    static bool stringComplete = false;
    // Handle serial input
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        inputString += inChar;
        
        if (inChar == '\n') {
            stringComplete = true;
            break;
        }
    }
    
    // Process the command if complete
    if (stringComplete) {
        command_handler(inputString);
        inputString = "";         
        stringComplete = false;  
    }
    delay(20);
}