#include <iostream>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <ros.h>
#include <chrono>
#include "motor.h"
#include "WiFiconection.h"
#include "utilities.h"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

// variable ThrottleDACValue is used to drive the car, 
// make sure the values are correct for safety.
// Stopping the car is ThrottleDACValue = DACCentre, NOT 0.

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

/*
int Kp = 1, Ki = 1, Kd = 1;
unsigned long currentTime = 0, previousTime = 0, elapsedTime = 0;
int diff_sum = 0, diff_prev = 0;
float output;
*/

int direction, speed;
int current_pos, desired_pos, diff=0;
float desired_steer, desired_speed;
bool USE_ROS=true;

// setup DAC for speed control
Adafruit_MCP4725 Throttle;
uint16_t ThrottleDACValue = DACCentre; //Value for ThrottleDACValue after reset/powerup
uint16_t battery_voltage = 0;
uint16_t voltage_reading = 0;


// setup ros node with publish and subscrib
ros::NodeHandle nh;
// the range of speed and steer msg are both -1 to 1
void steering_msg_cb(const std_msgs::Float32 &msg){
    desired_steer = msg.data;
    if (desired_steer>=0 ){
        desired_pos = translate(desired_steer, 0, 1, LA_MIDDLE, LA_UPPER_LIMIT);
    }else{
        desired_pos = translate(desired_steer, -1, 0, LA_LOWER_LIMIT, LA_MIDDLE);
    }
}
void speed_msg_cb(const std_msgs::Float32 &msg){
    desired_speed = msg.data;
    // map desired_speed ([-1,1]) to ThrottleDACValue ([DAC_Lower_LIMIT, DAC_Upper_LIMIT])
    if (desired_speed >= 0){
        ThrottleDACValue = translate(desired_speed, 0, 1, DACCentre, DAC_Lower_LIMIT);
    }else{
        ThrottleDACValue = translate(desired_speed, -1, 0, DAC_Upper_LIMIT, DACCentre);
    }
    // ThrottleDACValue = msg.data;
}
ros::Subscriber<std_msgs::Float32> steer_sub("steering_cmd", &steering_msg_cb);
ros::Subscriber<std_msgs::Float32> speed_sub("speed_cmd", &speed_msg_cb);
std_msgs::Int16 int16_msg;
ros::Publisher Linear_actuator_pos("linear_actuator_pos", &int16_msg);


// Forward declarations
void detect_I2C_device();
void reconnect_WiFi();
void read_steering(uint8_t iter);


void setup() {
    Serial.begin(115200);
    delay(100);
    
    if (USE_ROS){
        // connect to WiFi
        // if using iPhone, turn on Maximize compatibility
        // because by default, some only use 5GHz by bdefault
        // https://stackoverflow.com/questions/67084998/esp32-cant-connect-to-iphone-personal-hotspot
        WiFi.begin(SSID,PASS);
        Serial.print("WiFi connecting");
        while (WiFi.status() != WL_CONNECTED) {
            Serial.println("....");
            delay(100);
        }
        Serial.print(" connected! (");
        Serial.print(millis()*0.001);
        Serial.println("s");
        delay(100);
        
        // connect to ROS using WiFi, and register topics
        nh.getHardware()->setConnection(HOST_IP, serverPort);
        nh.initNode();
        Serial.println(" ros init complete ");
        nh.advertise(Linear_actuator_pos);
        nh.subscribe(steer_sub);
        nh.subscribe(speed_sub);
        nh.spinOnce();
        delay(10);
    }

    // setup DAC
    Throttle.begin(0x62);//0x60
    delay(10);
    
    // setup PWM pins
    pinMode(RPWM_Output, OUTPUT);
    pinMode(LPWM_Output, OUTPUT);
    ledcSetup(Steering_CH_1, freq, resolution);
    ledcAttachPin(RPWM_Output, Steering_CH_1);
    ledcSetup(Steering_CH_2, freq, resolution);
    ledcAttachPin(LPWM_Output, Steering_CH_2);
    
    if (!USE_ROS){
        Serial.println("Please enter the desired position...");
        while (Serial.available()==0) {}
        desired_pos = Serial.parseInt();
    }else{
        desired_pos = LA_MIDDLE;
    }

    
}


void loop() {

    // set speed
    Throttle.setVoltage(ThrottleDACValue, false);

    // give command to the linear actuator
    if (abs(diff) < 15){
        brake();
    }else{
        if (diff > 0){ // linear actuator is too long
            speed = 255;
            direction = 0;
        }else{
            speed = 255; // linear actuator is too short
            direction = 1;
        }
        // become longer when direction=1, shorter when direction=0
        set_speed(speed, direction);
    }

    // For detecting I2C device detection
    //detect_I2C_device();

    if (USE_ROS){
        // reconnect to WiFi if connection is lost
        if(WiFi.status() != WL_CONNECTED){
            reconnect_WiFi();
        }
    }

    // getting the current steering (linear actuator position)
    // by averaging n times, and publish
    read_steering(1);

    // measure current battery voltage
    voltage_reading = analogRead(batt_vol);
    battery_voltage = voltage_reading * (3.3/1023) * RATIO;
    
    
    if (USE_ROS){
        // important to use spinOnce(process callbacks now). 
        // Had trouble connect to node if spinOnce is not used.
        // https://answers.ros.org/question/11887/significance-of-rosspinonce/
        nh.spinOnce();  
        delay(1); 
    }
    

    // don't need a PID because adding force does not move the actuator 
    /* with PID
    currentTime = millis();
    elapsedTime = currentTime - previousTime;
    diff_sum += diff;
    output = Kp*diff + Ki*diff_sum + Kd*(diff-diff_prev)/elapsedTime;
    previousTime = currentTime;
    diff_prev = diff;
    Serial.println(currentTime);
    Serial.println(previousTime);
    Serial.println(diff);
    Serial.println(output);
    */

}

void detect_I2C_device(){
    byte error, address;
    int nDevices;
    Serial.println("Scanning...");
    nDevices = 0;
    for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
        Serial.print("I2C device found at address 0x");
        if (address<16) {
        Serial.print("0");
        }
        Serial.println(address,HEX);
        nDevices++;
    }
    else if (error==4) {
        Serial.print("Unknow error at address 0x");
        if (address<16) {
        Serial.print("0");
        }
        Serial.println(address,HEX);
    }    
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found\n");
    }
    else {
        Serial.println("done\n");
    }
    delay(5000); 
}

void reconnect_WiFi(){
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    Serial.print("WiFi reconnecting");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println("....");
        delay(100);
    }
    Serial.print("WiFi reconnected");
}

void read_steering(uint8_t iter){
    auto t1 = high_resolution_clock::now();
    current_pos = 0;
    int count_num = 0;
    while (true){
        current_pos += analogRead(position_pin);
        count_num += 1;
        auto t2 = high_resolution_clock::now();
        /* Getting number of milliseconds as an integer. */
        auto ms_int = duration_cast<milliseconds>(t2 - t1);
        if (ms_int.count() > 50){
            break;
        }
    }
    current_pos = current_pos/count_num;
    // desired_steer:[-1,1] need to be mapped to desired_pos:[?]
    diff = current_pos - desired_pos;
    Serial.print("ThrottleDACValue: ");
    Serial.print(ThrottleDACValue);
    Serial.print(" ");
    Serial.print("desired_pos: ");
    Serial.print(desired_pos);
    Serial.print(" ");
    Serial.print("current_pos: ");
    Serial.print(current_pos);
    Serial.print(" ");
    Serial.print("num of readings: ");
    Serial.println(count_num);

    if (USE_ROS){
        int16_msg.data = current_pos;
        Linear_actuator_pos.publish( &int16_msg );
    }
}
