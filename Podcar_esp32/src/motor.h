#include <Arduino.h>
#include "Global_parameters.h"


void set_speed(int speed, bool direction)
{
    // Make sure the speed is within the limit.
    if (speed > 255) {
        speed = 255;
    }
    
    // Set the speed and direction.
    if (direction) {
        //Serial.print("enlong");
        ledcWrite(Steering_CH_1, 0);
        ledcWrite(Steering_CH_2, speed);
    } else {
        //Serial.print("get short");
        ledcWrite(Steering_CH_1, speed);
        ledcWrite(Steering_CH_2, 0);
    }

}

void brake()
{
    ledcWrite(Steering_CH_1, 0);
    ledcWrite(Steering_CH_2, 0);
    // do not give zero, Low means below the voltage threshold that is considered a LOW
    //digitalWrite(RPWM_Output, LOW);
    //digitalWrite(LPWM_Output, LOW);
}