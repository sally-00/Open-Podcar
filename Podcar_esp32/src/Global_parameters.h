/* ------------------------------------------------
 * 
 * [Pin out]
 *                  ● 3.3V                GND ●
 *                  ● Enable               23 ● 
 *                  ● 36(VP,i)             22 ●  SCL_PIN
 *                  ● 39(VN,i)          1(TX) ●x
 *                  ● 34(AD,i)          3(RX) ●x
 *      read analog ● 35(AD,i)             21 ●  SDA_PIN
 *                  ● 32(AD4)             GND ●
 *                  ● 33(AD5)              19 ● 
 *                  ● 25(AD18)             18 ● 
 *      RPWM_Output ● 26(AD19)              5 ● 
 *      LPWM_Output ● 27(AD17)             17 ● 
 *                  ● 14(AD16)             16 ● 
 *                x ● 12(AD15)        4(AD10) ● 
 *                  ● GND             0(AD11) ● x
 *                  ● 13(AD14)        2(AD12) ● x
 *                  ● 9(D2)          15(AD13) ● x
 *                  ● 10(D3)            8(D1) ● x
 *                x ● 11(CMD)           7(D0) ● x
 *                  ● 5V              31(CLK) ●
 *                         [USB]
 * -------------------------------------------------
 */
# include<stdint.h>


// motor driver
#define RPWM_Output 26
#define LPWM_Output 27
// channel for PWM
#define Steering_CH_1 0
#define Steering_CH_2 1
// for setting frequency an resolution
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/ledc.html#_CPPv214ledc_timer_set11ledc_mode_t12ledc_timer_t8uint32_t8uint32_t14ledc_clk_src_t
// https://www.handsontec.com/dataspecs/module/BTS7960%20Motor%20Driver.pdf
#define freq 20000
#define resolution 8

// linear actuator position reading pin
#define position_pin 35

// This will change based on the resistor you use for the linear actuator
#define LA_LOWER_LIMIT 0
#define LA_UPPER_LIMIT 3680

// setup limits for DAC; speed control
uint16_t DACCentre                = 1679;  // new DAC value found in July 2019 -- Charles & Fanta 
uint16_t DAC_Upper_LIMIT          = 2519; // DO NOT CHANGE - COULD DAMAGE SCOOTER CIRCUIT
uint16_t DAC_Lower_LIMIT          = 1000; // DO NOT CHANGE - COULD DAMAGE SCOOTER CIRCUIT 
uint16_t DACFullScale             = 4095; // 12bit DAC

// setup pins for reading battery voltage
#define batt_vol 34
#define RATIO  1/11

// additional I2C for LCD display
#define SDA2 17
#define SCL2 16