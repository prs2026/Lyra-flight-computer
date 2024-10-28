#if !defined(OS)
#include <Arduino.h>
#include <Wire.h>

#define OS
//pin buzzer is wired to
#define BUZZERPIN 0
//pin led is connected to
#define LEDPIN 7

//I2C pins
#define SDA 4
#define SCL 5

//Pyro channel 4 pins
#define P4EN 8
#define P4CONT 11
//Pyro channel 3 pins
#define P3EN 9
#define P3CONT 13
//Pyro channel 2 pins
#define P2EN 10
#define P2CONT 14
//Pyro channel 1 pins
#define P1EN 15
#define P1CONT 12

//Breakout pin definetion 
#define BRKOUT1 25
#define BRKOUT2 27
#define BRKOUT3 28
#define BRKOUT4 29

//battery sense pin
#define BATTSENSE 26

//Sets up pinModes and pins.
int setuppins();

//Beep for duration ms at freq hz
int beep(uint32_t duration=200,uint16_t freq=4000);

//Set the LED to a state
int setled(int state);

//scans the i2c bus, 
int scani23c(bool printout);

#endif // OS