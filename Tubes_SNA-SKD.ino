//DEFINE LIBRARIES
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

//DEFINE SENSOR ID TO USE
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//INITIALIZE SENSOR VARIABLES
sensor_event_t mag_value; 

//USES LDR AS LIGHT DETECTION
#define LDR_NORTH A0
#define LDR_EAST  A1
#define LDR_SOUTH A2
#define LDR_WEST  A3

//ENABLING MOTOR INPUT FOR A
#define ENA 22
#define IN1 34
#define IN2 35

//ENABLING MOTOR INPUT FOR B
#define ENB 21
#define IN3 23
#define IN4 24

//DEFINING SPI PINS
#define MOSI 23
#define SCK 18
#define NCS 5
//#define MISO 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  if(!mag.begin()){
    Serial.println("Failed to detect the sensor HMC5883...");
    while(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  mag.getEvent(&mag_value);
  
}
