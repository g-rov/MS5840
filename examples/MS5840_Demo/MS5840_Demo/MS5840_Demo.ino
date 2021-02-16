/******************************************************************************
MS5840_I2C.h
Library for MS5840 pressure sensors.
Author: g-rov
https://github.com/g-rov/MS5840

The MS58XX MS57XX and MS56XX by Measurement Specialties is a low cost I2C pressure
sensor.  This sensor can be used in weather stations and for altitude
estimations. It can also be used underwater for water depth measurements.

In this file are the functions in the MS5840 class.

Resources:
This library uses the Arduino Wire.h to complete I2C transactions.

Development environment specifics:
	IDE: Arduino 1.8.13
	Hardware Platform: Arduino Mega 2560
	Sensor version: MS5840-02BA


Distributed as-is; no warranty is given.
******************************************************************************/

#include <MS5840_I2C.h>
#include <Wire.h>

MS5840 sensor(ADDRESS_HIGH);

//Create variables to store results
float temperature_c;
double pressure_abs;
unsigned int pressure_un;


void setup() {
  Wire.begin();
  Serial.begin(9600);
  sensor.reset();
  sensor.begin();
}

void loop() {

  // To measure to higher degrees of precision use the following sensor settings:
  // ADC_256
  // ADC_512
  // ADC_1024
  // ADC_2048
  // ADC_4096

  // Read temperature from the sensor in deg C. This operation takes about
  //temperature_c = sensor.getTemperature(CELSIUS, ADC_512);

  // Read pressure from the sensor in Pa.
  pressure_abs = sensor.getPressure(ADC_256);
  Serial.println(pressure_abs);

  // Read pressure from the sensor uncalibrated. Much faster read! ** Not stable, ADC creates steps
  //pressure_un = sensor.getPressureUncalib(ADC_256);
  //Serial.println(pressure_un);

  // Stability delay. * Not required, library handles all the necessary delays between measurements.
  delayMicroseconds(50);
}
