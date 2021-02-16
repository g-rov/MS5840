/******************************************************************************
MS5840_I2C.h
Library for MS5840 pressure sensors.
Author: g-rov
https://github.com/g-rov/MS5840

The MS58XX MS57XX and MS56XX by Measurement Specialties is a low cost I2C pressure
sensor.  This sensor can be used in weather stations and for altitude
estimations. It can also be used underwater for water depth measurements.

In this file are the function prototypes in the MS5840 class

Resources:
This library uses the Arduino Wire.h to complete I2C transactions.

Development environment specifics:
	IDE: Arduino 1.8.13
	Hardware Platform: Arduino Mega 2560
	Sensor version: MS5840-02BA


Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef MS5840_I2C_h
#define MS5840_I2C_h

#include <Arduino.h>

// Define units for conversions.
enum temperature_units
{
	CELSIUS,
	FAHRENHEIT,
};

// Define measurement type.
enum measurement
{
	PRESSURE = 0x00,
	TEMPERATURE = 0x10
};

// Define constants for Conversion precision
enum precision
{
	ADC_256  = 0x00,
	ADC_512  = 0x02,
	ADC_1024 = 0x04,
	ADC_2048 = 0x06,
	ADC_4096 = 0x08
};

// Define address choices for the device (I2C mode)
enum ms5840_addr
{
	ADDRESS_HIGH = 0x76,
	ADDRESS_LOW  = 0x77  // Not used by this sensor
};

//Commands
#define CMD_RESET 0x1E // reset command
#define CMD_ADC_READ 0x00 // ADC read command
#define CMD_ADC_CONV 0x40 // ADC conversion command

#define CMD_PROM 0xA0 // Coefficient location


class MS5840
{
	public:
		MS5840(ms5840_addr address);
		void reset(void);	 //Reset device
		uint8_t begin(void); // Collect coefficients from sensor

		// Return calculated temperature from sensor
		float getTemperature(temperature_units units, precision _precision);
		// Return calculated pressure from sensor
		float getPressure(precision _precision);
        // Return uncalibrated pressure from sensor (fast read)
        uint32_t getPressureUncalib(precision _precision);

	private:

		int32_t _temperature_actual;
		int32_t _pressure_actual;

		ms5840_addr _address; 		// Variable used to store I2C device address.
		uint16_t coefficient[8];// Coefficients;

		void getMeasurements(precision _precision);

		void sendCommand(uint8_t command);	// General I2C send command function
		uint32_t getADCconversion(measurement _measurement, precision _precision);	// Retrieve ADC result

		void sensorWait(uint16_t time); // General delay function see: delay()
};

#endif
