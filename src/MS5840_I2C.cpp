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

#include <Wire.h> // Wire library is used for I2C
#include "MS5840_I2C.h"

MS5840::MS5840(ms5840_addr address)
// Base library type I2C
{
	_address = address; //set interface used for communication
}

void MS5840::reset(void)
// Reset device I2C
{
   sendCommand(CMD_RESET);
   sensorWait(3);
}

uint8_t MS5840::begin(void)
// Initialize library for subsequent pressure measurements
{
	uint8_t i;
	for(i = 0; i <= 7; i++)
  {
		sendCommand(CMD_PROM + (i * 2));
		Wire.requestFrom( _address, 2);
		uint8_t highByte = Wire.read();
		uint8_t lowByte = Wire.read();
		coefficient[i] = (highByte << 8)|lowByte;
	// Uncomment below for debugging output.
	//	Serial.print("C");
	//	Serial.print(i);
	//	Serial.print("= ");
	//	Serial.println(coefficient[i]);
	}

	return 0;
}

float MS5840::getTemperature(temperature_units units, precision _precision)
// Return a temperature reading in either F or C.
{
	getMeasurements(_precision);
	float temperature_reported;
	// If Fahrenheit is selected return the temperature converted to F
	if(units == FAHRENHEIT)
  {
		temperature_reported = _temperature_actual / 100.0f;
		temperature_reported = (((temperature_reported) * 9) / 5) + 32;
		return temperature_reported;
	}

	// If Celsius is selected return the temperature converted to C
	else
  {
		temperature_reported = _temperature_actual / 100.0f;
		return temperature_reported;
	}
}

float MS5840::getPressure(precision _precision)
// Return a pressure reading units Pa.
{
	getMeasurements(_precision);
	float pressure_reported;
	pressure_reported = _pressure_actual;
	return pressure_reported;
}

uint32_t MS5840::getPressureUncalib(precision _precision)
// Return uncalibrated pressure (fast read)
{
    uint32_t pressure_raw = getADCconversion(PRESSURE, _precision);
    return pressure_raw;
}

void MS5840::getMeasurements(precision _precision)

{
	//Retrieve ADC result
	int32_t temperature_raw = getADCconversion(TEMPERATURE, _precision);
	int32_t pressure_raw = getADCconversion(PRESSURE, _precision);


	//Create Variables for calculations
	int32_t temp_calc;
	int32_t pressure_calc;

	int32_t dT;

	//Now that we have a raw temperature, let's compute our actual.
	dT = temperature_raw - ((int32_t)coefficient[5] << 8);
	temp_calc = (((int64_t)dT * coefficient[6]) >> 23) + 2000;

	// TODO TESTING  _temperature_actual = temp_calc;

	//Now we have our first order Temperature, let's calculate the second order.
	int64_t TI, OFFI, SENSI, OFF, SENS, OFF2, SENS2; //working variables

	if (temp_calc > 2000)
	// If temp_calc is above 20.0C
	{
		TI = 0;
		OFFI = 0;
		SENSI = 0;

    }
	else
	// If temp_calc is below 20.0C
	{
        if (temp_calc > 1000)
        // If temp_calc is above 10.0C
        {
            TI = 12 * ((uint64_t)dT * dT)/pow(2,35);
    		OFFI = 30 * ((temp_calc - 2000) * (temp_calc - 2000)) / 256;
    		SENSI = 0;
        }
        else
        // If temp_calc is below 10.0C
        {
            TI = 14 * ((uint64_t)dT * dT)/pow(2,35);
    		OFFI = 35 * ((temp_calc - 2000) * (temp_calc - 2000)) / 8;
    		SENSI = 63 * ((temp_calc - 2000) * (temp_calc - 2000)) / 32;
        }

	}

	// Now bring it all together to apply offsets

	OFF = ((int64_t)coefficient[2] << 17) + (((coefficient[4] * (int64_t)dT)) >> 6);
	SENS = ((int64_t)coefficient[1] << 16) + (((coefficient[3] * (int64_t)dT)) >> 7);

	temp_calc = temp_calc - TI;
	OFF2 = OFF - OFFI;
	SENS2 = SENS - SENSI;

	// Now lets calculate the pressure


	pressure_calc = (((SENS2 * pressure_raw) / 2097152 ) - OFF2) / 32768;

	_temperature_actual = temp_calc ;
	_pressure_actual = pressure_calc ; // 10;// pressure_calc;


}

uint32_t MS5840::getADCconversion(measurement _measurement, precision _precision)
// Retrieve ADC measurement from the device.
// Select measurement type and precision
{
	uint32_t result;
	uint8_t highByte = 0, midByte = 0, lowByte = 0;

	sendCommand(CMD_ADC_CONV + _measurement + _precision);
	// Wait for conversion to complete
	switch( _precision )
	{
		case ADC_256 : sensorWait(700); break;
		case ADC_512 : sensorWait(1200); break;
		case ADC_1024: sensorWait(2300); break;
		case ADC_2048: sensorWait(4500); break;
		case ADC_4096: sensorWait(8800); break;
	}

	sendCommand(CMD_ADC_READ);
	Wire.requestFrom(_address, 3);

	while(Wire.available())
	{
		highByte = Wire.read();
		midByte = Wire.read();
		lowByte = Wire.read();
	}

	result = ((uint32_t)highByte << 16) + ((uint32_t)midByte << 8) + lowByte;

	return result;

}

void MS5840::sendCommand(uint8_t command)
{
	Wire.beginTransmission( _address);
	Wire.write(command);
	Wire.endTransmission();

}

void MS5840::sensorWait(uint16_t time)
// Delay function.  This can be modified to work outside of Arduino based MCU's
{
	delayMicroseconds(time);
}
