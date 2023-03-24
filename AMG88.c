/*
This is a library for the Panasonic Grid-EYE AMG88
* Murphy Smith 5/1/2021
* 
* To Compile:
* gcc -c -g AMG88.c //this will create object file
* gcc -c -g SunSensor.c //creates object file
* gcc -o SS SunSensor.o AMG88.o -lwiringPi -lm //links the two libraries to the code and creates an executable (plus math.h)
*/
#include <stdio.h>
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <stdint.h>
#include <math.h>

#include "AMG88.h"

float 	getPixelTemperature(int pixelAddr, int devID)
{
	//Temperature registers are numbered 128-255
	//Each pixel has a lower and higher register
	int16_t temperature = getPixelTemperatureRaw(pixelAddr, devID);
	//temperature is reported as 12-bit 2's complement
	//check to see if temperature is negative
	if(temperature & (1 << 11))
	{
		//if temp is negative mask out sign byte
		//and make the float negative
		temperature &= ~(1 << 11);
		temperature = temperature * -1;
	}
	float DegreesC = temperature * 0.25;
	return DegreesC;
}
int16_t getPixelTemperatureRaw(int pixelAddr, int devID)
{
	//Temperature registers are numbered 128-255
	//Each pixel has a lower and higher register
	// pixelAddr = 0; pixelAddr < 64; pixelAddr++ to cycle all of the pixels
	int pixelLowRegister = TEMPERATURE_REGISTER_START + (2*pixelAddr);
	int16_t temperature = wiringPiI2CReadReg16(devID, pixelLowRegister);
	return temperature;
}
float 	getPixelTemperatureFahrenheit(int pixelAddr, int devID)
{
	int16_t temperature = getPixelTemperature(pixelAddr, devID);
	float DegreesF = temperature *1.8 + 32;
	return DegreesF;
}


float 	getDeviceTemperature()
{
	return 0;
}
int16_t getDeviceTemperatureRaw()
{
	return 0;
}
float	getDevicdeTemperatureFahrenheit()
{
	return 0;
}

	
void 	setFrameRate1FPS()
{
	
}
void	setFrameRate110FPS()
{
	
}
_Bool 	isFramerate10FPS()
{
	return 0;
}
		
		
void 	wake()
{
	
}
void 	sleep()
{
	
}
void 	standby60seconds()
{
	
}
void 	standby10seconds()
{
	
}

		
void 	interruptPinEnable()
{
	
}
void 	interruptPinDisable()
{
	
}
void 	setInterruptModeAbsolute()
{
	
}
void 	setInterruptModeDifference()
{
	
}
_Bool 	interruptPinEnabled()
{
	return 0;
}

		
_Bool 	interruptFlagSet()
{
	return 0;
}
_Bool 	pixelTemperatureOutputOK()
{
	return 0;
}
_Bool 	deviceTemperatureOutputOK()
{
	return 0;
}
void 	clearInterruptFlag()
{
	
}
void 	clearPixelTemperatureOverlow()
{
	
}
void 	clearDeviceTemperatureOverflow()
{
	
}
void 	clearAllOverflow()
{
	
}
void	clearAllStatusFlags()
{
	
}

	
_Bool 	pixelInterruptSet(u_int8_t pixelAddr)
{
	return 0;
}
		
void 	movingAverageEnable()
{
	
}
void 	movingAverageDisable()
{
	
}
_Bool 	movingAverageEnabled()
{
	return 0;
}

		
void 	setUpperInterruptValue(float DegreesC)
{
	
}
void 	setUpperInterruptValueRaw(int16_t regValue)
{
	
}
void	setUpperInterruptValueFahrenheit(float DegreesF)
{
	
}

		
void 	setLowerInterruptValue(float DegreesC)
{
	
}
void 	setLowerInterruptValueRaw(int16_t regValue)
{
	
}
void 	setLowerInterruptValueFahrenheit(float DegreesF)
{
	
}

		
void 	setInterruptHysteresis(float DegreesC)
{
	
}
void 	setInterruptHysteresisRaw(int16_t regValue)
{
	
}
void	setInterruptHysteresisFahrenheit(float DegreesF)
{
	
}

		
float 	getUpperInterruptValue()
{
	return 0;
}
int16_t getUpperInterruptValueRaw()
{
	return 0;
}
float 	getUpperInterruptValueFahrenheit()
{
	return 0;
}

		
float 	getLowerInterruptValue()
{
	return 0;
}
int16_t	getLowerInterruptValueRaw()
{
	return 0;
}
float 	getLowerInterruptValueFahrenheit()
{
	return 0;
}

	
float	getInterruptHystersis()
{
	return 0;
}
int16_t getInterruptHystersisRaw()
{
	return 0;
}
float 	getInterruptHystersisFahrenheit()
{
	return 0;
}

	
void 	setRegister(unsigned char reg, unsigned char val)
{
	
}
int16_t getRegister(unsigned char reg, int8_t len)
{
	return 0;
}
		
		
void 	setI2CAddress(u_int8_t addr)
{
	
}
