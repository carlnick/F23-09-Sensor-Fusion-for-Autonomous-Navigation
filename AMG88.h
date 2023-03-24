/*
This is a library for the Panasonic Grid-EYE AMG88
* Murphy Smith 5/1/2021
*/

#include <stdio.h>
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <stdint.h>
#include <math.h>

#define DEFAULT_ADDRESS 0X69 //default address for AMG88

//Registers
#define POWER_CONTROL_REGISTER 			0x00
#define RESET_REISTER 					0x01
#define FRAMERATE_REGISTER 				0x02
#define INT_CONTROL_REGISTER 			0x03
#define STATUS_REGISTER					0x04
#define STATUS_CLEAR_REGISTER			0x05
#define AVERAGE_REGISTER				0x07
#define INT_LEVEL_REGISTER_UPPER_LSB	0x08
#define INT_LEVEL_REGISTER_UPPER_MSB	0x09
#define INT_LEVEL_REGISTER_LOWER_LSB	0x0A
#define INT_LEVEL_REGISTER_LOWER_MSB	0x0B
#define INT_LEVEL_REGISTER_HYST_LSB		0x0C
#define INT_LEVEL_REGISTER_HYST_MSB		0x0D
#define THERMISTOR_REGISTER_LSB			0x0E
#define THERMISTOR_REGISTER_MSB			0x0F
#define INT_TABLE_REGISTER_INTO			0x10
#define RESERVED_AVERAGE_REGISTER		0x1F
#define TEMPERATURE_REGISTER_START		0x80

#ifdef __cplusplus
extern "C" {
#endif
	extern float 	getPixelTemperature(int pixelAddr, int devID);
	extern int16_t getPixelTemperatureRaw(int pixelAddr, int devID);
	extern float 	getPixelTemperatureFahrenheit(int pixelAddr, int devID);
	
	extern float 	getDeviceTemperature();
	extern int16_t getDeviceTemperatureRaw();
	extern float	getDevicdeTemperatureFahrenheit();

	extern void 	setFrameRate1FPS();
	extern void	setFrameRate110FPS();
	extern _Bool 	isFramerate10FPS();
	
	extern void 	wake();
	extern void 	sleep();
	extern void 	standby60seconds();
	extern void 	standby10seconds();
	
	extern void 	interruptPinEnable();
	extern void 	interruptPinDisable();
	extern void 	setInterruptModeAbsolute();
	extern void 	setInterruptModeDifference();
	extern _Bool 	interruptPinEnabled();
	
	extern _Bool 	interruptFlagSet();
	extern _Bool 	pixelTemperatureOutputOK();
	extern _Bool 	deviceTemperatureOutputOK();
	extern void 	clearInterruptFlag();
	extern void 	clearPixelTemperatureOverlow();
	extern void 	clearDeviceTemperatureOverflow();
	extern void 	clearAllOverflow();
	extern void	clearAllStatusFlags();
	
	extern _Bool 	pixelInterruptSet(uint8_t pixelAddr);
	
	extern void 	movingAverageEnable();
	extern void 	movingAverageDisable();
	extern _Bool 	movingAverageEnabled();

	extern void 	setUpperInterruptValue(float DegreesC);
	extern void 	setUpperInterruptValueRaw(int16_t regValue);
	extern void	setUpperInterruptValueFahrenheit(float DegreesF);
	
	extern void 	setLowerInterruptValue(float DegreesC);
	extern void 	setLowerInterruptValueRaw(int16_t regValue);
	extern void 	setLowerInterruptValueFahrenheit(float DegreesF);
	
	extern void 	setInterruptHysteresis(float DegreesC);
	extern void 	setInterruptHysteresisRaw(int16_t regValue);
	extern void	setInterruptHysteresisFahrenheit(float DegreesF);
	
	extern float 	getUpperInterruptValue();
	extern int16_t getUpperInterruptValueRaw();
	extern float 	getUpperInterruptValueFahrenheit();
	
	extern float 	getLowerInterruptValue();
	extern int16_t	getLowerInterruptValueRaw();
	extern float 	getLowerInterruptValueFahrenheit();
	
	extern float	getInterruptHystersis();
	extern int16_t getInterruptHystersisRaw();
	extern float 	getInterruptHystersisFahrenheit();
	
	extern void 	setRegister(unsigned char reg, unsigned char val);
	extern int16_t getRegister(unsigned char reg, int8_t len);
	
	extern void 	setI2CAddress(uint8_t addr);
	

		
	extern uint8_t _deviceAddress;
	
#ifdef __cplusplus
}
#endif
