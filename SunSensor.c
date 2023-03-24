/* Author Jeremy Bruce
* Some code adapted from previous group's code, author Murphy Smith
* Five ea. Sun Sensor interfacing with full CubeSat Navigation system
**/

#include <stdio.h>
#include <wiringPiI2C.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <math.h>
#include "AMG88.h"
#include <time.h>
#include <dos.h>

#define MINUTES 7
#define SECONDS_PER_MINUTE 60
#define s1_address 0x69 // SS 1
#define s2_address 0x68 // SS 2
#define s3_address 0x69 // SS 3
#define s4_address 0x68 // SS 4
#define s5_address 0x69 // SS 5
#define N 1 //number of sensors

int sensorID[N];

void init()
{
	sensorID[0] = wiringPiI2CSetupInterface("/dev/i2c-2", s1_address);
	sensorID[1] = wiringPiI2CSetupInterface("/dev/i2c-2", s2_address);
	sensorID[2] = wiringPiI2CSetupInterface("/dev/i2c-3", s3_address);
	sensorID[3] = wiringPiI2CSetupInterface("/dev/i2c-3", s4_address);
	sensorID[4] = wiringPiI2CSetupInterface("/dev/i2c-1", s5_address);
}
int main()
{
	init();
	time_t start = time(NULL);
	int pixelArrayAverage[8][8][N];
	int pixelLocationCounter = 0;
	double mean[N];
	double sum = 0;
	double standardDeviation[N];
	
	while (1) {
		
	for(unsigned char y = 0; y<8; y++){
		for(unsigned char x = 0; x<8; x++){
			pixelArrayAverage[x][y][0] = (getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[0])+getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[0])+getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[0]))/3;
			pixelArrayAverage[x][y][1] = (getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[1])+getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[1])+getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[1]))/3;
			pixelArrayAverage[x][y][2] = (getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[2])+getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[2])+getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[2]))/3;
			pixelArrayAverage[x][y][3] = (getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[3])+getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[3])+getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[3]))/3;
			pixelArrayAverage[x][y][4] = (getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[4])+getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[4])+getPixelTemperatureFahrenheit(pixelLocationCounter, sensorID[4]))/3;
			pixelLocationCounter++;
		}
	}
	for(int i = 0; i < N; i++){
		mean[i] = 0;
		sum = 0;
		for(int x = 0; x<8; x++){
			for(int y=7; y>=0; y--){
				printf("%d ",pixelArrayAverage[x][y][i]);
			}
			printf("\n");
		}
	
		for(unsigned char y = 0; y<8; y++){ //find the mean of the data
			for(unsigned char x = 0; x<8; x++){
				mean[i] = mean[i] + pixelArrayAverage[x][y][i];
			}
		} mean[i] = mean[i]/64;	
	
		for(unsigned char y = 0; y<8; y++){ 
			for(unsigned char x = 0; x<8; x++){
				sum = ((pixelArrayAverage[x][y][i]-mean[i])*(pixelArrayAverage[x][y][i]-mean[i]))+sum;
			}
		} sum = sum/64;
	
		standardDeviation[i] = sqrt(sum);
		printf("Standard Deviation = %f\nMean = %f\n",standardDeviation[i], mean[i]);
		
		double threshold;
		threshold = 5.8*standardDeviation[i]+mean[i];
		printf("Threshold Value = %f\n",threshold);
		for(unsigned char y = 0; y<8; y++){ //If pixel is above 6SD away from mean
			for(unsigned char x = 0; x<8; x++){
				double temp;
				temp = (pixelArrayAverage[x][y][i]-mean[i])/standardDeviation[i];
				if(temp > 5.8){
					printf("Pixel Location x:%d y:%d value: %d \n",x, y, pixelArrayAverage[x][y][i]);
				}
			}
		}
	}
	delay(500);
}
	
}
