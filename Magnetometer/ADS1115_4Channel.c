// Distributed with a free-will license.
// Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
// ADS1115
// This code is designed to work with the ADS1115_I2CADC I2C Mini Module available from ControlEverything.com.
// https://www.controleverything.com/content/Analog-Digital-Converters?sku=ADS1115_I2CADC#tabs-0-product_tabset-2

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h>

#define CONVERSION_FACTOR 14.7/2

void main() 
{
	// Create I2C bus
	int file;
	int raw_adc;
	char *bus = "/dev/i2c-1";
	if ((file = open(bus, O_RDWR)) < 0) 
	{
		printf("Failed to open the bus. \n");
		exit(1);
	}
	// Get I2C device, ADS1115 I2C address is 0x48(72)
	ioctl(file, I2C_SLAVE, 0x48);
	
	// Create output txt file
	FILE *fp, *fp2;
	fp = fopen("magData.txt", "w");
	fclose(fp);
	
while (1)
{
	float x = 0;
	float y = 0;
	float z = 0;
	int mag = 0;
	// Select configuration register(0x01)
	// AINP = AIN0 and AINN = AIN1, +/- 2.048V
	// Continuous conversion mode, 128 SPS(0x84, 0x83)
	char config[3] = {0};
	config[0] = 0x01;
	config[1] = 0xC4;
	config[2] = 0x83;
	write(file, config, 3);
	sleep(1);
	
	fp2 = fopen("magData.txt", "a");

	// Read 2 bytes of data from register(0x00)
	// raw_adc msb, raw_adc lsb
	char reg[1] = {0x00};
	write(file, reg, 1);
	char data[2]={0};
	if(read(file, data, 2) != 2)
	{
		printf("Error : Input/Output Error \n");
	}
	else 
	{
		// Convert the data
		raw_adc = (data[0] * 256 + data[1]);
		if (raw_adc > 32767)
		{
			raw_adc -= 65535;
		}
		
		raw_adc = raw_adc * CONVERSION_FACTOR;
		x = raw_adc;
		// Output data to screen
		printf("X-axis magnetic field reading in nT: %d \n", raw_adc);
		fprintf(fp2, "X-axis magnetic field reading in nT: %d \n", raw_adc);
	}

	// Select configuration register(0x01)
	// AINP = AIN0 and AINN = AIN3, +/- 2.048V
	// Continuous conversion mode, 128 SPS(0x84, 0x83)
	//char config[3] = {0};
	config[0] = 0x01;
	config[1] = 0xD4;
	config[2] = 0x83;
	write(file, config, 3);
	sleep(1);

	// Read 2 bytes of data from register(0x00)
	// raw_adc msb, raw_adc lsb
	//char reg[1] = {0x00};
	write(file, reg, 1);
	//char data[2]={0};
	if(read(file, data, 2) != 2)
	{
		printf("Error : Input/Output Error \n");
	}
	else 
	{
		// Convert the data
		raw_adc = (data[0] * 256 + data[1]);
		if (raw_adc > 32767)
		{
			raw_adc -= 65535;
		}
		
		raw_adc = raw_adc * CONVERSION_FACTOR;
		y = raw_adc;
		// Output data to screen
		printf("Y-axis magnetic field reading in nT: %d \n", raw_adc);
		fprintf(fp2, "Y-axis magnetic field reading in nT: %d \n", raw_adc);
	}

	// Select configuration register(0x01)
	// AINP = AIN1 and AINN = AIN3, +/- 2.048V
	// Continuous conversion mode, 128 SPS(0x84, 0x83)
	//char config[3] = {0};
	config[0] = 0x01;
	config[1] = 0xE4;
	config[2] = 0x83;
	write(file, config, 3);
	sleep(1);

	// Read 2 bytes of data from register(0x00)
	// raw_adc msb, raw_adc lsb
	//char reg[1] = {0x00};
	write(file, reg, 1);
	//char data[2]={0};
	if(read(file, data, 2) != 2)
	{
		printf("Error : Input/Output Error \n");
	}
	else 
	{
		// Convert the data
		raw_adc = (data[0] * 256 + data[1]);
		if (raw_adc > 32767)
		{
			raw_adc -= 65535;
		}
		
		raw_adc = raw_adc * CONVERSION_FACTOR;
		z = raw_adc;
		// Output data to screen
		printf("Z-axis magnetic field reading in nT: %d \n", raw_adc);
		fprintf(fp2, "Z-axis magnetic field reading in nT: %d \n", raw_adc);
	}
	
	mag = sqrt(x*x+y*y+z*z);
	
	printf("Magnitude of magnetic field reading in nT: %d \n", mag);
	fprintf(fp2, "Magnetic of magnetic field reading in nT: %d \n", raw_adc);

	// Select configuration register(0x01)
	// AINP = AIN2 and AINN = AIN3, +/- 2.048V
	// Continuous conversion mode, 128 SPS(0x84, 0x83)
	//char config[3] = {0};
	config[0] = 0x01;
	config[1] = 0xF4;
	config[2] = 0x83;
	write(file, config, 3);
	sleep(1);

	// Read 2 bytes of data from register(0x00)
	// raw_adc msb, raw_adc lsb
	//char reg[1] = {0x00};
	write(file, reg, 1);
	//char data[2]={0};
	if(read(file, data, 2) != 2)
	{
		printf("Error : Input/Output Error \n");
	}
	else 
	{
		// Convert the data
		raw_adc = (data[0] * 256 + data[1]);
		if (raw_adc > 32767)
		{
			raw_adc -= 65535;
		}
		
		raw_adc = raw_adc * CONVERSION_FACTOR;
		// Output data to screen
		printf("GND reading converted to nT: %d \n\n", raw_adc);
		fprintf(fp2, "GND reading coverted to nT: %d \n\n", raw_adc);
	}
	
	fclose(fp2);
	/*
	double inner = x*x + y*y + z*z;
	double resultant = sqrt(inner);
	printf("Resultant magnetic field in nT: %d \n\n", resultant);
	*/
	
}
}
