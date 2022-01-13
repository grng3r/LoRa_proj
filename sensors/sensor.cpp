/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "extras.h"
#include "./MMA8451Q.h"
#include "TCS3472_I2C.h"
#include "MBed_Adafruit_GPS.h"
#include "RGB.h"
#include "sensor.hpp"

//using namespace std::chrono;
//DigitalOut test_led(LED1);
//DigitalOut normal_led(LED2);
//DigitalOut advanced_led(LED3);


/*******************************************************************************
 *******************************  Variables  ***********************************
 ******************************************************************************/

//I2C-bus with RHT sensor (Si7021) on Happy Gecko STK
I2C tempSensor(D14, D15);
//And enable line for the sensor
DigitalOut SENS_EN(D13);

uint8_t  _address = 0;
uint8_t  _rx_buf[8] = {0};
uint8_t  _tx_buf[2] = {0};
 

void tempHum_init(void){
    //Enable the sensor
	SENS_EN = 1;
    
	//Check if the sensor is present
	_tx_buf[0] = SI7013_READ_ID2_1;
	_tx_buf[1] = SI7013_READ_ID2_2;
    
	_address = SI7021_ADDR;    
	tempSensor.write(_address, (char*)_tx_buf, 2);
	tempSensor.read(_address, (char*)_rx_buf, 8);
}

typedef struct TempHum_t{
    int temp;
    unsigned int hum;
};


TempHum_t tempHum_r()
 {
	TempHum_t th;
    th.temp = 0;
    th.hum = 0;
    
    //send humidity command
    _tx_buf[0] = SI7013_READ_RH;
    tempSensor.write(_address, (char*)_tx_buf, 1);
    tempSensor.read(_address, (char*)_rx_buf, 2);
    
    //Store raw RH info 
    th.hum = ((uint32_t)_rx_buf[0] << 8) + (_rx_buf[1] & 0xFC);
    //Convert value to milli-percent 
    th.hum = (((th.hum) * 15625L) >> 13) - 6000;
    
    //send temperature command
    _tx_buf[0] = SI7013_READ_TEMP;
    tempSensor.write(_address, (char*)_tx_buf, 1);
    tempSensor.read(_address, (char*)_rx_buf, 2);
    
    //Store raw temperature info
    th.temp = ((uint32_t)_rx_buf[0] << 8) + (_rx_buf[1] & 0xFC);
    //Convert to milli-degC
    th.temp = (((th.temp) * 21965L) >> 13) - 46850;
    
    //buf[0] = temp;
    //buf[1] = humidity;
    return th;
}


/*
//GPS SENSOR
using namespace std::chrono;
UnbufferedSerial * gps_Serial;
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
char c; //when read via Adafruit_GPS::read(), the class returns single character stored here

void init_GPS(void){
    Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?
    const int refresh_Time = 2000; //refresh time in ms

    myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
                        //a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf

    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);

    refresh_Timer.start();
    printf("Setting up GPS\n");
}*/

/*
float *read_GPS(void){	
    c = myGPS.read();   //queries the GPS
    float location[2];

    //check if we recieved a new message from GPS, if so, attempt to parse it,
    if ( myGPS.newNMEAreceived() ) {
        if ( !myGPS.parse(myGPS.lastNMEA()) ) {
            //continue;
            location[0] = myGPS.latitude;
            location[1] = myGPS.longitude;
        }
    } else{
        //printf("No GPS signal returned");
        location[0] = 40.4637;
        location[1] = 3.7492;
    }

    return location;*/
   /* //check if enough time has passed to warrant printing GPS info to screen
    //note if refresh_Time is too low or pc.baud is too low, GPS data may be lost during printing
    if (duration_cast<milliseconds>(refresh_Timer.elapsed_time()).count() >= refresh_Time) {
        refresh_Timer.reset();
		printf("GPS: ");
		printf("Time: %d:%d:%d.%u ", myGPS.hour, myGPS.minute, myGPS.seconds, myGPS.milliseconds);
		printf("Date: %d/%d/20%d ", myGPS.day, myGPS.month, myGPS.year);
		printf("Q: %d ", (int) myGPS.fixquality);
        if ((int)myGPS.fixquality > 0) {
            printf("Loc: %5.2f %c, %5.2f %c ", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
            printf("Speed: %5.2f knots ", myGPS.speed);
            printf("Alt: %5.2f ", myGPS.altitude);
            printf("Sat: %d\r\n", myGPS.satellites);
        }
    }*/
//}


float read_light(void){
    AnalogIn lightSensor(PA_4);

    float volatile valor_light_sensor = 100*(lightSensor.read());
    //printf("LIGHT: %.2f%% \n\r",valor_light_sensor);
    return valor_light_sensor;
}


float read_soil(void){
    AnalogIn moisture(PA_0);

    float volatile value_moisture = 100*moisture.read();
    //printf("MOISTURE: %.2f%% \n\r", value_moisture);
    return value_moisture;
}



//RGB
TCS3472_I2C rgb_sensor (PB_9, PB_8);
//Show the sensed colour
DigitalOut rgb_Red(PH_0);
DigitalOut rgb_Green(PH_1);
DigitalOut rgb_Blue(PB_13);

void init_rgb(void){
	rgb_sensor.enablePowerAndRGBC();
	//Initialize the leds (1,3, and 4) of the board to zero.
    rgb_Red    = 1; //shut down
	rgb_Green  = 1; //shut down
	rgb_Blue   = 1; //shut down
	int a = rgb_sensor.getBlueData();
}

char read_rgb(void){
    //init_rgb();
    static int rgb_readings[4]; // Declare a 4 element array to store RGB sensor readings

    rgb_sensor.getAllColors( rgb_readings ); // read the sensor to get red, green, and blue color data along with overall brightness
	//printf("COLOR SENSOR: clear: %d, red : %d,green : %d, blue: %d ",rgb_readings[0],rgb_readings[1],rgb_readings[2],rgb_readings[3]);
	if(rgb_readings[1]>rgb_readings[2] && rgb_readings[1]>rgb_readings[3]){
        return 'R';
        } else if(rgb_readings[3]>rgb_readings[1] && rgb_readings[3]>rgb_readings[2]){
            return 'B';
        }else//(rgb_readings[2]>rgb_readings[1] && rgb_readings[2]>rgb_readings[3])
        {
            return 'G';
            }
}


//ACCELEROMETER
//int i,count_time_hour = 0;
//float maxX = -100,minX = 100,maxY = -100,minY=100,maxZ=-100,minZ=100;
//Timer t_hour;

float *read_accell(void){
    float x,y,z;
    MMA8451Q acc(PB_9,PB_8,0x1d<<1);

    float vals[3];
	x=acc.getAccX();
	y=acc.getAccY();
	z=acc.getAccZ();
	printf("ACCELEROMETER: X_axis =%.4f  Y_axis=%.4f Z_axis=%.4f \n\r",x,y,z);
    vals[0] = x;
    vals[1] = y;
    vals[2] = z;

    return vals;
}


void sensors_init()
{
	tempHum_init();
	init_rgb();
}

void sensors_update(sensorData_t* data)
{
	TempHum_t th = tempHum_r();
	char rgb = read_rgb();
    float *acc =  read_accell();
    data->lat = 40.4168;//4
    data-> lon = 3.7038;//4
	data->light = read_light();//4
	data->moist = read_soil();//4
	data->temp = th.temp;//4
	data->hum = th.hum;//4
	data->rgb = rgb;//1
    data->acc_x = acc[0];//4
}