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
//DigitalOut rgb_Red(PH_0);
//DigitalOut rgb_Green(PH_1);
//DigitalOut rgb_Blue(PB_13);

void init_rgb(void){
	rgb_sensor.enablePowerAndRGBC();
	//Initialize the leds (1,3, and 4) of the board to zero.
    //rgb_Red    = 1; //shut down
	//rgb_Green  = 1; //shut down
	//rgb_Blue   = 1; //shut down
	//int a = rgb_sensor.getBlueData();
}

int *read_rgb(void){
    //init_rgb();
    static int rgb_readings[4]; // Declare a 4 element array to store RGB sensor readings

    rgb_sensor.getAllColors( rgb_readings ); // read the sensor to get red, green, and blue color data along with overall brightness
	//printf("COLOR SENSOR: clear: %d, red : %d,green : %d, blue: %d ",rgb_readings[0],rgb_readings[1],rgb_readings[2],rgb_readings[3]);
	//if(rgb_readings[1]>rgb_readings[2] && rgb_readings[1]>rgb_readings[3]){printf("--COLOUR DOMINANT: RED \n\n\r");}
	//if(rgb_readings[3]>rgb_readings[1] && rgb_readings[3]>rgb_readings[2]){printf("--COLOUR DOMINANT: BLUE \n\n\r");}
	//if(rgb_readings[2]>rgb_readings[1] && rgb_readings[2]>rgb_readings[3]){printf("--COLOUR DOMINANT: GREEN \n\n\r");}
    return rgb_readings;
}


//ACCELEROMETER
//int i,count_time_hour = 0;
//float maxX = -100,minX = 100,maxY = -100,minY=100,maxZ=-100,minZ=100;
//Timer t_hour;

/*float *read_accell(void){
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
}*/

/*void readallsensors (void) {
	// TEMP AND HUM SENSOR
	//******************************************
      readSensor();
      printf("TEMP: %d,%2d C ", _tData / 1000, _tData % 1000);
      printf("HUM: %d,%2d%%\r\n", _rhData / 1000, _rhData % 1000);
			
	//LIGHT SENSOR
	//*****************************************
			valor_light_sensor = 100*(lightSensor.read());
			printf("LIGHT: %.2f%% \n\r",valor_light_sensor);
		  
	//SOIL MOISTURE
			value_moisture = 100*moisture.read();
            printf("MOISTURE: %.2f%% \n\r", value_moisture);
			
	// ACCELEROMETER
			x=acc.getAccX();
			y=acc.getAccY();
			z=acc.getAccZ();
			printf("ACCELEROMETER: X_axis =%.4f  Y_axis=%.4f Z_axis=%.4f \n\r",x,y,z);
			
	// RGB SENSOR
			rgb_sensor.getAllColors( rgb_readings ); // read the sensor to get red, green, and blue color data along with overall brightness
			printf("COLOR SENSOR: clear: %d, red : %d,green : %d, blue: %d ",rgb_readings[0],rgb_readings[1],rgb_readings[2],rgb_readings[3]);
			if(rgb_readings[1]>rgb_readings[2] && rgb_readings[1]>rgb_readings[3]){printf("--COLOUR DOMINANT: RED \n\n\r");}
			if(rgb_readings[3]>rgb_readings[1] && rgb_readings[3]>rgb_readings[2]){printf("--COLOUR DOMINANT: BLUE \n\n\r");}
			if(rgb_readings[2]>rgb_readings[1] && rgb_readings[2]>rgb_readings[3]){printf("--COLOUR DOMINANT: GREEN \n\n\r");}
}*/
/*void limits (void){
	//LIGHT SENSOR
			if (valor_light_sensor > 30){
				//red							
				rgb_Red    = 0;
				rgb_Green  = 1;
				rgb_Blue   = 1;
			}
	//TEMP AND HUM
			if((_tData/1000)>40){
			//green							
				rgb_Red    = 1;
				rgb_Green  = 0;
				rgb_Blue   = 1;
			}
			if((_rhData / 1000)>90){
			//blue							
				rgb_Red    = 1;
				rgb_Green  = 1;
				rgb_Blue   = 0;
			}
	//SOIL MOISTURE
			if (value_moisture > 60){
				//red + green							
				rgb_Red    = 0;
				rgb_Green  = 0;
				rgb_Blue   = 1;
			}
	// ACCELEROMETER
			if ((x > 1)||(y > 1)||(z > 1)){
				//red							
				rgb_Red    = 1;
				rgb_Green  = 0;
				rgb_Blue   = 0;
			}
	// RGB SENSOR
			if(rgb_readings[1]>800 || rgb_readings[2]>800 || rgb_readings[3]>800){
				rgb_Red    = 0;
				rgb_Green  = 1;
				rgb_Blue   = 0;
			}
}*/


/*int main(void){
	//thread.start(limits_thread);
	//GPS SENSOR
	//***************
gps_Serial = new UnbufferedSerial(PA_9, PA_10,9600); //serial object for use w/ GPS
  Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
  char c; //when read via Adafruit_GPS::read(), the class returns single character stored here
  Timer refresh_Timer; //sets up a timer for use in loop; how often do we print GPS info?
  const int refresh_Time = 2000; //refresh time in ms

  myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
                        //a list of GPS commands is available at http://www.adafruit.com/datasheets/PMTK_A08.pdf

  myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
  myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  myGPS.sendCommand(PGCMD_ANTENNA);
  //printf("Connection established at 9600 baud...\r\n");
  //ThisThread::sleep_for(1s);
  refresh_Timer.start();  //starts the clock on the timer
	
		//RGB Sensor
			//Get ENABLE register	
			rgb_sensor.enablePowerAndRGBC();
			//Initialize the leds (1,3, and 4) of the board to zero.
		rgb_Red    = 1; //shut down
		rgb_Green  = 1; //shut down
		rgb_Blue   = 1; //shut down
		int a = rgb_sensor.getBlueData();
		
		//TEMP AND HUM SENSOR
			//Enable the sensor
			SENS_EN = 1;
    
			//Check if the sensor is present
			_tx_buf[0] = SI7013_READ_ID2_1;
			_tx_buf[1] = SI7013_READ_ID2_2;
    
			_address = SI7021_ADDR; //TODO: update if we use another sensor
    
			tempSensor.write(_address, (char*)_tx_buf, 2);
			tempSensor.read(_address, (char*)_rx_buf, 8);
			//Check ID byte
			if(_rx_buf[0] != SI7021_DEVICE_ID) {
        printf("No sensor present!\n\r");
        while(1);
			}
	
	while(true){
			swbutton.fall(button);
		
		if(test == true){
			printf("\nMODE TEST \n\r");
			normal_led = 0;
			test_led = 1;
			advanced_led = 0;
			button_state = false;**/
			//**************************
			//GPS
			//**************************
			
			/**c = myGPS.read();   //queries the GPS

			//if (c) { printf("%c", c); } //this line will echo the GPS data if not paused

      //check if we recieved a new message from GPS, if so, attempt to parse it,
        if ( myGPS.newNMEAreceived() ) {
            if ( !myGPS.parse(myGPS.lastNMEA()) ) {
                continue;
            }
        }
      //check if enough time has passed to warrant printing GPS info to screen
      //note if refresh_Time is too low or pc.baud is too low, GPS data may be lost during printing
      if (duration_cast<milliseconds>(refresh_Timer.elapsed_time()).count() >= refresh_Time) {
      //if (refresh_Timer.read_ms() >= refresh_Time) {
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
        }**/
			//***********************
			//READ ALL SENSORS
			//***********************
			/**readallsensors();
				
			//RGB LED FOR RGB SENSOR		
						if(rgb_readings[0]>30000)
						{
								rgb_Red    = 0;
								rgb_Green  = 0;
								rgb_Blue   = 0;		
						}
						else
							{
								if(rgb_readings[1]<500 && rgb_readings[2]<500 && rgb_readings[3]<500) //If low: none
									{
										rgb_Red    = 1;
										rgb_Green  = 1;
										rgb_Blue   = 1;
									}
									else
										{
										if(rgb_readings[1]>rgb_readings[2] && rgb_readings[1]>=rgb_readings[3]) //If max=RED
											{
												rgb_Red    = 0;
												rgb_Green  = 1;
												rgb_Blue   = 1;
											}else if(rgb_readings[2]>rgb_readings[1] && rgb_readings[2]>rgb_readings[3]) //If max=Green
											{
												rgb_Red    = 1;
												rgb_Green  = 0;
												rgb_Blue   = 1;
											}
											else if(rgb_readings[3]>rgb_readings[1] && rgb_readings[3]>rgb_readings[2])   //If max=Blue
											{
												rgb_Red    = 1;
												rgb_Green  = 1;
												rgb_Blue   = 0;						
											}
										}
							}
	
			ThisThread::sleep_for(2s);
		}
		
		if (normal == true){
			
			printf("\nMODE NORMAL \n\r");
			normal_led = 1;
			test_led = 0;
			advanced_led = 0;
			button_state = false;
			t_hour.start(); //begin the hour
			
			//shut down rgb lights
			rgb_Red    = 1;
			rgb_Green  = 1;
			rgb_Blue   = 1;	
		
			//GPS
			c = myGPS.read();   //queries the GPS

			//if (c) { printf("%c", c); } //this line will echo the GPS data if not paused

      //check if we recieved a new message from GPS, if so, attempt to parse it,
        if ( myGPS.newNMEAreceived() ) {
            if ( !myGPS.parse(myGPS.lastNMEA()) ) {
                continue;
            }
        }
      //check if enough time has passed to warrant printing GPS info to screen
      //note if refresh_Time is too low or pc.baud is too low, GPS data may be lost during printing
      if (duration_cast<milliseconds>(refresh_Timer.elapsed_time()).count() >= refresh_Time) {
      //if (refresh_Timer.read_ms() >= refresh_Time) {
      refresh_Timer.reset();
				printf("GPS: ");
				printf("Time: %d:%d:%d.%u ", myGPS.hour, myGPS.minute, myGPS.seconds, myGPS.milliseconds);
				printf("Date: %d/%d/20%d ", myGPS.day, myGPS.month, myGPS.year);
				printf("Q: %d ", (int) myGPS.fixquality);
      if ((int)myGPS.fixquality > 0) {
             printf("Loc: %5.2f %c, %5.2f %c ", myGPS.latitude, myGPS.lat, myGPS.longitude, myGPS.lon);
             printf("Speed: %5.2f knots ", myGPS.speed);
             //printf("Angle: %5.2f\r\n", myGPS.angle);
             printf("Alt: %5.2f ", myGPS.altitude);
             printf("Sat: %d\r\n", myGPS.satellites);
            }
        }**/
		  //***********************
			//READ ALL SENSORS
			//***********************
			//readallsensors();
			
			//****************************
			//MAX-MIN-MEAN FOR ALL SENSORS
			//****************************
			// TEMP AND HUM SENSOR		
			/**max_temp = Max(max_temp,_tData);
			min_temp = Min(min_temp,_tData);
			mean_temp = Measure(mean_temp,_tData);
			
			max_hum = Max(max_hum,_rhData);
			min_hum = Min(min_hum,_rhData);
			mean_hum = Measure(mean_hum,_rhData);	 			
			
			//MOISTURE
			maxmoisture = Max(maxmoisture,value_moisture);
			minmoisture = Min(minmoisture,value_moisture);
			meanmoisture = Measure(meanmoisture,value_moisture);
						
			//LIGHT SENSOR
			maxlight = Max(maxlight,valor_light_sensor);
			minlight = Min(minlight,valor_light_sensor);
			meanlight = Measure(meanlight,valor_light_sensor);
									
			//RGB SENSOR
											if(rgb_readings[1]>rgb_readings[2] && rgb_readings[1]>=rgb_readings[3]) //If max=RED
											{count_red++;}
											else if(rgb_readings[2]>rgb_readings[1] && rgb_readings[2]>rgb_readings[3]) //If max=Green
											{count_green++;}
											else if(rgb_readings[3]>rgb_readings[1] && rgb_readings[3]>rgb_readings[2])   //If max=Blue
											{count_blue++;}
			// ACCELEROMETER
					maxX=Max(maxX,x);
					maxY=Max(maxY,y);
					maxZ=Max(maxZ,z);
					minX=Min(minX,x);
					minY=Min(minY,y);
					minZ=Min(minZ,z);
			
			//LIMITS
			limits();
					
			ThisThread::sleep_for(5s); //monitored 30seg
					
			//print results every hour
			if(duration_cast<milliseconds>(t_hour.elapsed_time()).count() > 20000){
				onehour();
				t_hour.reset();
			}
		}
		if (advanced == true){
		printf("\nMODE ADVANCED \n\n\r");
			ThisThread::sleep_for(1s);
			normal_led = 0;
			test_led = 0;
			advanced_led = 1;
			button_state = false;
			
			rgb_Red    = 1;
			rgb_Green  = 1;
			rgb_Blue   = 1;**/
			
			//****************
			/*FALL DETECTED
			******************/
			//ACCELEROMETER
		    /**float x2=acc.getAccX();
			float y2=acc.getAccY();
			float z2=acc.getAccZ();
            
      //change to multiliner if warning by led needed
			free_fall(x2, y2, z2) == true ? printf("[WARNING!!!] Free fall detected!!!\n\r") : printf("ACCELEROMETER: X_axis =%.4f  Y_axis=%.4f Z_axis=%.4f \n\r",x2,y2,z2);
		}
		
		// button
				if (state1){		
					state1 = false;	
					if ((test == true)&& (button_state == false)){
							test = false;
							normal = true;
							advanced = false;
							button_state = true;
							printf("BUTTON - MODE NORMAL \n\r");
						}
					if ((normal == true)&& (button_state == false)){
							test = false;
							normal = false;
							advanced = true;
							button_state = true;
							printf("BUTTON - MODE ADVANCED \n\r");
						}
					if ((advanced == true)&& (button_state == false)){
							test = true;
							normal = false;
							advanced = false;
							button_state = true;
							printf("BUTTON - MODE TEST \n\r");
						}
				}
			
	}		
}**/

void sensors_init()
{
	tempHum_init();
	init_rgb();
}

void sensors_update(sensorData_t* data)
{
	TempHum_t th = tempHum_r();
	int* rgb = read_rgb();

	data->light = read_light();
	data->light = read_soil();
	data->temp = th.temp;
	data->hum = th.hum;
	data->r = rgb[0];
	data->g = rgb[1];
	data->b = rgb[2];
}