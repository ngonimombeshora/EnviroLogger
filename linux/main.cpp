/*
 * EnviroLogger.cpp
 * 
 * Originall written by NGONI MOMBESHORA, TAPIWA MATARE
 * 
 * Adapted for EEE3096S 2019 by Keegan Crankshaw
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "EnviroLogger.h"
#include <wiringPiSPI.h>
#include "CurrentTime.h"

//---------------------------------------------------------
//#define BLYNK_DEBUG
#define BLYNK_PRINT stdout
#ifdef RASPBERRY
  #include <BlynkApiWiringPi.h>
#else
  #include <BlynkApiLinux.h>
#endif
#include <BlynkSocket.h>
#include <BlynkOptionsParser.h>
//-----------------------------------------------------------
static BlynkTransportSocket _blynkTransport;
BlynkSocket Blynk(_blynkTransport);
 
static const char *auth, *serv;
static uint16_t port;
 
#include <BlynkWidgets.h>
 //global variables
long lastInterruptTime = 0;
int RTC; //Holds the RTC instance
int TEMP_READING;
int HUMIDITY_READING;
int LDR_READING;
bool warning_flag = false;
//------------------------------------------------------
int buttonPin = 17; //GPIO17 Pin on the Pi // to seee how buttons are labelled

 void setup()
{
    Blynk.begin(auth, serv, port);
    pinMode(buttonPin, INPUT); //Set GPIO17 as input
    pullUpDnControl (buttonPin, PUD_UP); //Set GPIO17 internal pull up
}
//------------------------------------------------------------
void loop()
{
	warning_checker(); //validate reading before sending to blynk app
    Blynk.run();
	 Blynk.virtualWrite(V1, HUMIDITY_READING); 
	 Blynk.virtualWrite(V2, TEMP_READING); 
	 Blynk.virtualWrite(V3, LDR_READING); 
	if (warning_flag){
		warning_light();
	}

}
//-------------------------------------------------------------------



// Configure interrupts here. NB:For button interupts____with DEBOUNCING
//...DONE WITH FILLING IN LOGIC!
void change_time_interval(void){
    //debouncing
    long interruptTime = millis();
	if (interruptTime - lastInterruptTime>200){
        //Write your logis here
       
    }
	lastInterruptTime = interruptTime;
}

void reset_system_time(void){
    //debouncing
    long interruptTime = millis();
	if (interruptTime - lastInterruptTime>200){
        //Write your logis here
       
    }
	lastInterruptTime = interruptTime;
}

void dismiss_alarm(void){
    //debouncing
    long interruptTime = millis();
	if (interruptTime - lastInterruptTime>200){
        //Write your logis here
       
    }
	lastInterruptTime = interruptTime;
}
void start_stop_monitoring(void){
    //debouncing
    long interruptTime = millis();
	if (interruptTime - lastInterruptTime>200){
        //Write your logis here
       
    }
	lastInterruptTime = interruptTime;
}

int setup_gpio(void){
    printf("Setting up gpio...\n");
    //Set up wiring Pi
    wiringPiSetup();

    //setting up the buttons
	pinMode(CHANGE_TIME_INTERVAL, INPUT);           //btn 1
	pullUpDnControl(CHANGE_TIME_INTERVAL, PUD_UP);

    pinMode(RESET_STYSTEM_TIME, INPUT);             //btn2
	pullUpDnControl(RESET_STYSTEM_TIME, PUD_UP);

    pinMode(DISMISS_ALARM, INPUT);             //btn3
	pullUpDnControl(DISMISS_ALARM, PUD_UP);


    pinMode(START_STOP_MONITORING, INPUT);             //btn4
	pullUpDnControl(START_STOP_MONITORING, PUD_UP);

    //LEDS
     pinMode(ALARM_LED, OUTPUT); 

    printf("BTNS done...\n");
    printf("Attaching INTERUPTS done...\n");
    //Attach interrupts to Buttons	
    wiringPiISR (CHANGE_TIME_INTERVAL, INT_EDGE_FALLING,change_time_interval) ;
    wiringPiISR (RESET_STYSTEM_TIME, INT_EDGE_FALLING,reset_system_time) ;
    wiringPiISR (DISMISS_ALARM, INT_EDGE_FALLING,dismiss_alarm) ;
    wiringPiISR (START_STOP_MONITORING, INT_EDGE_FALLING,start_stop_monitoring) ;
	printf("Attaching INTERUPTS done...\n");

	printf(" Setting up SPI ...\n");
    //setting up the SPI interface
    wiringPiSPISetup (SPI_CHAN, SPI_SPEED) ;
    printf(" Setting up SPI done ...\n");
	printf("Setup done\n");
    return 0;
}

//THREAD STUFF REMOVED..Using mcp3008_read, hopefully it works. It works!!!

int main(int argc, char* argv[])
{
  //----------------------------------------------
	parse_options(argc, argv, auth, serv, port);
	
	//calling blynk and gpio setup
    setup();
	if(setup_gpio()==-1){
        return 0;
    }
	
    //WRITE MAIN HERE!!!
	for(;;){
	mcp3008_read(analog_ldr_pin);
	delay(50);
	mcp3008_read(analog_humidity_pin);
	delay(50);
	mcp3008_read(analog_temp_pin);
	delay(50);
	printf("|Humidity%d, |Temprature%d, |Light%d|\n", HUMIDITY_READING,TEMP_READING,LDR_READING);
	delay(100);
	loop(); //for sending data to blynk
	}
  return 0;
}
//to be used for RTC
//hex to dec...for RTC reading and writing
int hexCompensation(int units){
	/*Convert HEX or BCD value to DEC where 0x45 == 0d45 
	  This was created as the lighXXX functions which determine what GPIO pin to set HIGH/LOW
	  perform operations which work in base10 and not base16 (incorrect logic) 
	*/
	int unitsU = units%0x10;

	if (units >= 0x50){
		units = 50 + unitsU;
	}
	else if (units >= 0x40){
		units = 40 + unitsU;
	}
	else if (units >= 0x30){
		units = 30 + unitsU;
	}
	else if (units >= 0x20){
		units = 20 + unitsU;
	}
	else if (units >= 0x10){
		units = 10 + unitsU;
	}
	return units;
}

//This function "undoes" hexCompensation in order to write the correct base 16 value through to I2C
int decCompensation(int units){
	int unitsU = units%10;

	if (units >= 50){
		units = 0x50 + unitsU;
	}
	else if (units >= 40){
		units = 0x40 + unitsU;
	}
	else if (units >= 30){
		units = 0x30 + unitsU;
	}
	else if (units >= 20){
		units = 0x20 + unitsU;
	}
	else if (units >= 10){
		units = 0x10 + unitsU;
	}
	return units;
}

void mcp3008_read(uint8_t adcnum) //threading? or is it buffering?
{ 
    unsigned int commandout = 0;
    unsigned int adcout = 0;

    if(adcnum==4){  //the value read from the temperature sensorneeds  to  be  converted  to  degrees  Celsius
		commandout = 0x4;
		commandout |= 0x18;     // start bit + single-ended bit
	uint8_t spibuf[3];

    spibuf[0] = commandout;
    spibuf[1] = 0;
    spibuf[2] = 0;

    wiringPiSPIDataRW(0, spibuf, 3);    

    adcout = ((spibuf[1] << 8) | (spibuf[2])) >> 4;
	TEMP_READING = adcout;
	}

	else if(adcnum==2){
	commandout = adcnum & 0x3;  // only 0-7
    commandout |= 0x18;     // start bit + single-ended bit
	uint8_t spibuf[3];

    spibuf[0] = commandout;
    spibuf[1] = 0;
    spibuf[2] = 0;

    wiringPiSPIDataRW(0, spibuf, 3);    

    adcout = ((spibuf[1] << 8) | (spibuf[2])) >> 4;
	HUMIDITY_READING = adcout;
	}
	else if(adcnum==1){
	unsigned int commandout = 0;
    unsigned int adcout = 0;
	commandout = adcnum & 0x3;  // only 0-7
    commandout |= 0x18;     // start bit + single-ended bit
	uint8_t spibuf[3];

    spibuf[0] = commandout;
    spibuf[1] = 0;
    spibuf[2] = 0;

    wiringPiSPIDataRW(0, spibuf, 3);    

    adcout = ((spibuf[1] << 8) | (spibuf[2])) >> 4;
	LDR_READING = adcout;
	}
}

//flash LED as warning...might use notification setting in blynk
void warning_light(void){
	 Blynk.virtualWrite(V0, 0); 
     delay(200);
     Blynk.virtualWrite(V0, 255); 
     delay(200);
}

void warning_checker(void){
	//check temp
	if ((TEMP_READING>temp_upper_limit)|(TEMP_READING<temp_lower_limit)){
		warning_flag = true;
	}
	else{
		warning_flag = false;
	} //else do nothing...add code for light and humidity thresholds

}
