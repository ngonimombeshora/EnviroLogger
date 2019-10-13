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
#include "CurrentTime.h"
#include <time.h>

int HH,MM,SS;

#include "EnviroLogger.h"
#include <wiringPiSPI.h>
#include "CurrentTime.h"
#include <softPwm.h>

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
int TEMP_READING ;
float HUMIDITY_READING ; 
int LDR_READING;
bool warning_flag = false;
bool STOP = false;
float DAC_VOUT ;//(LDR_READING/1023)*HUMIDITY_READING;
char alarm_status = ' ';
// hours, minutes, seconds of timer 
int hours = 0; 
int minutes = 0; 
int seconds = 0; 
int delay_time = 700;
int delay_compensation = 700; 
int dismissal_hours = 0;
int dismissal_Minutes = 0;
int dismissal_Seconds=0;
int ran = 0;

//------------------------------------------------------

 void setup()
{
    Blynk.begin(auth, serv, port);
   
}
//------------------------------------------------------------
void loop()
{
	warning_checker(); //validate reading before sending to blynk app
	Blynk.run();
	 Blynk.virtualWrite(V1, HUMIDITY_READING); 
	 Blynk.virtualWrite(V2, (TEMP_READING*330/1024)); 
	 Blynk.virtualWrite(V3, LDR_READING); 
	Blynk.virtualWrite(V4, "|Humidity:",HUMIDITY_READING, "|Temp:",(TEMP_READING*330/1024), "|Light",LDR_READING),"|\n";
	//Serial.println("|Humidity:",HUMIDITY_READING, "|Temp:",TEMP_READING, "|Light",LDR_READING),"|\n");
	warning_light();
}
//-------------------------------------------------------------------



// Configure interrupts here. NB:For button interupts____with DEBOUNCING
//...DONE WITH FILLING IN LOGIC!
void change_time_interval(void){
    //debouncing
    long interruptTime = millis();
	if (interruptTime - lastInterruptTime>200){
        //Write your logis here
       if(delay_time == 700){
		   delay_time = 1700;//850*2
		   delay_compensation = 850;
	   }
	   else if(delay_time == 1700){
		   delay_time = 4750;//950*5
		   delay_compensation = 950;
	   }
	   else{
		   delay_time = 700;//700*1
		   delay_compensation = 700;
	   }
	   printf("Delay_time%d\n", delay_time);
    }
	lastInterruptTime = interruptTime;
}

void reset_system_time(void){
    //debouncing
    long interruptTime = millis();
	if (interruptTime - lastInterruptTime>200){
        //Write your logis here
		hours = 0;
		minutes = 0;
		seconds = 0;
		Blynk.virtualWrite(V4,"clr");
			printf("Reset button pressed\n");
       
    }
	lastInterruptTime = interruptTime;
}

void dismiss_alarm(void){
    //debouncing
    long interruptTime = millis();
	if (interruptTime - lastInterruptTime>200){
        //Write your logis here
		warning_flag = false;
		dismissal_Minutes = minutes;// capture of time when alarm was dismissed
		dismissal_Seconds = seconds;
		ran =1;	// varibale to show alarm has ran before
	   printf("Dismiss Alarm\n");
    }
	lastInterruptTime = interruptTime;
}

void start_stop_monitoring(void){
    //debouncing
    long interruptTime = millis();
	if (interruptTime - lastInterruptTime>200){
        //Write your logis here
		if (STOP==false){
			STOP = true;
		}
		else{
			STOP = false;
		}
		
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
	softPwmCreate(4,0,100);
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
	for(;;){
	int HH = getHours();
	int MM = getMins();
	int SS = getSecs();
	timer(delay_time) ;
	loop(); //for sending data to blynk
	mcp3008_read(analog_ldr_pin);
	mcp3008_read(analog_humidity_pin);
	mcp3008_read(analog_temp_pin);
	DAC_VOUT = (LDR_READING*HUMIDITY_READING)/1023;//*HUMIDITY_READING;//
	if(!STOP)
	{
		printf("|RTC Time||System Timer|| Humidity ||   Temp   ||   Light   ||  DAC Vout  ||Alarm||\n");
		printf("|%02d:%02d:%02d||%02d:%02d:%02d    ||    %0.2f  ||   %d     ||    %d    ||   %0.2f   ||  %c  ||\n",HH,MM,SS,hours,minutes,seconds, HUMIDITY_READING,(TEMP_READING*330/1024),LDR_READING,DAC_VOUT,alarm_status);
	}
	}
  return 0;
}

void secPWM(int units){
	softPwmWrite(4,units);
}

void mcp3008_read(uint8_t adcnum) //threading? or is it buffering?
{ 
    unsigned int commandout = 0;
    unsigned int adcout = 0;
	if (STOP==false){
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
		HUMIDITY_READING = (adcout*3.3)/1023;
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
	
}

//flash LED to blynk as warning...might use notification setting in blynk
void warning_light(void){
	int diff = timeConverter(hours, minutes, seconds) - timeConverter(dismissal_hours, dismissal_Minutes, dismissal_Seconds); 
	if (warning_flag == false){// no need for alarm 
		Blynk.virtualWrite(V0, 0);
		alarm_status = ' ';
	    pinMode (4, OUTPUT) ;
	  	digitalWrite (4, LOW) ;
	}else if (warning_flag == true & ran ==0){// first alarm
     	Blynk.virtualWrite(V0, 255); 
		alarm_status = '*';
		 pinMode (4, OUTPUT) ;
	  	digitalWrite (4, HIGH) ;
	} else if (warning_flag == true & ran ==1 & (diff> 30)){// last sounded alarm more than 3 min ago
		Blynk.virtualWrite(V0, 255);
		alarm_status = '*';
		 pinMode (4, OUTPUT) ;
		 	  	digitalWrite (4, HIGH) ;
	}else if (warning_flag == true & ran ==1 & (diff<= 30)){// last sounded alarm less than 3 min ago
		Blynk.virtualWrite(V0, 0);
		alarm_status = ' ';
			    pinMode (4, OUTPUT) ;
	  	digitalWrite (4, LOW) ;
	}
}

void warning_checker(void){
	//check temp
	if ((DAC_VOUT<0.65)|(DAC_VOUT>2.65)){
		warning_flag = true;
	}
	else{
		warning_flag = false;
	} //else do nothing...add code for light and humidity thresholds

}

void timer(int delay_time) 
{ 
    // infinte loop because timer will keep  
    // counting. To kill the process press 
    // Ctrl+D. If it does not work ask 
    // ubuntu for other ways. 
	delay(delay_time);

        // increment seconds 
        seconds = seconds + (delay_time/delay_compensation); 
  
        // if seconds reaches 60 
		if (seconds>60){
			
			 // increment minutes 
            minutes++; 
  
    // if minutes reaches 60 
            if         (minutes == 60) { 
          
                // increment hours 
                hours++; 
                minutes = 0; 
            } 
            seconds = seconds - 60;
		}
    else if (seconds == 60) { 
          
            // increment minutes 
            minutes++; 
  
    // if minutes reaches 60 
            if         (minutes == 60) { 
          
                // increment hours 
                hours++; 
                minutes = 0; 
            } 
            seconds = 0; 
        
    } 
} 

int timeConverter(int hrs, int min, int sec)// converts system time to seconds
{
	int time_in_sec = (hrs*60*60)+(min*60)+(sec);
	return time_in_sec;
}


void getCurrentTime(void){
  time_t rawtime;
  struct tm * timeinfo;
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );

  HH = timeinfo ->tm_hour;
  MM = timeinfo ->tm_min;
  SS = timeinfo ->tm_sec;
}

int getHours(void){
    getCurrentTime();
    return HH;
}

int getMins(void){
    return MM;
}

int getSecs(void){
    return SS;
}