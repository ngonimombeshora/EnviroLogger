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


#ifndef ENVIROLOGGER_H
#define ENVIROLOGGER_H

//Includes
#include <wiringPi.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <iostream>

//Define buttons *4
#define CHANGE_TIME_INTERVAL 27// Write your value here
#define RESET_STYSTEM_TIME 23// Write your value here
#define DISMISS_ALARM 22// Write your value here
#define START_STOP_MONITORING 17 // Write your value here

//FOR RTC
// define constants
const char RTCAddr = 0x6f;
const char SEC = 0x00; // see register table in datasheet
const char MIN = 0x01;
const char HOUR = 0x02;
const char TIMEZONE = 2; // +02H00 (RSA)

// for analog readings
int analog_ldr_pin = 1;
int analog_humidity_pin = 2;
int analog_temp_pin = 4;
int temp_upper_limit = 850; //otherwise send warning to use reasonable values here
int temp_lower_limit = 500; //otherwise send warning

//Define LED PINS
// define pins
const int ALARM_LED = 5;


#define BUFFER_SIZE 1000

//SPI Settings FOR ADC. TO BE CONNECTED TO LDR,TEMP_SENSOR,POT
#define SPI_CHAN 0 
#define SPI_SPEED 4*1000*1000

//callbacks for button interupts

void change_time_interval(void);
void reset_system_time(void);
void dismiss_alarm(void);
void start_stop_monitoring(void);
int setup_gpio(void);
void setup(void);
int main(int argc, char* argv[]);
int hexCompensation(int units);
int decCompensation(int units);
void mcp3008_read(uint8_t);
void warning_light(void);
void warning_checker(void);
void timer(int delay_time) ;
int timeConverter(int hrs, int min, int sec);

#endif
