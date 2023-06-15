/*
  Arduino Digital Clock V0.1

  Author: 
    Daniel Martinez
    dagmtzs@gmail.com
  
  Date: 
    14/06/2023
*/

#include <Arduino.h>

#define MILLIS_PER_SECOND 1000
#define SECONDS_PER_MINUTE 60
#define MINUTES_PER_HOUR 60
#define HOURS_PER_DAY 24
#define SECOND_CHANGE_THRESHOLD 200

unsigned int hours = 0;
unsigned int minutes = 0;
unsigned int seconds = 0;
unsigned int msCounter = 0;
bool newSecondFlag = false;

char timeBuffer[9];
char time[9]="00:00:00";

void setup() 
{
  Serial.begin(9600);
  delay(SECOND_CHANGE_THRESHOLD);
}

void updateBuffer()
{
  msCounter = millis() % 1000;

  if ( msCounter <= SECOND_CHANGE_THRESHOLD && !newSecondFlag )
  {
    seconds++;
    if ( seconds == SECONDS_PER_MINUTE )
    {
      seconds = 0;
      minutes++;
    }

    if ( minutes == MINUTES_PER_HOUR )
    {
      minutes = 0;
      hours++;
    }

    if ( hours == HOURS_PER_DAY )
    {
      hours = 0;
    }

    newSecondFlag = true;
  }

  if ( msCounter > SECOND_CHANGE_THRESHOLD && newSecondFlag )
  {
    newSecondFlag = false;
  }
  
  sprintf(timeBuffer, "%02d:%02d:%02d", hours, minutes, seconds);
}

void printTime()
{
  if ( time[7] != timeBuffer[7] )
  {
    strcpy(time, timeBuffer);
    Serial.println(timeBuffer);
  }
}

void loop() 
{

  updateBuffer();
  printTime();
  
}
