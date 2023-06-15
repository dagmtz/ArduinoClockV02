/*
  Arduino Digital Clock V0.2

  Author: 
    Daniel Martinez
    dagmtzs@gmail.com
  
  Date: 
    14/06/2023
*/

#include <Arduino.h>

#define SECONDS_PER_MINUTE 60
#define MINUTES_PER_HOUR 60
#define HOURS_PER_DAY 24
#define SECOND_CHANGE_THRESHOLD 50

typedef enum 
{
  STOP,
  RUNNING,
  SETUP
} clock_state_t ; 

typedef enum 
{
  NONE,
  INCREMENT,
  DECREMENT
} sign_t ;

typedef enum 
{
  SECONDS,
  MINUTES,
  HOURS,
} units_to_set_t ; 

typedef struct 
{
  uint8_t seconds;
  uint8_t minutes;
  uint8_t hours;
} time_hhmmss_t ;

void modifyUnit( const units_to_set_t, const sign_t, const bool );
void setTime( void );

bool printTimeFlag = false;
bool timeSetup = false;
bool buttonPressed = false;
bool buttonMantained = false;

char timeString[9]="00:00:00";

unsigned int lastButtonPressed = 0;

time_hhmmss_t time = { 0,0,0 };
time_hhmmss_t timeBuffer = { 0,0,0 };
clock_state_t clock_state = STOP;
units_to_set_t units_to_set = SECONDS;
units_to_set_t next_units_to_set = MINUTES;

void setup() 
{
  Serial.begin(9600);

  pinMode(4, INPUT); // minus
  pinMode(5, INPUT); // plus
  pinMode(4, INPUT); // change unit to change
  pinMode(7, INPUT); // change clock state

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 3036;
  TCCR1B |= ( 1<<CS12 );
  TIMSK1 |= ( 1<<TOIE1 );

  delay(SECOND_CHANGE_THRESHOLD);
  clock_state = RUNNING;
}

ISR(TIMER1_OVF_vect){
  printTimeFlag = true;
}

void loop() 
{
  if ( clock_state != STOP )
  {
    if( printTimeFlag ){
      modifyUnit( SECONDS, INCREMENT, true );
      sprintf(timeString, "%02d:%02d:%02d", time.hours, time.minutes, time.seconds);    
      
      Serial.println(timeString);
      
      printTimeFlag = false;
    }
    
    if ( digitalRead(7) == LOW  && clock_state == RUNNING )
    {
      clock_state = SETUP;
      units_to_set = SECONDS;
      next_units_to_set = MINUTES;
      Serial.print("SETTING ");
      Serial.print((int) units_to_set);
      Serial.println();
      delay(500);
    }

    if ( clock_state == SETUP )
    {
      setTime();
      if ( digitalRead(6) == LOW )
      {
        if ( next_units_to_set == SECONDS )
        {
          clock_state = RUNNING;
        }
        
        units_to_set = next_units_to_set;

        Serial.print("SETTING ");
        Serial.print((int) units_to_set);
        Serial.println();
        delay(500);
      }
    }

  }
}

void modifyUnit ( const units_to_set_t unit, const sign_t sign, const bool affect_next_unit )
{
  switch (unit)
  {
  case SECONDS:
    if ( sign == INCREMENT )
    {
      time.seconds++;
      if (time.seconds >= SECONDS_PER_MINUTE)
      {
        time.seconds = 0;
        if (affect_next_unit)
        {
          modifyUnit( MINUTES, INCREMENT, true );
        }
      }
    } 
    else if ( sign == DECREMENT )
    {
      time.seconds--;
      if (time.seconds == 0 || time.seconds >= SECONDS_PER_MINUTE)
      {
        time.seconds = SECONDS_PER_MINUTE - 1;
      }
    }
    break;
  
  case MINUTES:
    if ( sign == INCREMENT )
    {
      time.minutes++;
      if (time.minutes >= MINUTES_PER_HOUR)
      {
        time.minutes = 0;
        if (affect_next_unit)
        {
          modifyUnit( HOURS, INCREMENT, false );
        }
      }
    }
    else if ( sign == DECREMENT )
    {
      time.minutes--;
      if (time.minutes == 0 || time.minutes >= MINUTES_PER_HOUR)
      {
        time.minutes = MINUTES_PER_HOUR - 1;
      }
    }
    break;

  case HOURS:
    if ( sign == INCREMENT )
    {
      time.hours++;
      if (time.hours >= HOURS_PER_DAY)
      {
        time.hours = 0;
      }
    }
    else if ( sign == DECREMENT )
    {
      time.hours--;
      if (time.hours == 0 || time.hours >= HOURS_PER_DAY)
      {
        time.hours = HOURS_PER_DAY - 1;
      }
    }
    break;
  
  default:
    break;
  }
}



void setTime()
{
  sign_t signPressed;

  if ( digitalRead(5) == LOW )
  {
    signPressed = INCREMENT;
  } 
  else if ( digitalRead(4) == LOW )
  {
    signPressed = DECREMENT;
  } 
  else 
  {
    signPressed = NONE;
  }

  switch (units_to_set)
  {
    case SECONDS:
      next_units_to_set = MINUTES;
      if( signPressed != NONE )
      {
        modifyUnit( SECONDS, signPressed, false );
        printTimeFlag = true;
      }
      break;
    
    case MINUTES:
      next_units_to_set = HOURS;
      if( signPressed != NONE )
      {
        modifyUnit( MINUTES, signPressed, false );
        printTimeFlag = true;
      }
      break;
    
    case HOURS:
      next_units_to_set = SECONDS;
      if( signPressed != NONE )
      {
        modifyUnit( HOURS, signPressed, false );
        printTimeFlag = true;
      }
      break;
  } 

  if( signPressed != NONE )
  {
    delay(300);
  }
}