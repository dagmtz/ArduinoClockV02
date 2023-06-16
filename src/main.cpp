/*
  Arduino Digital Clock V0.2

  Author: 
    Daniel Martinez
    dagmtzs@gmail.com
  
  Date: 
    15/06/2023
*/

#include <Arduino.h>

#define SECONDS_PER_MINUTE 60
#define MINUTES_PER_HOUR 60
#define HOURS_PER_DAY 24
#define STARTUP_DELAY_MS 50
#define DEBOUNCE_THRESHOLD_MS 20
#define PRESS_THRESHOLD_MS 330
#define REPEAT_THRESHOLD_MS 200
#define BUTTON_SETUP_BIT B10000000
#define BUTTON_PLUS_BIT B01000000
#define BUTTON_MINUS_BIT B00100000

typedef struct 
{
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
} time_hhmmss_t ;

typedef enum 
{
  MINUS = -1,
  NO_SIGN, 
  PLUS
} sign_t ;

typedef enum 
{
  STOP,
  RUNNING,
  SETUP
} clock_state_t ; 

typedef enum
{
  NOT_PRESSED,
  DEBOUNCE,
  SINGLE,
  HOLD, 
  WAIT
} button_press_state_t ;

typedef enum 
{
  NO_UNIT,
  SECONDS,
  MINUTES,
  HOURS
} time_units_t ; 

typedef struct
{
  uint8_t currentButtonReading;
  uint8_t lastButtonReading;
  uint8_t currentButtonSelected;
  button_press_state_t currentButtonState;
  button_press_state_t lastButtonState;
  unsigned long lastPressTime; 
} button_control_t ;

typedef struct 
{
  clock_state_t clockState;
  time_hhmmss_t time;
  char timeString[9];
} time_control_t;

volatile bool g_heartbeat_s = false;

time_control_t g_timeControl = { STOP, {0,0,0}, "00:00:00" };
button_control_t g_buttonControl = { 0, 0, 0, NOT_PRESSED, NOT_PRESSED, 0 };
time_units_t g_units_to_set = NO_UNIT;
time_units_t g_next_units_to_set = SECONDS;

void read_buttons(void);
void update_time(const time_units_t, const sign_t, const bool, const bool);
void print_time(void);
void update_settings(void);


void setup() 
{
  Serial.begin(9600);

  // Set WGM13, WGM12, WGM11 and WGM10 to 0 to select "nomal mode" interruption mode
  TCCR1A = 0;
  TCCR1B = 0;
  // Set start value for the timer counter register
  TCNT1 = 3036;
  
  // Set prescaler to 256
  TCCR1B |= ( 1<<CS12 );
  // Enable Overflow Interrupt
  TIMSK1 |= ( 1<<TOIE1 );

  // Set pins 5 through 7 as inputs, let the rest of the register unchanged
  DDRD = DDRD & ~( BUTTON_SETUP_BIT | BUTTON_PLUS_BIT | BUTTON_MINUS_BIT );
  // Set pins 5 through 7 HIGH (to set the pull-up resistors), let the rest of the register unchanged
  PORTD = PORTD | ( BUTTON_SETUP_BIT | BUTTON_PLUS_BIT | BUTTON_MINUS_BIT );

  delay(STARTUP_DELAY_MS);
  g_timeControl.clockState = RUNNING;
  
}

ISR(TIMER1_OVF_vect){
  TCNT1 = 3036;
  g_heartbeat_s = true;
}

void loop()
{
  if( g_heartbeat_s )
  {
    update_time(SECONDS, PLUS, true, true);
    g_heartbeat_s = false;
  }
  
  read_buttons();
  update_settings();

}

void read_buttons()
{
   g_buttonControl.currentButtonReading = (~PIND) & B11100000;
  
  switch ( g_buttonControl.currentButtonState)
  {
  case NOT_PRESSED:
    if(  g_buttonControl.currentButtonReading != 0 )
    {
      g_buttonControl.lastPressTime = millis();
      g_buttonControl.lastButtonReading = g_buttonControl.currentButtonReading;

      switch ( g_buttonControl.currentButtonReading)
      {
      case BUTTON_SETUP_BIT:
        g_buttonControl.currentButtonSelected = 7;
        break;

      case BUTTON_PLUS_BIT:
        g_buttonControl.currentButtonSelected = 6;
        break;

      case BUTTON_MINUS_BIT:
        g_buttonControl.currentButtonSelected = 5;
        break;
      
      default:
        g_buttonControl.currentButtonSelected = 0;
        break;
      }

       g_buttonControl.currentButtonState = DEBOUNCE;
    } 
    break;
  
  case DEBOUNCE:
    if ( (g_buttonControl.currentButtonReading == g_buttonControl.lastButtonReading) && (g_buttonControl.currentButtonSelected != 0) )
    {
      g_buttonControl.currentButtonState = WAIT;
      g_buttonControl.lastButtonState = DEBOUNCE;
    } 
    break;
  
  case SINGLE:
    if ( (g_buttonControl.currentButtonReading == g_buttonControl.lastButtonReading) && (g_buttonControl.currentButtonSelected != 0) )
    {
      g_buttonControl.currentButtonState = WAIT;
      g_buttonControl.lastButtonState = SINGLE;
    }
    else 
    {
      g_buttonControl.currentButtonState = NOT_PRESSED;
      g_buttonControl.lastPressTime = 0;
      g_buttonControl.lastButtonReading = 0;
      g_buttonControl.currentButtonSelected = 0;
    }
    break;
  
  case HOLD:
    if ( (g_buttonControl.currentButtonReading == g_buttonControl.lastButtonReading) && (g_buttonControl.currentButtonSelected != 0) )
    {
      g_buttonControl.currentButtonState = WAIT;
      g_buttonControl.lastButtonState = HOLD;
    }
    else 
    {
      g_buttonControl.currentButtonState = NOT_PRESSED;
      g_buttonControl.lastPressTime = 0;
      g_buttonControl.lastButtonReading = 0;
      g_buttonControl.currentButtonSelected = 0;
    }
    break;
  
  case WAIT:
    if ( (g_buttonControl.currentButtonReading == g_buttonControl.lastButtonReading) && (g_buttonControl.currentButtonSelected != 0) )
    {
      switch (g_buttonControl.lastButtonState)
      {
      case DEBOUNCE:
        if( (millis() - g_buttonControl.lastPressTime) > DEBOUNCE_THRESHOLD_MS )
        { 
          g_buttonControl.currentButtonState = SINGLE;
          g_buttonControl.lastButtonState = WAIT;
        }
        break;
      
      case SINGLE:
        if( (millis() - g_buttonControl.lastPressTime) > PRESS_THRESHOLD_MS )
        {
          g_buttonControl.currentButtonState = HOLD;
          g_buttonControl.lastButtonState = WAIT;
        }
        break;

      case HOLD:
        if( (millis() - g_buttonControl.lastPressTime) > REPEAT_THRESHOLD_MS )
        {
          g_buttonControl.currentButtonState = HOLD;
          g_buttonControl.lastButtonState = WAIT;
        }
        break;

      default:
        break;
      }
    }
    else 
    {
      g_buttonControl.currentButtonState = NOT_PRESSED;
      g_buttonControl.lastButtonState = WAIT;
      g_buttonControl.lastPressTime = 0;
      g_buttonControl.lastButtonReading = 0;
      g_buttonControl.currentButtonSelected = 0;
    }
    break;

  default:
    break;
  }
}

void update_time( const time_units_t unit, const sign_t sign, const bool affectNextUnit, const bool printTime )
{
  switch (unit)
  {
  case SECONDS:
    switch (sign)
    {
    case MINUS:
      g_timeControl.time.seconds--;
      if (g_timeControl.time.seconds >= SECONDS_PER_MINUTE)
      {
        g_timeControl.time.seconds = SECONDS_PER_MINUTE - 1;
      }
      break;
    
    case NO_SIGN:
      break;

    case PLUS:
      g_timeControl.time.seconds++;
      if (g_timeControl.time.seconds >= SECONDS_PER_MINUTE)
      {
        g_timeControl.time.seconds = 0;
        if (affectNextUnit)
        {
          update_time( MINUTES, PLUS, true, false );
        }
      }
      break;

    default:
      break;
    }
    break;

  case MINUTES:
    switch (sign)
    {
    case MINUS:
      g_timeControl.time.minutes--;
      if (g_timeControl.time.minutes >= MINUTES_PER_HOUR)
      {
        g_timeControl.time.minutes = MINUTES_PER_HOUR - 1;
      }
      break;
    
    case NO_SIGN:
      break;

    case PLUS:
      g_timeControl.time.minutes++;
      if (g_timeControl.time.minutes >= MINUTES_PER_HOUR)
      {
        g_timeControl.time.minutes = 0;
        if (affectNextUnit)
        {
          update_time( HOURS, PLUS, true, false );
        }
      }
      break;

    default:
      break;
    }
    break;


  case HOURS:
    switch (sign)
    {
    case MINUS:
      g_timeControl.time.hours--;
      if (g_timeControl.time.hours >= HOURS_PER_DAY)
      {
        g_timeControl.time.hours = HOURS_PER_DAY - 1;
      }
      break;
    
    case NO_SIGN:
      break;

    case PLUS:
      g_timeControl.time.hours++;
      if (g_timeControl.time.hours >= HOURS_PER_DAY)
      {
        g_timeControl.time.hours = 0;
      }
      break;

    default:
      break;
    }
    break;

  default:
    break;
  }

  if (printTime) 
  {
    print_time();
  }
}

void print_time()
{

  sprintf(g_timeControl.timeString, "%02d:%02d:%02d", g_timeControl.time.hours, g_timeControl.time.minutes, g_timeControl.time.seconds);    
  Serial.println(g_timeControl.timeString);

}

void update_settings()
{
  switch (g_timeControl.clockState)
  {
    case RUNNING:
      switch (g_buttonControl.currentButtonSelected)
      {
        case 7:
          if( g_buttonControl.currentButtonState == SINGLE )
          {
            g_timeControl.clockState = SETUP;
            Serial.println("SETUP: SECONDS");
            g_units_to_set = SECONDS;
          }
          break;
        
        default:
          break;
      }
      break;
    
    default:
      break;
    
    case SETUP:
      switch (g_buttonControl.currentButtonSelected)
      {
        case 7:
          if( g_buttonControl.currentButtonState == SINGLE )
          {
            switch (g_units_to_set)
            {        
              case SECONDS:
                Serial.println("SETUP: MINUTES");
                g_units_to_set = MINUTES;
                break;

              case MINUTES:
                Serial.println("SETUP: HOURS");
                g_units_to_set = HOURS;
                break;
              
              case HOURS:
                Serial.println("EXIT SETUP");
                g_units_to_set = NO_UNIT;
                g_timeControl.clockState = RUNNING;
                break;
              
              default:
                break;
            }
          }
          break;
      
        case 6:
          if( (g_buttonControl.currentButtonState == SINGLE || g_buttonControl.currentButtonState == HOLD) && (g_timeControl.clockState == SETUP) )
          {
            update_time(g_units_to_set, PLUS, false, true);
          }
          break;
        
        case 5:
          if( (g_buttonControl.currentButtonState == SINGLE || g_buttonControl.currentButtonState == HOLD) && (g_timeControl.clockState == SETUP) )
          {
            update_time(g_units_to_set, MINUS, false, true);
          }
          break;

        default:
          break;
      }
    break;
  }
}
