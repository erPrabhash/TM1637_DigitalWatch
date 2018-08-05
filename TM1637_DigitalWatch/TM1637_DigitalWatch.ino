//Timer with RTC using FreeRTOS
//Updated by: Er_prabhash
//Updated On: 8/aug/2018 09:07 PM
//Hardware Used: Arduino Uno 328P
//RTC: DS1307
//Display: TM1637 4Digit7Segment

#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <Wire.h>
#include "RTClib.h" //this Library use I2C protocol A5=SCL, A4=SDA @ArduinoUno
#define CLK 2 //Set the CLK pin connection to the display
#define DIO 3 //Set the DIO pin connection to the display
#define PIN_U 10   //digitalPin for UP  */ Both Pins are pulledUp_Internally
#define PIN_D 11   //digitalPin for DOWN

RTC_DS1307 rtc;
                     /*0*/ /*1*/ /*2*/ /*3*/ /*4*/ /*5*/ /*6*/ /*7*/ /*8*/ /*9*/
uint8_t digitsUp[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f };
                  /*L*/ /*A*/ /*B*/ /*U*/
uint8_t logoUp[]= { 0x38, 0x77, 0x7f, 0x3e };

                        /*0*/ /*1*/ /*2*/ /*3*/ /*4*/ /*5*/ /*6*/ /*7*/ /*8*/ /*9*/
uint8_t digitsDown[] = { 0x3f, 0x30, 0x5b, 0x79, 0x74, 0x6d, 0x6f, 0x38, 0x7f, 0x7d };
                      /*L*/ /*A*/ /*B*/ /*U*/
//uint8_t logoDown[]= { 0x07, 0x7e, 0x7f, 0x37 };

bool origin_status=true;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;
// define tasks here
void TaskShowTime( void *p ); //Normal time
void TaskGetTime( void *p );

struct TimeFormate{
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
  }RTC_time;

void setup()
{
  Serial.begin(9600);
  pinMode(CLK, OUTPUT);
  pinMode(DIO, OUTPUT);
  origin_status=true;
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
  
  if (! rtc.begin()) 
   {Serial.println("Couldn't find RTC");
    while (1);
   }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    }
    
  Serial.print("\n Timer ON. ");
  start_fun();
  writeValue(0x8c);
  stop_fun();

  setBrightness(0x0f, true);
  write_fun(logoUp[0], logoUp[1] , logoUp[2], logoUp[3]);
  delay(2000); //Display LABU for 1 second
  // clear display
  write_fun(0x00, 0x00, 0x00, 0x00);

  xTaskCreate(
    TaskGetTime
    ,(const portCHAR *) "Read_RTC"
    ,128
    ,NULL
    ,configMAX_PRIORITIES-1
    ,NULL);

  xTaskCreate(
    TaskShowTime
    ,(const portCHAR *) "Show_Segment_Up"
    ,128
    ,NULL
    ,configMAX_PRIORITIES-1
    ,NULL);
}

void loop()
{
  // Empty. Things are done in Tasks.
}

void TaskGetTime(void *p )
{  
   (void) p;
   while(1)
   {
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      DateTime now = rtc.now();
      uint8_t a=now.hour();
  //     1 to 3 night
  //     4 to 11 morning
  //    12 to 17 afternoon
  //    18 to 21 evening
  //    22 to 24 night
  //    Serial.print(" Value of a = ");
  //    Serial.print(a);
      if (a>=0 && a<=3)
        { Serial.print(" Good Night, Prabhash. ");  }
      else if (a>=4 && a<=11)
        { Serial.print(" Good Morning, Prabhash. ");  }
      else if (a>=12 && a<=17)
        { Serial.print(" Good Afternoon, Prabhash. ");  }
      else if (a>=18 && a<=21)
        { Serial.print("Good Evening, Prabhash. ");  }
      else if (a>=22 && a<=24)
        { Serial.print(" Good Night, Prabhash. ");  }
        
        Serial.println("\t  ");
        Serial.print(now.year(), DEC);
        Serial.print('/');
        Serial.print(now.month(), DEC);
        Serial.print('/');
        Serial.print(now.day(), DEC);
        Serial.print(" (");
        Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
        Serial.print(") ");
        Serial.print(now.hour(), DEC);
        Serial.print(':');
        Serial.print(now.minute(), DEC);
        Serial.print(':');
        Serial.print(now.second(), DEC);
        Serial.println();

        RTC_time.hours=(uint8_t)now.hour();
        RTC_time.minutes=(uint8_t)now.minute();
        RTC_time.seconds=(uint8_t)now.second();

        //Serial.print("Time Successfully written in RTC_time");
        xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
     }
     vTaskDelay(1);
   }
}

void TaskShowTime(void *p )
{
   (void) p;
   while(1)
   {
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {  
      pinMode(PIN_U, INPUT);
      digitalWrite(PIN_U,INPUT_PULLUP);
      pinMode(PIN_D, INPUT);
      digitalWrite(PIN_D,INPUT_PULLUP);
      
      if(!digitalRead(PIN_U))
       {
         origin_status=true;
         Serial.println("PIN UP");
         write_fun(digitsUp[RTC_time.hours / 10], (RTC_time.seconds%2==0)?(digitsUp[RTC_time.hours % 10]):(digitsUp[RTC_time.hours % 10] | ((0x01) << 7)) , digitsUp[RTC_time.minutes / 10], digitsUp[RTC_time.minutes % 10]);
         setBrightness(0x0f, true);
       }
       
      else if(!digitalRead(PIN_D))
       {
         origin_status=false;
         Serial.println("PIN DOWN");
         write_fun(digitsDown[RTC_time.minutes % 10],(RTC_time.seconds%2==0)?(digitsDown[RTC_time.minutes / 10]):(digitsDown[RTC_time.minutes / 10] | ((0x01) << 7)),digitsDown[RTC_time.hours % 10], digitsDown[RTC_time.hours / 10]);
         setBrightness(0x0f, true);
       }
      else
       {
         origin_status=true;
         Serial.println("PIN OTHER");
         write_fun(digitsUp[RTC_time.hours / 10], (RTC_time.seconds%2==0)?(digitsUp[RTC_time.hours % 10]):(digitsUp[RTC_time.hours % 10] | ((0x01) << 7)) , digitsUp[RTC_time.minutes / 10], digitsUp[RTC_time.minutes % 10]);
         setBrightness(0x03, false);
       }
    }
    xSemaphoreGive( xSerialSemaphore );

      vTaskDelay(1000 / portTICK_PERIOD_MS );
      vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
   }
}

void write_fun(uint8_t first, uint8_t second, uint8_t third, uint8_t fourth)
{
  start_fun();
  writeValue(0x40);
  stop_fun();

  start_fun();
  writeValue(0xc0);
  writeValue(first);
  writeValue(second);
  writeValue(third);
  writeValue(fourth);
  stop_fun();
}

void start_fun(void)
{
  digitalWrite(CLK,HIGH); //send start signal to TM1637
  digitalWrite(DIO,HIGH);
  delayMicroseconds(5);

  digitalWrite(DIO,LOW);
  digitalWrite(CLK,LOW);
  delayMicroseconds(5);
}

void stop_fun(void)
{
  digitalWrite(CLK,LOW);
  digitalWrite(DIO,LOW);
  delayMicroseconds(5);

  digitalWrite(CLK,HIGH);
  digitalWrite(DIO,HIGH);
  delayMicroseconds(5);
}

bool writeValue(uint8_t value)
{
 for(uint8_t i = 0; i < 8; i++)
  {
    digitalWrite(CLK, LOW);
    delayMicroseconds(5);
    digitalWrite(DIO, (value & (1 << i)) >> i);
    delayMicroseconds(5);
    digitalWrite(CLK, HIGH);
    delayMicroseconds(5);
  }

  // wait for ACK
  digitalWrite(CLK,LOW);
  delayMicroseconds(5);

  pinMode(DIO,INPUT);

  digitalWrite(CLK,HIGH);
  delayMicroseconds(5);

  bool ack = digitalRead(DIO) == 0;

  pinMode(DIO,OUTPUT);

  return ack;
}

void setBrightness(uint8_t brightness, bool on)
{
  uint8_t m_brightness = (brightness & 0x7) | (on? 0x08 : 0x00);
  writeValue(m_brightness);
  start_fun();
  writeValue(0x80 + (m_brightness & 0x0f));
  stop_fun();
}



/* For better understanding of 7-SEGMENT
//
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D

    // XGFEDCBA
     0b0011 1000,    // L  38
     0b0111 0111,    // A  77
     0b0111 1111,    // 8  7f
     0b0011 1110,    // U  3e

const uint8_t digitToSegment[] = {
 // XGFEDCBA
  0b00111111,    // 0
  0b00000110,    // 1
  0b01011011,    // 2
  0b01001111,    // 3
  0b01100110,    // 4
  0b01101101,    // 5
  0b01111101,    // 6
  0b00000111,    // 7
  0b01111111,    // 8
  0b01101111,    // 9
  0b01110111,    // A
  0b01111100,    // b
  0b00111001,    // C
  0b01011110,    // d
  0b01111001,    // E
  0b01110001     // F
  };*/

