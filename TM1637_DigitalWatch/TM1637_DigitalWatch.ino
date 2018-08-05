//Timer with RTC without FreeRTOS
//Updated by: Er_prabhash
//Email: er.prabhash2015@gmail.com
//Hardware Used: Arduino Uno 328P
//RTC: DS1307
//Display: TM1637 4Digit7Segment

#include <Wire.h>
#include "RTClib.h"  //this Library use I2C protocol A5=SCL, A4=SDA @Arduino Uno
#define CLK 2 //Set the CLK pin connection to the display
#define DIO 3 //Set the DIO pin connection to the display

RTC_DS1307 rtc;
                     /*0*/ /*1*/ /*2*/ /*3*/ /*4*/ /*5*/ /*6*/ /*7*/ /*8*/ /*9*/
uint8_t digits[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f };
                  /*L*/ /*A*/ /*B*/ /*U*/
uint8_t logo[]= { 0x38, 0x77, 0x7f, 0x3e };
uint8_t hours;
uint8_t minutes;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setup()
{
  Serial.begin(9600);
  pinMode(CLK, OUTPUT);
  pinMode(DIO, OUTPUT);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
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
  write_fun(logo[0], logo[1] , logo[2], logo[3]);
  delay(1000); //Display LABU for 1 second
  // clear display
  write_fun(0x00, 0x00, 0x00, 0x00);
}

void loop()
{
    DateTime now = rtc.now();
    uint8_t a=now.hour();
//    1 se 3 night
//    4 se 11 morning
//    12 se 17 afternoon
//    18 se 21 evening
//    22 se 24 night
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

      hours=(uint8_t)now.hour();
      minutes=(uint8_t)now.minute();

      write_fun(digits[hours / 10], (now.second()%2==0)?(digits[hours % 10]):(digits[hours % 10] | ((0x01) << 7)) , digits[minutes / 10], digits[minutes % 10]);
      setBrightness(0x0f, true);
      delay(800);

//      setBrightness(0x06, false);
      delay(200);
      Serial.println();
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

