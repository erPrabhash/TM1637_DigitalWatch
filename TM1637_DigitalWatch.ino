//Timer with RTC using FreeRTOS
//Updated by: Er_prabhash
//Email: er.prabhash2015@gmail.com
//Updated On: 4/aug/2018 09:07 PM
//Hardware Used: Arduino Uno 328P
//RTC: DS1307
//Display: TM1637 4Digit7Segment

#define CLK 2 //Set the CLK pin connection to the display
#define DIO 3 //Set the DIO pin connection to the display
#define TEST_DELAY 1000 //(in milliSeconds)

                     /*0*/ /*1*/ /*2*/ /*3*/ /*4*/ /*5*/ /*6*/ /*7*/ /*8*/ /*9*/
uint8_t digits[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f };
                  /*L*/ /*A*/ /*B*/ /*U*/
uint8_t logo[]= { 0x38, 0x77, 0x7f, 0x3e };

uint8_t minutes;
uint8_t seconds;

void setup()
{
  Serial.begin(9600);
  pinMode(CLK, OUTPUT);
  pinMode(DIO, OUTPUT);
  Serial.print("\n Timer ON. ");

  start();
  writeValue(0x8c);
  stop();

  write(logo[0], logo[1] , logo[2], logo[3]);
  delay(2000);
  // clear display
  write(0x00, 0x00, 0x00, 0x00);
}

void loop()
{
for (minutes = 0; minutes < 3; minutes++) 
   {for (seconds = 0; seconds < 60; seconds++)
     {
      Serial.print("\n\nMinute: ");
      Serial.print(minutes);
      Serial.print("\tSecond: ");
      Serial.print(seconds);
      write(digits[minutes / 10], digits[minutes % 10] | ((seconds & 0x01) << 7) , digits[seconds / 10], digits[seconds % 10]);
      delay(1000);
     }
   }
}

void write(uint8_t first, uint8_t second, uint8_t third, uint8_t fourth)
{
  start();
  writeValue(0x40);
  stop();

  start();
  writeValue(0xc0);
  writeValue(first);
  writeValue(second);
  writeValue(third);
  writeValue(fourth);
  stop();
}

void start(void)
{
  digitalWrite(CLK,HIGH); //send start signal to TM1637
  digitalWrite(DIO,HIGH);
  delayMicroseconds(5);

  digitalWrite(DIO,LOW);
  digitalWrite(CLK,LOW);
  delayMicroseconds(5);
}

void stop(void)
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



/* For better understanding of 7-SEGMENT
//
//     -A-
//  F |   | B
//     -G-
//  E |   | C
//     -D-
//      

// XGFE  DCBA
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
