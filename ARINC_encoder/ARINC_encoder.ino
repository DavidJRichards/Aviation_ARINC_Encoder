// djrm ARINC encoder
// David Richards, 2022 Feb 10
// Initial - untested

// based on idea here: https://www.youtube.com/watch?v=ih7ZmV-XU-I
// ARINC tutorial: http://leonardodaga.insyde.it/Corsi/AD/Documenti/ARINCTutorial.pdf
// ARINC info: https://www.altadt.com/wp-content/uploads/dlm_uploads/2013/11/ARINC-Protocol-Summary.pdf

#include <Arduino.h>
#include <RotaryEncoder.h>

typedef union {
//    byte ar249_B[4];
//    word ar429_W[2];
    unsigned long ar429_L;
    struct 
    {
      unsigned long label:  8;  // 2,3,3 LSB
      unsigned long sdi:    2;  // source / destination identifier
      unsigned long data:  19;  // 
      unsigned long ssm:    2;  // sign / status matrix
      unsigned long parity: 1; // odd parity MSB
    };
} ARINC429;

#define PIN_IN1 2
#define PIN_IN2 3
RotaryEncoder encoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

constexpr float m = 10;
// at 200ms or slower, there should be no acceleration. (factor 1)
constexpr float longCutoff = 50;
// at 5 ms, we want to have maximum acceleration (factor m)
constexpr float shortCutoff = 5;
// To derive the calc. constants, compute as follows:
// On an x(ms) - y(factor) plane resolve a linear formular factor(ms) = a * ms + b;
// where  f(4)=10 and f(200)=1
constexpr float a = (m - 1) / (shortCutoff - longCutoff);
constexpr float b = 1 - longCutoff * a;
// a global variables to hold the last position
static int lastPos, newPos;
#define INITIAL_ENCODER_VALUE 12

int Hi_429 = 12;
int Lo_429 = 11;
int Debug  = 13;

long t;

ARINC429 dAR429;



// function copied from https://stackoverflow.com/questions/13247647/convert-integer-from-pure-binary-to-bcd
long bin2BCD(long binary) { // double dabble: 8 decimal digits in 32 bits BCD
  if (!binary) return 0;
  long bit = 0x4000000; //  99999999 max binary
  while (!(binary & bit)) bit >>= 1;  // skip to MSB

  long bcd = 0;
  long carry = 0;
  while (1) {
    bcd <<= 1;
    bcd += carry; // carry 6s to next BCD digits (10 + 6 = 0x10 = LSB of next BCD digit)
    if (bit & binary) bcd |= 1;
    if (!(bit >>= 1)) return bcd;
    carry = ((bcd + 0x33333333) & 0x88888888) >> 1; // carrys: 8s -> 4s
    carry += carry >> 1; // carrys 6s  
  }
}

word ARINC429_Permute8Bits(word label8bits)
{
  int i;
  word mask, result;

  result = 0;
  for(i = 0, mask = 0x80; i<8; i += 1, mask >>= 1)
    result |= ( (label8bits & mask) ? (1 << i) : 0 );
  return result;
}

word ARINC429_ComputeMSBParity(unsigned long ar429_L)
{
  int bitnum;
  int parity=0;
  unsigned long bitmask = 1L;

  // check all bits except parity bit
  for(bitnum = 0, bitmask = 1L; bitnum<31; bitnum += 1, bitmask <<= 1L)
  {
    if(ar429_L & bitmask)
      parity++; 
  }
  return (parity%2) ? 0 : 1;
}

unsigned long ARINC429_BuildCommand( unsigned char label, unsigned char sdi, unsigned long data, unsigned char ssm)
{

  dAR429.data = bin2BCD(data); // 19bits data
  dAR429.label = ARINC429_Permute8Bits(label);
  dAR429.sdi = sdi & 0x03;  
  dAR429.ssm = ssm & 0x03;  
  dAR429.parity = ARINC429_ComputeMSBParity(dAR429.ar429_L); // parity: MSB  

  return dAR429.ar429_L;
 }

void printbits(unsigned long value, word start, word finish, char delim)
{
  unsigned long bitnum;
  unsigned long bitmask;

  bitmask = 1L << finish;
  for(bitnum = finish; bitnum+1 > start; bitnum -- , bitmask >>= 1)
    Serial.print(value & bitmask ? "1" : "0");
  
  Serial.print(delim);
}

void ARINC429_PrintCommand( unsigned long ar429_L)
{
  int bitnum;
  unsigned long bitmask = 1L;

  dAR429.ar429_L = ar429_L;

  Serial.print("parity: ");
  Serial.println(dAR429.parity, HEX);
  Serial.print("ssm: ");
  Serial.println(dAR429.ssm, HEX);
  Serial.print("BCD data: ");
  Serial.println(dAR429.data, HEX);
  Serial.print("sdi: ");
  Serial.println(dAR429.sdi, HEX);
  Serial.print("Label (oct): ");
  Serial.println(ARINC429_Permute8Bits(dAR429.label), OCT);

  Serial.print("Bitstream: ");
  printbits(dAR429.parity,0,0,',');
  printbits(dAR429.ssm,1,2,',');
  Serial.print('[');
  printbits(dAR429.data,16,18,',');
  printbits(dAR429.data,12,15,',');
  printbits(dAR429.data,8,11,',');
  printbits(dAR429.data,4,7,',');
  printbits(dAR429.data,0,3,']');
  Serial.print(',');
  printbits(dAR429.sdi,0,1,',');
  printbits(dAR429.label,0,7,'\n');

}


void transmit_bit(char bitvalue)
{
  if(bitvalue)
  {
    digitalWrite(Hi_429, HIGH);
    digitalWrite(Lo_429, LOW);
  }
  else
  {
    digitalWrite(Hi_429, LOW);
    digitalWrite(Lo_429, HIGH);
  }
  delayMicroseconds(24);
  digitalWrite(Hi_429, LOW);
  digitalWrite(Lo_429, LOW);
  delayMicroseconds(40);
}

void ARINC429_SendCommand( unsigned long ar429_L)
{
  int bitnum;
  unsigned long bitmask = 1L;

  digitalWrite(Debug,HIGH);
  // for all bits
  for(bitnum = 0, bitmask = 1L; bitnum<32; bitnum ++, bitmask <<= 1)
  {
    transmit_bit(ar429_L & bitmask ? 1 : 0);
  }  
  digitalWrite(Debug,LOW);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(Hi_429, OUTPUT);
  pinMode(Lo_429, OUTPUT);
  pinMode(Debug,  OUTPUT);
 
  Serial.begin(115200); // Any baud rate should work
  Serial.println("djrm ARINC encoder\n");

  lastPos = newPos= INITIAL_ENCODER_VALUE;
  encoder.setPosition(lastPos);
}

void loop() {
  // put your main code here, to run repeatedly: 
  unsigned long data, ARINC_data;
  word label, sdi, ssm;

  encoder.tick();
  newPos = encoder.getPosition();
  if (lastPos != newPos) {

    // accelerate when there was a previous rotation in the same direction.

    unsigned long ms = encoder.getMillisBetweenRotations();

    if (ms < longCutoff) {
      // do some acceleration using factors a and b

      // limit to maximum acceleration
      if (ms < shortCutoff) {
        ms = shortCutoff;
      }

      float ticksActual_float = a * ms + b;
      Serial.print("  f= ");
      Serial.println(ticksActual_float);

      long deltaTicks = (long)ticksActual_float * (newPos - lastPos);
      Serial.print("  d= ");
      Serial.println(deltaTicks);

      newPos = newPos + deltaTicks;
      encoder.setPosition(newPos);
    }

    Serial.print(newPos);
    Serial.print("  ms: ");
    Serial.println(ms);
    lastPos = newPos;
  } // if


  label = 0201;  // octal
  sdi   = 0;
//  data  = 25786; // decimal
  ssm   = 0;

  t=millis();
  if(t%250L == 0)
  {
    data = lastPos * 10L;
    ARINC_data = ARINC429_BuildCommand(label, sdi, data, ssm);
    ARINC429_PrintCommand(ARINC_data);
    ARINC429_SendCommand(ARINC_data);
  } 
//  delay(200);
}
