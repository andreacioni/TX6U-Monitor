/**
    Arduino Sketch for TX6U LaCrosse Temperature Sensor
   
    Reference Links:
        http://www.f6fbb.org/domo/sensors/tx_signals.php  -- wave forms
        http://www.f6fbb.org/domo/sensors/tx3_th.php -- message format
        https://www.sparkfun.com/products/10532 -- 433Mhz receiver
        https://projects.drogon.net/raspberry-pi/wiringpi/ -- GPIO library

    The TX6U La Crosse temperature sensors work by sending data, 1 time
    per minute (approximately) on the 433Mhz channel. 

    Bits arrive at the receiver according to the waveforms at the first link
    above. Replicated here in case the link is dead.

    Bit 0 -- Total frame size is ~2.3ms, data is ~1.3ms, break word is ~1ms

      +----------------+
      |                |    
   ___|                |_______________|
       <---  1.3ms ---><----  1ms  --->

    Bit 1 -- Total frame size is ~1.6ms, data is ~0.5ms, break word is ~1.1ms

      +-------+
      |       |    
   ___|       |_______________|
       <0.5ms ><---- 1.1ms --->

    Once receiving bits, the complete message from the sensor is described in the 
    second link. Replicated here in case the link is dead.

    Each message is 44 bits long. It is composed of:

    2 nibbles (start sequence 0x0A)
    1 nibble (sensor type. 0 = temp)
    7 bits (sensor id)
    1 bit (parity)
    8 nibbles (data)
    1 nibble (checksum)

                           Parity Bit
      Preamble  ST     Id    | 10s   1s  .0   10s  1s   CRC
      +-------+ +--+ +------+| +--+ +--+ +--+ +--+ +--+ +--+
      0000 1010 0000 0000 1110 0111 0011 0001 0111 0011 1101
        0    A    0    0   7 0   7    3    1    7    3    D    
            Checksum: (0 + A + 0 + 0 + E + 7 + 3 + 1 + 7 + 3) and F = D     

    The measure is on 3 BCD digits, the two first digits are repeated to fill the 5 nibles.

    Acknowlegement:
        https://github.com/pwdng/WeatherMonitor 
            provided the initial code. I rewrote the various functions to my style.
        
    Special thanks to Ernest Biancarelli for this post and the description above
        http://biancarelli.org/blog/2014/07/27/fun-with-a-tx6u-temperature-sensor/

**/


#define PIN 2 // interrupt -> 0 on Arduino UNO
#define BUFF_LEN 200

volatile unsigned long currTime=0,lastTime=0;
volatile int currState=0,readI=0,writeI=0;
volatile unsigned long diff;
volatile byte buff[BUFF_LEN],b;

volatile boolean shortHighDetect = false;
volatile boolean longHighDetect = false;

int waited=0;

boolean recognizePattern();
void printRing();
void putValue(byte);
byte getValue();

#define DEBUG

void setup() {
  
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  
  attachInterrupt(0,isr,CHANGE);
}

void loop() {
  if(recognizePattern())
  {
    #ifdef DEBUG
     Serial.println("pattern recognized");
    #endif
    int i=0,value=0;
    byte sum=0;
    byte type,id,data,checksum;
     while(i<32)
     {
       if(readI != writeI)
       {
         byte bit = getValue();
         #ifdef DEBUG
          Serial.print(bit);
         #endif
         if(bit == 1)
           value += bit(i%4);
         if(((i+1) % 4)  == 0)
         {
           Serial.print("(");
           Serial.print(value);
           Serial.print(")");
           Serial.print(" ");
           sum += (byte) value;
           value=0;
         }
         
         i++;
       }
       
     }
     #ifdef DEBUG
       sum = sum & 0x0F;
       Serial.print(" SUM:");
       Serial.print(sum,HEX);
       Serial.println();
     #endif
          
  }
    
    //printRing();
}

#ifdef DEBUG
  void printRing()
  {
    if(readI != writeI)
    {
      Serial.print(getValue());
    }
  }
#endif

boolean recognizePattern()
{        
  if(readI == writeI)
   return false;
  
  switch(waited)
  {
    case 0:
      if(getValue() == 0)
        waited++;
      else
        waited=0;
      break;
    case 1:
      if(getValue() == 0)
        waited++;
      else
        waited=0;
      break;
    case 2:
      if(getValue() == 0)
        waited++;
      else
        waited=0;
      break;
    case 3:
     if(getValue() == 0)
        waited++;
      else
        waited=0;
      break;
    case 4:
      if(getValue() == 1)
        waited++;
      else
        waited=0;
      break;
    case 5:
      if(getValue() == 0)
        waited++;
      else
        waited=0;
      break;
    case 6:
      if(getValue() == 1)
        waited++;
      else
        waited=0;
      break;
    case 7:
     if(getValue() == 0)
      {  
        waited=0;
        return true;
      }
      waited=0;
      break;
  }
  
  return false;
}
  

void isr() {
  currTime = micros();
  currState = digitalRead(PIN);  
  
  diff = currTime - lastTime;
  
  if((lastTime != 0) && (diff > 400) && (diff < 1500))
  {      
       if(currState == LOW)
       {
          //FALLING - calcolare il tempo in cui è stato alto l'impulso
          if((diff > 400) && (diff < 700))
          {
            //SHORT HIGH
            shortHighDetect = true;
          }
          else if ((diff > 1100) && (diff < 1500))
          {
            //LONG HIGH
            longHighDetect = true;            
          }    
       }
       else if((currState == HIGH))
       {
         //RISING
         if(longHighDetect)
         {         
           if((diff > 800) && (diff < 1200))
             putValue(0);
         }
         else if(shortHighDetect)
         {
           if((diff > 800) && (diff < 1200))
             putValue(1);             
         }
         
         longHighDetect = false;
         shortHighDetect = false;          
       }     
  }
  
  lastTime = currTime;
  //lastState = currState;
  
}

void putValue(byte b)
{
  buff[writeI] = b; 
  writeI++;
  if(writeI == BUFF_LEN)
   writeI = 0;
}

byte getValue()
{
  b = buff[readI];
  readI++;
  if(readI == BUFF_LEN)
    readI = 0;
   
  return b;
}
