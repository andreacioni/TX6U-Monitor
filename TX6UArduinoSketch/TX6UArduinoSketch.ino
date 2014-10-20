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


#define INTERRUPT_PIN 39 // pin 2 -> interrupt 0 on Arduino UNO
#define LED_PIN 45
#define BUFF_LEN 200
#define MESSAGE_QUEUE_LEN 3

struct msg_map
{
  int id;
  float temp;
};

struct msg_map incoming[MESSAGE_QUEUE_LEN];

volatile unsigned long currTime=0,lastTime=0;
volatile int currState=0,readI=0,writeI=0;
volatile unsigned long diff;
volatile byte buff[BUFF_LEN],b;

volatile boolean shortHighDetect = false;
volatile boolean longHighDetect = false;

int waited=0;

boolean msgReady = false;

boolean recognizePattern();
void printRing();
void putValue(byte);
byte getValue();
int powerOfTwo(int); //pow() give round error need to be re-write!!!
boolean checkMessage(byte[]);
void buildMsg(byte[]);
struct msg_map get();
void put(struct msg_map);


#define DEBUG
//define DEBUG2

void setup() {
  
  #ifdef DEBUG
    Serial.begin(9600);
    pinMode(LED_PIN,OUTPUT);
  #endif
  
  attachInterrupt(INTERRUPT_PIN,isr,CHANGE);
  
  //digitalWrite(LED_PIN,HIGH);
}

void loop() {
  if(recognizePattern())
  {
    unsigned long start = micros() + 1000000; // 1 sec.
    
    #ifdef DEBUG
     Serial.println("pattern rec!");
     Serial.print("0000 1010 ");
    #endif
    
    int i=0,j=3; // 0A init pattern
    byte msg[36];
    
     while(i<36 && micros() < start)
     {
       if(readI != writeI)
       {
         msg[i] = getValue();
         
         #ifdef DEBUG
          Serial.print(msg[i]);
          if(((i+1)%4)==0)
            Serial.print(" ");
         #endif
         
         if(i==35)
           if(checkMessage(msg))
           {
             Serial.println("Correct message received!"); 
             buildMsg(msg);
             msgReady = true;
           } 
           
         i++;        
       }       
     }
          
  }
    
    //printRing();
}

boolean checkMessage(byte message[])
{
  byte val=0,chk=(byte)0x0A,sum=0;
  int parity_bit=0;
  
  #ifdef DEBUG
     Serial.println();
  #endif
  
  for(int j=0;j<8;j++)
  {    
    for(int i=(4*j);i<(4*(j+1));i++)
    {  if(message[i] == 1)
      {
        #ifdef DEBUG1
          Serial.print(powerOfTwo(i-(4*j)),DEC);
        #endif
        
        val += (byte) powerOfTwo(i-(4*j));
      }
    }
    
    sum += val;      
    
    #ifdef DEBUG
     Serial.print(" [ ");
     Serial.print(val,HEX);
     Serial.print(" ] ");
     
     Serial.print(" ( ");
     Serial.print(sum,HEX);
     Serial.println(" ) ");
    #endif
    
    val = 0;
  }
  
  sum &= (byte) 0x0F;
  
  #ifdef DEBUG
    Serial.print(" SUM = ");
    Serial.println(sum,HEX);
  #endif
  
  for(int i=32;i<36;i++)
    if(message[i] == 1)
        chk += (byte) powerOfTwo(32-i);  
   
   if(chk == sum)
     return true;
   else
     return false;
  
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
  currState = digitalRead(INTERRUPT_PIN);  
  
  digitalWrite(LED_PIN,LOW);
  
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

void buildMsg(byte msg[])
{
	struct msg_map newMsg;
	
	newMsg.id=0;
	newMsg.temp=0.0f;
	
  for(int i=4;i<11;i++)
  	if(msg[i]==1)
  		newMsg.id += powerOfTwo(i-4);
  		
  #ifdef DEBUG
    Serial.print(" ID = ");
    Serial.println(newMsg.id,DEC);
  #endif
  
  return newMsg;
}

void put(struct msg_map)
{
  if(queueEnd < MESSAGE_QUEUE_LEN)
  {
    incoming[queueEnd] = msg_map;
    queueEnd++;
  }
}

struct msg_map get()
{
  if( queueEnd > 0)
  {
   queueEnd--;
   return incoming[queueEnd];
  }
  
  return NULL;
}

int powerOfTwo(int expo)
{
  int value=0;
  for(int i=0;i<expo+1;i++)
  {
    if(i==0)
      value = 1;
    else
      value *= 2;
  }
  
  return value;
}