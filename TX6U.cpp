#include "Arduino.h"
#include "TX6U.h"

int _pin,_interrupt;

volatile unsigned long currTime=0,_lastTime=0,blinkTime=0;
volatile int currState=0,readI=0,writeI=0;
volatile unsigned long diff;
volatile byte buff[BUFF_LEN],b;
volatile boolean shortHighDetect = false;
volatile boolean longHighDetect = false;

struct msg_map incoming;
//int waited=0;
boolean msgReady = false;

static void putValue(byte);
static byte getValue();
static void TX6UISR();

TX6U::TX6U(int pin,int interrupt) {
  _interrupt = interrupt;
  _pin = pin;  
}

void TX6U::setup() {
  
  #ifdef DEBUG
    Serial.begin(9600);
    pinMode(LED_PIN,OUTPUT);
    digitalWrite(LED_PIN,LOW);
  #endif
  
  attachInterrupt(_interrupt,TX6UISR,CHANGE);
}

void TX6U::setCelsius(boolean yesno) {
  _celsius = true;
}

boolean TX6U::available() {
  while(readI!=writeI) {
     if(recognizePattern())
     {
	unsigned long start = micros() + 1000000; // 1 sec.
	
	#ifdef DEBUG
	  Serial.println("pattern rec!");
	  Serial.print("0000 1010 ");
	#endif
	
	int i=0,j=3; // 0A init pattern
	byte msg[36];
	
	while((i<36) && (micros() < start))
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
		#ifdef DEBUG
		  Serial.println("Correct message received!"); 
		  digitalWrite(LED_PIN,HIGH);
		  blinkTime = millis()+300; //300ms of blink
		#endif
		
		buildMsg(msg);
		
		break;
	      } else {
		#ifdef DEBUG
		  Serial.println("ERROR, wrong checksum");
		#endif
	      }
	    i++;        
	  }       
	}
	      
     }
  }
  
  #ifdef DEBUG
    blink();
  #endif
  
  return msgReady;  
  //printRing();
}

boolean TX6U::recognizePattern() {
  if(readI == writeI)
   return false;
  
  switch(_waited)
  {
    case 0:
      if(getValue() == 0)
        _waited++;
      else
        _waited=0;
      break;
    case 1:
      if(getValue() == 0)
        _waited++;
      else
        _waited=0;
      break;
    case 2:
      if(getValue() == 0)
        _waited++;
      else
        _waited=0;
      break;
    case 3:
     if(getValue() == 0)
        _waited++;
      else
        _waited=0;
      break;
    case 4:
      if(getValue() == 1)
        _waited++;
      else
        _waited=0;
      break;
    case 5:
      if(getValue() == 0)
        _waited++;
      else
        _waited=0;
      break;
    case 6:
      if(getValue() == 1)
        _waited++;
      else
        _waited=0;
      break;
    case 7:
     if(getValue() == 0)
      {  
        _waited=0;
        return true;
      }
      _waited=0;
      break;
  }
  
  return false;
}

void TX6U::buildMsg(byte msg[])
{	
  incoming.id=0;
  incoming.temperature=0.0f;
	
  for(int i=4;i<11;i++)
    if(msg[i]==1)
      incoming.id += (1 << (6-(i%4)));
  		
  #ifdef DEBUG
    Serial.print(" ID = ");
    Serial.println(incoming.id,DEC);
  #endif
  
  incoming.temperature=0;
  
  for(int j=3;j<8;j++)
  { 
    int val=0;   
    
    for(int i=(4*j);i<(4*(j+1));i++)
    {  if(msg[i] == 1)
      {       
        val += (byte) (1 << (3-(i%4)));
      }
    }
    
    incoming.temperature += val * pow(10,4-j);
  }
  
  if(_celsius)
    incoming.temperature = ((incoming.temperature-32.0f)/1.8f);
  		
  #ifdef DEBUG
    Serial.print(" TEMP = ");
    Serial.println(incoming.temperature);
    
    if(msgReady) 
      Serial.println("message overwrite...lost 1 msg");
    #endif    
  
  msgReady = true;
}

struct msg_map TX6U::get()
{
  msgReady = false;
  return incoming;
}

boolean TX6U::checkMessage(byte message[])
{
  byte val=0,chk=0,sum=(byte)0x0A; //chk = A because is the value of start sequence
  //int parity_bit=0;
  
  #ifdef DEBUG
     Serial.println();
  #endif
  
  for(int j=0;j<8;j++)
  {    
    for(int i=(4*j);i<(4*(j+1));i++)
    {  if((message[i]==1) && (i!=PARITY_BIT))
      {
        #ifdef DEBUG2
          Serial.print(3-(i%4),DEC);
          Serial.print(" -- ");
          Serial.print(1 << (3-(i%4)),DEC);
          Serial.print(" / ");
        #endif
        
        val += (byte) (1 << (3-(i%4)));
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
  
  for(int i=32;i<36;i++)
    if(message[i] == 1)
        chk += (byte) (1 << (3-(i%4)));
      
  #ifdef DEBUG
    Serial.print("[CHK ");
    Serial.print(chk,HEX);
    Serial.print(",SUM ");
    Serial.print(sum,HEX);
    Serial.println("]");
  #endif  
   
   if(chk == sum)
     return true;
   else
     return false;
  
}

#ifdef DEBUG
  void TX6U::blink() {
    if(blinkTime!=0)
      if(millis() > (blinkTime)) {
	digitalWrite(LED_PIN,LOW);
	blinkTime = 0;
      }    
  }
#endif 

static void putValue(byte b)
{
  buff[writeI] = b; 
  writeI++;
  if(writeI == BUFF_LEN)
   writeI = 0;
}

static byte getValue()
{
  b = buff[readI];
  readI++;
  if(readI == BUFF_LEN)
    readI = 0;
   
  return b;
}

static void TX6UISR() {
  currTime = micros();
  currState = digitalRead(_pin);  
  
  //digitalWrite(LED_PIN,LOW);
  
  diff = currTime - _lastTime;
  
  if((_lastTime != 0) && (diff > 400) && (diff < 1400))
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
  
  _lastTime = currTime;
  //lastState = currState;
}