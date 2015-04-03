#ifndef TX6U_h
#define TX6U_h

#include "Arduino.h"

#define BUFF_LEN 200 //Lenght of bit array
//#define DEBUG	//Debug use serial line

#define TOLLERANCE 0.1f
#define PARITY_BIT 11

#ifdef DEBUG
  #define LED_PIN 13
#endif

#define CHK_OK
#define CHK_ERR
#define NOTHING

struct msg_map
{
  int id;
  float temperature;
};

class TX6U
{
  public:
    TX6U(int pin,int interrupt);
    void setup();
    void setCelsius(boolean yesno);
    boolean available(); 
    struct msg_map get();
  private:    
    void buildMsg(byte[]);
    boolean recognizePattern();
    boolean checkMessage(byte[]);
#ifdef DEBUG
    void blink();
#endif
    boolean _celsius = false;
    int _waited=0;
  
};

#endif