// Here's an encoder reader that maps and limits the output
// Encoder value is sent to serial, and PWM light outputs .
//  This is built around the N=1 test standalone test fixture board
//  and the Sparkfun RBG push-button encoder. (which needs a pull down resitor BTW)

//  Rev 3 adds the push button function to latch the LED ON or OFF
// and when in toggled off mode, ping the RED led to indicate power and off/standby state.

// Use the softwareserial library to create a new "soft" serial port
// for the display. This prevents display corruption when uploading code.
#include <SoftwareSerial.h>

// Attach the serial display's RX line to digital pin 2
SoftwareSerial LCDSerial(12,13); // pin 2 = TX, pin 3 = RX (unused)

#include <ClickEncoder.h>
#include <TimerOne.h>


#define ENC_FLAKY  
ClickEncoder *encoder;
int16_t last, value, b, last2, wiggle, lastB; 
boolean  toggleState = 0; 
boolean buttOn;

int Wiggle[]    = {1,    4,    6,  6,    6,    4,     3,   2,   1,   0};

int  Red_LEDpin = 11;
int  Green_LEDpin = 6;
int  Blue_LEDpin = 5;
int  White_LEDpin = 6; 
int  knobValue; 
int  scaler = 25; 
int  pushButton = 4; 

void timerIsr() {
  encoder->service();
}




//    setup section _________________________________________________________________________

void setup()
{
  LCDSerial.begin(9600); // set up serial port for 9600 baud
  delay(500); // wait for display to boot up
  
  encoder = new ClickEncoder(3, 2, 4);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr); 
  
  last = -1;
  
  Serial.begin(9600);
  Serial.println("Start");
  
  pinMode(pushButton, INPUT);

  
  
}

//    looping section _________________________________________________________________________

void loop()
{
  knobValue = getEncoder();
 // knobValue = 100; 
  b = digitalRead(pushButton); 
  
  toggleState = toggle(b,toggleState);    // functin to return a 1 or a 0 if the push botton has been pressed/toggle style
  
 // analogWrite(White_LEDpin, knobValue);
  
  if(toggleState == 255)
  {
    //We need to flip the knob values to shine the LEDs properly, 255=0ff ect ect.
    //The PWM for an LED goes here:_______________

  analogWrite(White_LEDpin, knobValue);
//  Timer1.pwm(White_LEDpin, knobValue, 1000); 
  knobValue=map( knobValue, 0,255,255,0);
 
  analogWrite(Blue_LEDpin, breath(knobValue)); 
 // analogWrite(Green_LEDpin, breath(knobValue));  
  analogWrite(Red_LEDpin, 255);
  }
  else
  {
        //The PWM for an LED goes here:_______________
  analogWrite(White_LEDpin, 0); 
  analogWrite(Blue_LEDpin, 255); 
 // analogWrite(Green_LEDpin, 255); 
  analogWrite(Red_LEDpin, breath(247)); 
  delay(100);
  }
    
  delay(125);
 
}


//    support functions section _________________________________________________________________________



int  getEncoder(void)
{
  
  
     value += (encoder->getValue()*scaler);


//limit the output to greater than zero
  if (value < 0){
    value = 0; 
  }
  
  // but less than 255
  if (value > 255){
    value = 255;
  }
  
  
  
  if (value != last) 
  {
    last = value;
    Serial.print("Encoder Value: ");
    Serial.println(value);
  }
  
  return value;
}


//  This just moves the input up and down by a few counts to simulate a breathing rythm)
int  breath(int wind)
{
  
  wind = wind + Wiggle[wiggle];
  
  if(wiggle < 9)
  {
    wiggle++;
  }
  else
  wiggle =0;
  
  //limit the output to greater than zero
  if (wind < 0){
    wind = 0; 
  }
  
  // but less than 255
  if (wind > 255){
    wind = 255;
  }
  
  
  return wind; 
}

//---------------------------------------------------------
//Toggle function that compares last reads to current reads and sets a toggle output bit
int toggle(int Bin, boolean toggleBit)
{

  if(Bin != lastB)
{
//  Serial.println("button pushed/released");
 // Serial.println(b);
  lastB=b;
  
  //and if, that push is to a 1, then the pushbutton toggle has been set
  if(Bin == 1)
  {
    toggleBit = ~toggleBit;
    Serial.print("toggle state = ");
    Serial.println(toggleBit);
  } 
}

return toggleBit;
}


