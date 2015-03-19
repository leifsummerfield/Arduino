// Here's an encoder reader that maps and limits the output
// Encoder value is sent to serial, and PWM light outputs .
//  This is built around the N=1 test standalone test fixture board
//  and the Sparkfun RBG push-button encoder. (which needs a pull down resitor BTW)

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

boolean buttOn;

int Wiggle[]    = {1,    4,    6,  6,    6,    4,     3,   2,   1,   0};

int  Red_LEDpin = 7;
int  Green_LEDpin = 6;
int  Blue_LEDpin = 5;
int  knobValue; 
int  scaler = 15; 

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
  
  pinMode(4, INPUT);

  
  
}

//    looping section _________________________________________________________________________

void loop()
{
  knobValue = getEncoder();

  b = digitalRead(4); 

  knobValue=map( knobValue, 0,255,255,0);
  
  analogWrite(Blue_LEDpin, breath(knobValue)); 
  analogWrite(Green_LEDpin, breath(knobValue));  
  
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
