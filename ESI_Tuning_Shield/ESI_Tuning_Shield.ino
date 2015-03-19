/*
  Analog Input
Retooled arduino to read analog inputs, and output on a software serial output a string needed to set the gains on
VIX250IH drives in the format 1GAINS(G1,G2,G3,G4,0)
Where G1-4 come from scaled analog inputs, that have potentiometers on their inputs
And a momentary contact switch to activate a test move to test the tuning.
 
 The circuit:
 * Potentiometer attached to analog input 0,1,2,3
 * center pin of the potentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +5V
 * Activate test switch on Digital5, with pull up resistor
 *   When LOW, it's pushed.
 
 * Note: because most Arduinos have a built-in LED attached 
 to pin 13 on the board, the LED is optional.
 
 
 Created by Leif Summerfield
Rev1 Feb 2, 2015
Rev2 March 18, 2015 - GitHub test...testing a small change


 
 */
 
 int Axis = 2
 ;       ; //What Axis are we talking too?
 int  JogSize_uM = 100    ; //in Microns...we're multiple downbelow to encoder counts

int Pgain_sensorPin = A0;    // select the input pin for the potentiometer
int Igain_sensorPin = A1;
int Dgain_sensorPin = A2;
int Fgain_sensorPin = A3;

int Test_ActivatePin = 5;

int ledPin = 13;      // select the pin for the LED
  // Pin 13: Arduino has an LED connected on pin 13
  // Pin 11: Teensy 2.0 has the LED on pin 11
  // Pin  6: Teensy++ 2.0 has the LED on pin 6
  // Pin 13: Teensy 3.0 has the LED on pin 13
int Pgain_value = 0;  // variable to store the value coming from the sensor
int Igain_value = 0;
int Dgain_value = 0;
int Fgain_value = 0;

float P_max = 200;
float I_max = 100;
float D_max = 100;
float F_max = 10;

int buttonTest_state = 0;

int inByte; 

#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);  
  pinMode(Test_ActivatePin, INPUT);

//Setup serial for debug output
  Serial.begin(9600);
  
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Hello, world?");  
  
}

void loop() {
  // read the value from the sensor:
  Pgain_value = analogRead(Pgain_sensorPin);    
  Igain_value = analogRead(Igain_sensorPin);
  Dgain_value = analogRead(Dgain_sensorPin);
  Fgain_value = analogRead(Fgain_sensorPin);
  
  Pgain_value =  map( Pgain_value,0,1023,0,P_max);
  Igain_value =  map( Igain_value,0,1023,0,I_max);
  Dgain_value =  map( Dgain_value,0,1023,0,D_max);
  Fgain_value =  map( Fgain_value,0,1023,0,F_max);
  Serial.print(" AXIS ");  Serial.println(String(Axis,DEC)) ;
  Serial.print(Pgain_value);
  Serial.println("  Proportional gain");
    mySerial.println(VixC(Axis,"ON"));
    mySerial.println("1OFF");
  
  
  Serial.print(Igain_value);
  Serial.println("  Integral");
  
  Serial.print(Dgain_value);
  Serial.println("  Derivative");
  
  Serial.print(Fgain_value);
  Serial.println("  Feedforward");
  
  Serial.println();
  
  mySerial.println();
  mySerial.print(VixC(Axis,"GAINS("));
  mySerial.print(Fgain_value);     mySerial.print(",");
  mySerial.print(Igain_value);     mySerial.print(",");
  mySerial.print(Pgain_value);     mySerial.print(",");
  mySerial.print(Dgain_value);     mySerial.println(",0)");
  
  buttonTest_state = digitalRead(Test_ActivatePin);
  if (buttonTest_state == LOW){
    mySerial.print(VixC(Axis,"ON"));
    digitalWrite(ledPin, HIGH);
    SendGoMove();
  }
  else {
    digitalWrite(ledPin, LOW);
    mySerial.print(VixC(Axis,"SV"));
    mySerial.print(VixC(Axis,"MA"));      //reset mode to absolute positioning
    delay(500);  
  }  
  
           
 
 //If we get a command in through the Arduino serial port...ping it through to the VIX
  // if we get a valid byte, read analog ins:
  if (Serial.available()) {
    // get incoming byte:
    int inByte = Serial.read();
  //  Serial.println(inByte);
    
  }
 
 
}



// 
////  Communication string to send when Test button flipped
//

void  SendGoMove(void)
{

    mySerial.println(VixC(Axis,"MI"));
    mySerial.println(VixC(Axis,"MI"));
    
    mySerial.println(VixC(Axis,"PROFILE1(10,10,50000,1)"));

    mySerial.println(VixC(Axis,"USE(1)"));
    mySerial.println(VixC(Axis,"G"));
    delay(700);
    mySerial.println(VixC(Axis,"H"));   
    mySerial.println(VixC(Axis,"G"));   
    delay(700);


    
  
  
}

String  VixC(int address, String command)
{
 
 return String(String(address, DEC) + command); 
 delay(50);
  
}
