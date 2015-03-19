// SparkFun Serial LCD example 1
// Clear the display and say "Hello World!"

// This sketch is for Arduino versions 1.0 and later
// If you're using an Arduino version older than 1.0, use
// the other example code available on the tutorial page.

// Use the softwareserial library to create a new "soft" serial port
// for the display. This prevents display corruption when uploading code.
#include <SoftwareSerial.h>

// Attach the serial display's RX line to digital pin 2
SoftwareSerial LCDSerial(12,13); // pin 2 = TX, pin 3 = RX (unused)

#include <ClickEncoder.h>
#include <TimerOne.h>


#define ENC_FLAKY  
ClickEncoder *encoder;
int16_t last, value, b; 

void timerIsr() {
  encoder->service();
}




//    setup section _________________________________________________________________________

void setup()
{
  LCDSerial.begin(9600); // set up serial port for 9600 baud
  delay(500); // wait for display to boot up
  
  encoder = new ClickEncoder(2, 3, 4);
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
   value += encoder->getValue();
   b = encoder->getButton();
   
  if (value != last) {
    last = value;
    Serial.print("Encoder Value: ");
    Serial.println(value);
  }
 
 
  b = digitalRead(4); 
      Serial.print("Button Value: ");
    Serial.println(b);
    
  LCDSerial.write(254); // move cursor to beginning of first line
  LCDSerial.write(128);

  LCDSerial.write("                "); // clear display
  LCDSerial.write("                ");

  LCDSerial.write(254); // move cursor to beginning of first line
  LCDSerial.write(128);
 
  LCDSerial.write("Hello, world!");
  LCDSerial.write(254);
  LCDSerial.write(192);
  
  LCDSerial.write(value);
  
  delay(100);
 
}

