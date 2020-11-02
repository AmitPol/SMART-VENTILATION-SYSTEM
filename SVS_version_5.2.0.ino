#include <LiquidCrystal.h>
#include <Wire.h>
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

#define interrupt_pin 8 //Define Analog Pin (analog pins are 16-21)
volatile unsigned long t=0;
volatile unsigned long ot=0;

int Fan = 10; // Assign pin10 for main fans
int Fans = 11; // Assign pin11 for small fans
const int sensor1=A0; // Assigning analog pin A1 to variable 'sensor'
float tempc1;  //variable to store temperature in degree Celsius
float vout1;  //temporary variable to hold sensor reading
const int sensor2=A1; // Assigning analog pin A2 to variable 'sensor'
float tempc2;  //variable to store temperature in degree Celsius
float vout2;  //temporary variable to hold sensor reading
const int sensor3=A2; // Assigning analog pin A3 to variable 'sensor'
float tempc3;  //variable to store temperature in degree Celsius
float vout3;  //temporary variable to hold sensor reading
float indoor; //store avarage of 3 sensors.
float outdoor; //outside temperature.
int inPin = 12;   // choose the input pin (for a pushbutton)
int inPin2 = 9;   // choose the input pin (for a pushbutton)
int val = 0;     // variable for reading the pin status
int potPin = A3;  // Potentiometer1
int potValue = 0; // Reading Potentiometer value
int motorValue = 0;
int val2 = 0;     // variable for reading the pin status
int potPin2 = A4;  // Potentiometer2
int potValue2 = 0; // Reading Potentiometer value
int motorValue2 = 0;
int piezoPin = 13;  // beeper
int alreadybeeped = 0;
//boolean alreadyBeeped = false;   // flag for beeper



ISR(PCINT0_vect) 
{

  if (PINB & 1) 
  {
   ot=t;
   t=millis();
  }
   

}



void setup() 
{
 Serial.begin(115200);
  pinMode(sensor1,INPUT); // Configuring pin A1 as input
  pinMode(sensor2,INPUT);
  pinMode(sensor3,INPUT);
  pinMode(piezoPin, OUTPUT);  
  lcd.begin(16, 2);
  lcd.clear();
  delay(50);
  pinMode(Fans, OUTPUT); // Set pin for output to control TIP122 tranistor's Base pin
  analogWrite(Fans, 0); // By changing values from 0 to 255 we can control motor speed
  pinMode(Fan, OUTPUT); // Set pin for output to control TIP122 tranistor's Base pin
  analogWrite(Fan,0); // By changing values from 0 to 255 we can control motor speed
  delay(500);



  MCUCR = (1<<ISC01) | (1<<ISC00); 

 PCICR |= (1<<PCIE0); //For PIN CHANGE interrupt

 PCMSK0 |= (1<<PCINT0); //Enable interrupt PCINT0

 
 pinMode(interrupt_pin, INPUT); //Make pin an input 

 digitalWrite(interrupt_pin,HIGH); //Enable pullup resistor on Analog Pin 

 interrupts();
}

unsigned int rpm=0;

void loop() 
{

  {
 if((millis()-t)>300) 
 rpm=0;

 else if (t!=ot) 
 {

   rpm=60000.0/(float)(t-ot);
   }

   
   vout1=analogRead(sensor1);
   vout1=(vout1*500)/1023;
   tempc1=vout1; // Storing value in Degree Celsius
   vout2=analogRead(sensor2);
   vout2=(vout2*500)/1023;
   tempc2=vout2; // Storing value in Degree Celsius
   vout3=analogRead(sensor3);
   vout3=(vout3*500)/1023;
   outdoor=vout3; // Storing value in Degree Celsius
   indoor=(tempc1+tempc2)/2; //+tempc3)/3;
   lcd.setCursor(0,0);
   lcd.print("I=");
   lcd.print(indoor);
   delay(500);
   lcd.setCursor(8,0);
   lcd.print("To=");
   lcd.print(outdoor);
   delay(500);
   lcd.setCursor(0,1);
   lcd.print(rpm);
   
   val = digitalRead(inPin);
   val2 = digitalRead(inPin2);
   

 lcd.print(" rpms    ");
   
  
  }


// Fans speed according to temperature___________________________________________________________________
    
     if(indoor>=45)
    {
    for(int i = 0 ; i <= 255; i ++)
     { 
      analogWrite(Fans,i);
      analogWrite(Fan,i);

      if(alreadybeeped == 0)
     {
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            alreadybeeped = 1;
      }
    
    }
    }
        else if(indoor>=40)
    {
    for(int i = 0 ; i <= 210; i ++)
     { 
      analogWrite(Fans,i);
      analogWrite(Fan,i);

      if(alreadybeeped == 0)
     {
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            alreadybeeped = 1;
      }
    
    }
    }
        else if(indoor>=35)
    {
    for(int i = 0 ; i <= 163; i ++)
     { 
      analogWrite(Fans,i);
      analogWrite(Fan,i);

      if(alreadybeeped == 0)
     {
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            alreadybeeped = 1;
      }
      
     
    }
    }
        else if(indoor>=30)
    {
    for(int i = 0 ; i <= 116; i ++)
     { 
      analogWrite(Fans,i);
      analogWrite(Fan,i);

     if(alreadybeeped == 0)
     {
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            alreadybeeped = 1;
      }
       
    }
    }
        else if(indoor>=25)
    {
    for(int i = 0 ; i <= 68; i ++)
     { 
      analogWrite(Fans,i);
      analogWrite(Fan,i);
      
      if(alreadybeeped == 0)
     {
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, HIGH);   // set the beep on
            delay(1000);                  // wait for a second
            digitalWrite(piezoPin, LOW);    // set the beep off
            alreadybeeped = 1;
      }
       
      
    }
    }
        
    


    

    
   // IF outside temperature is high -----------------------------------------------------------------------
      if(outdoor>=indoor)
    {
      analogWrite(Fan,0);
    }
      if(outdoor<=indoor)
    {
      analogWrite(Fans,0);
    }
    if (val == HIGH) // SWITCH
  {
   //Serial.println("Mannual Fan");     
   potValue = analogRead(potPin);  
   motorValue = map(potValue, 0, 1023, 0, 255);
   analogWrite(Fan, motorValue);  
   delay(2);
  } 
    if (val2 == HIGH)
  {
 // Serial.println("Mannual Fans");  
  potValue2 = analogRead(potPin2);  
  motorValue2 = map(potValue2, 0, 1023, 0, 255);
  analogWrite(Fans, motorValue2);  
  delay(2);  
  } 


  
}
   
//Final_______________________________________________________
    
/*void serialEvent()
{
     if (Serial.available())
  {
   
    Serial.println("T1");
    Serial.println(tempc1);
    Serial.println("T2");
    Serial.println(tempc2);
    Serial.println("out");
    Serial.println(outdoor);
    Serial.println("Av");
    Serial.println(indoor);
  }
}*/
