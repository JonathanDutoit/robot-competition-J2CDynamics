#include <Arduino.h>

int E1 = 8;     //M1 Speed Control
int E2 = 10;     //M2 Speed Control
int M1 = 9;     //M1 Direction Control
int M2 = 11;     //M2 Direction Control
int counter=0;

void stop(void)                    //Stop
{
  digitalWrite(E1,0);
  digitalWrite(M1,LOW);
  digitalWrite(E2,0);
  digitalWrite(M2,LOW);
}
void advance(char a,char b)          //Move forward
{
  analogWrite (E1,a);      //PWM Speed Control
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,HIGH);
}
void back_off (char a,char b)          //Move backward
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
}
void turn_L (char a,char b)             //Turn Left
{
  analogWrite (E1,a);
  digitalWrite(M1,LOW);
  analogWrite (E2,b);
  digitalWrite(M2,HIGH);
}
void turn_R (char a,char b)             //Turn Right
{
  analogWrite (E1,a);
  digitalWrite(M1,HIGH);
  analogWrite (E2,b);
  digitalWrite(M2,LOW);
}
void current_sense()                  // current sense and diagnosis
{
  int val1=digitalRead(2);
  int val2=digitalRead(3);
  if(val1==HIGH || val2==HIGH){
    counter++;
    if(counter==3){
      counter=0;
      Serial.println("Warning");
    }
  }
}

void setup(void)
{
  int i;
  for(i=4;i<=7;i++)
    pinMode(i, OUTPUT);
  Serial.begin(115200);      //Set Baud Rate
  Serial.println("Run keyboard control");
  digitalWrite(E1,LOW);
  digitalWrite(E2,LOW);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
}

void loop(void)
{
  /*
  static unsigned long timePoint = 0;    // current sense and diagnosis,if you want to use this
   if(millis() - timePoint > 1000){       //function,please show it & don't forget to connect the IS pins to Arduino
   current_sense();
   timePoint = millis();
   }
   */
  if(Serial.available()){
    char val = Serial.read();
    if(val != -1)
    {
      switch(val)
      {
      case 'w'://Move Forward
        advance (255,255);   //move forward in max speed
        break;
      case 's'://Move Backward
        back_off (255,255);   //move back in max speed
        break;
      case 'a'://Turn Left
        turn_L (100,100);
        break;
      case 'd'://Turn Right
        turn_R (100,100);
        break;
      case 'z':
        Serial.println("Hello");
        break;
      case 'x':
        stop();
        break;
      }
    }
    else stop();
  }

}
