
/*
| Mensaje recibido | Valor del LED |
|------------------|---------------|
| 0                | Apagado       |
| 1                | Encendido     |
| Cualquier otro   | Parpadeando   |
*/

#include <Arduino.h>

#include <ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher pub("mensaje", &str_msg);

const int ledPin = 2;
int ledValue = 0;
bool isblinkMode = false;

// subscriptor

void ledValueCb(const std_msgs::Int16 &ledMsg)
{
  if (ledMsg.data == 0)
  {
    isblinkMode = false;
    digitalWrite(ledPin, LOW);
  }
  else if (ledMsg.data == 1)
  {
    isblinkMode = false;
    digitalWrite(ledPin, HIGH);
  }
  else
  {
    isblinkMode = true;
  }
}

ros::Subscriber<std_msgs::Int16> sub("ledValue", &ledValueCb);

void setup()
{
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  pinMode(ledPin, OUTPUT);
}

unsigned long startTime = millis();
unsigned long delayTime = 250; // ms

unsigned long startTime1 = millis();
unsigned long delayTime1 = 1000; // ms

int a = 0;

char msg[22];

void loop()
{
  if (millis() - startTime >= delayTime)
  {
    // ocurre esto cada segundo
    startTime = millis();
    a = a + 1;

    sprintf(msg, "El valor de a es: %d", a);
    str_msg.data = msg;
    pub.publish(&str_msg);

    if (isblinkMode)
    {
      if (ledValue)
        ledValue = 0;
      else
        ledValue = 1;

      digitalWrite(ledPin, ledValue);
    }
  }

  if (millis() - startTime1 >= delayTime1)
  {
    startTime1 = millis();
  }

  nh.spinOnce();
}
