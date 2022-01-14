#include <RH_ASK.h>
#include <SPI.h>

// program constants
#define MESSAGE_LEN 4
#define LEFT_STICK_OFFSET 132
#define RIGHT_STICK_OFFSET 123
#define MOTOR_PERIOD 7000
#define CUTOFF_TIME 500
#define LEFT_SENSITIVITY 0.3
#define RIGHT_SENSITIVITY 0.4

// left and right channels are swapped, so just keep that in mind when adjusting sensitivities

// pin assignments
#define LEFTFORWARD 7
#define LEFTBACKWARD 8
#define RIGHTFORWARD 9
#define RIGHTBACKWARD 10

RH_ASK driver; // pin 11 is the radio rx pin and pin 12 is the radio tx pin (unused)

unsigned long lastMotorRise = 0;
unsigned long currMicros;
unsigned long lastRadioComm = 0;
float leftMotorHighTime = 0;
float rightMotorHighTime = 0;
boolean leftForwardPinState = false;
boolean leftBackwardPinState = false;
boolean rightForwardPinState = false;
boolean rightBackwardPinState = false;
int leftStickPos;
int rightStickPos;

void setup() {
  pinMode(LEFTFORWARD, OUTPUT);
  pinMode(LEFTBACKWARD, OUTPUT);
  pinMode(RIGHTFORWARD, OUTPUT);
  pinMode(RIGHTBACKWARD, OUTPUT);

  driver.init();
}

void loop() {
  uint8_t inBuffer[MESSAGE_LEN];
  uint8_t bufLen = sizeof(inBuffer);

  if (driver.recv(inBuffer, &bufLen))
  {
    if (inBuffer[0] == 45 && inBuffer[3] == 46)
    {
      leftStickPos = inBuffer[1] - LEFT_STICK_OFFSET;
      rightStickPos = inBuffer[2] - RIGHT_STICK_OFFSET;

      if (leftStickPos >= 0)
        leftMotorHighTime = leftStickPos/(float)(255 - LEFT_STICK_OFFSET) * MOTOR_PERIOD;
      else
        leftMotorHighTime = -leftStickPos/(float)(LEFT_STICK_OFFSET) * MOTOR_PERIOD;

      if (rightStickPos >= 0)
        rightMotorHighTime = rightStickPos/(float)(255 - RIGHT_STICK_OFFSET) * MOTOR_PERIOD;
      else
        rightMotorHighTime = -rightStickPos/(float)(RIGHT_STICK_OFFSET) * MOTOR_PERIOD;

      leftMotorHighTime *= LEFT_SENSITIVITY;
      rightMotorHighTime *= RIGHT_SENSITIVITY;
      
      lastRadioComm = millis();
    }
  }

  currMicros = micros();
  if (currMicros - lastMotorRise >= MOTOR_PERIOD)
  {
    lastMotorRise = currMicros;
    if (leftStickPos >= 0) 
    {
      digitalWrite(LEFTFORWARD, HIGH);
      digitalWrite(LEFTBACKWARD, LOW);
      leftForwardPinState = true;
      leftBackwardPinState = false;
    }
    else
    {
      digitalWrite(LEFTFORWARD, LOW);
      digitalWrite(LEFTBACKWARD, HIGH);
      leftForwardPinState = false;
      leftBackwardPinState = true;
    }

    if (rightStickPos >= 0) 
    {
      digitalWrite(RIGHTFORWARD, HIGH);
      digitalWrite(RIGHTBACKWARD, LOW);
      rightForwardPinState = true;
      rightBackwardPinState = false;
    }
    else
    {
      digitalWrite(RIGHTFORWARD, LOW);
      digitalWrite(RIGHTBACKWARD, HIGH);
      rightForwardPinState = false;
      rightBackwardPinState = true;
    }
  }
  if (currMicros - lastMotorRise >= leftMotorHighTime && (leftForwardPinState || leftBackwardPinState))
  {
    if (leftStickPos >= 0)
    {
      digitalWrite(LEFTFORWARD, LOW);
      leftForwardPinState = false;
    }
    else
    {
      digitalWrite(LEFTBACKWARD, LOW);
      leftBackwardPinState = false;
    }
  }
  if (currMicros - lastMotorRise >= rightMotorHighTime && (rightForwardPinState || rightBackwardPinState))
  {
    if (rightStickPos >= 0)
    {
      digitalWrite(RIGHTFORWARD, LOW);
      rightForwardPinState = false;
    }
    else
    {
      digitalWrite(RIGHTBACKWARD, LOW);
      rightBackwardPinState = false;
    }
  }

  if (millis() - lastRadioComm > CUTOFF_TIME)
  {
    leftMotorHighTime = 0;
    rightMotorHighTime = 0;
    digitalWrite(LEFTFORWARD, LOW);
    digitalWrite(LEFTBACKWARD, LOW);
    digitalWrite(RIGHTFORWARD, LOW);
    digitalWrite(RIGHTBACKWARD, LOW);
  }
}
