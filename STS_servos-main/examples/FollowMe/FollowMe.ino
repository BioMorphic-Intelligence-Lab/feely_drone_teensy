// Make servo 2 follow the position of servo 1.
// Servo 1 is turned off so it can be moved by hand: servo2 then follows the position.

#include "STSServoDriver.h"

STSServoDriver servos;

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // Configure Serial5 for half-duplex
  Serial5.begin(1000000);

  // Since the serial port is taken by the servo, we can't easily send debug messages, so
  // we use the on-board led instead.
  if (!servos.init(&Serial5, 1000000))
  {
    // Failed to get a ping reply, turn on the led.
    digitalWrite(13, HIGH);
  }
  // Disable torque on servo 1
  servos.writeRegister(1, STSRegisters::TORQUE_SWITCH, 0);
  // Set servo 2 to position mode.
  servos.setMode(2, STSMode::POSITION);
}

void loop()
{
  // Get position of servo 1
  int pos = servos.getCurrentPosition(1);
  // Set the target of servo 2 to this position
  servos.setTargetPosition(2, pos);
  delay(50);
}