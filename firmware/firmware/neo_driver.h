#include <Servo.h>

class NeoDriver
{
public:
  NeoDriver(int directionServo, int speedServo);
  void init();

  void driveNeo();
  void stopNeo();

  bool isDriving();

  void turnRight();
  void turnLeft();

  bool turnedRight();

private:
  int _directionPin;
  int _speedPin;

  Servo _DirectionServo;
  Servo _SpeedServo;

  const int _driveAngle = 0;
  const int _stopAngle = 60;

  const int _leftAngle = 110;
  const int _rightAngle = 165;

  bool _turnedRight;
  bool _driving;
};

NeoDriver::NeoDriver(int directionServo, int speedServo)
: _directionPin(directionServo), _speedPin(speedServo)
{
}

void NeoDriver::init()
{
  _DirectionServo.attach(_directionPin);
  _SpeedServo.attach(_speedPin);

  stopNeo();
  // In order to know the direction of the servo, we need to set it as the
  // servo could have been moved while the program wasn't running.
  turnRight();
}

void NeoDriver::driveNeo()
{
  if ( ! _driving )
  {
    // Build up to it.
    for ( int i = _stopAngle; i < _driveAngle; ++i )
    {
      _SpeedServo.write(i);
      delay(15);
    }
  }

  // Push a little extra.
  for ( int i = 1; i <= 10; ++i )
  {
    _SpeedServo.write(_driveAngle + i);
    delay(15);
  }

  _SpeedServo.write(_driveAngle);
  
  _driving = true;
}

void NeoDriver::stopNeo()
{
  _SpeedServo.write(_stopAngle);
  _driving = false;
}

bool NeoDriver::isDriving()
{
  return _driving;
}

// void NeoDriver::turnRight()
// {
//   _DirectionServo.write(_rightAngle);
//   delay(15);
//   // Overpress to make sure that we get it.
//   for (int i = 1; i <= 10; ++i)
//   {
//     _DirectionServo.write(_rightAngle + i);
//     delay(15);
//   }
//   _DirectionServo.write(_rightAngle);
//   _turnedRight = true;
// }

// void NeoDriver::turnLeft()
// {
//   _DirectionServo.write(_leftAngle);
//   delay(15);
//   // Overpress to make sure that we get it.
//   for (int i = 1; i <= 10; ++i)
//   {
//     _DirectionServo.write(_leftAngle - i);
//     delay(15);
//   }
//   _DirectionServo.write(_leftAngle);
//   _turnedRight = false;
// }

// bool NeoDriver::turnedRight()
// {
//   return _turnedRight;
// }
