#include "SimplePID.h"

SimplePID::SimplePID(float kp, float ki, float kd)
{
  _kp = kp;
  _ki = ki;
  _kd = kd;

  _setpoint   = 0.0f;
  _output     = 0.0f;
  _outMin     = 0.0f;
  _outMax     = 255.0f;   // เหมาะกับ analogWrite() ของ Arduino Uno

  _integral   = 0.0f;
  _lastInput  = 0.0f;

  _sampleTime = 100;      // 100 ms
  _lastTime   = 0;
}

void SimplePID::setSetpoint(float setpoint)
{
  _setpoint = setpoint;
}

void SimplePID::setTunings(float kp, float ki, float kd)
{
  // กัน gain ติดลบ (ป้องกันค่าผิดพลาด)
  if (kp < 0.0f) kp = 0.0f;
  if (ki < 0.0f) ki = 0.0f;
  if (kd < 0.0f) kd = 0.0f;

  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void SimplePID::setOutputLimits(float minOut, float maxOut)
{
  if (minOut >= maxOut) return; // ค่าไม่ถูกต้อง ไม่ทำอะไร

  _outMin = minOut;
  _outMax = maxOut;

  // จำกัด output และ integral ให้อยู่ในช่วงใหม่ทันที (anti-windup)
  if (_output > _outMax) _output = _outMax;
  else if (_output < _outMin) _output = _outMin;

  if (_integral > _outMax) _integral = _outMax;
  else if (_integral < _outMin) _integral = _outMin;
}

void SimplePID::setSampleTime(unsigned long sampleTimeMs)
{
  if (sampleTimeMs > 0)
  {
    _sampleTime = sampleTimeMs;
  }
}

bool SimplePID::compute(float input)
{
  unsigned long now = millis();
  unsigned long timeChange = now - _lastTime; // ใช้การลบ unsigned กัน millis() ล้น

  if (timeChange < _sampleTime)
  {
    return false; // ยังไม่ถึงเวลาคำนวณ
  }

  float error = _setpoint - input;

  // ส่วน Integral: สะสมแบบรวม Ki ไว้แล้ว เพื่อให้ anti-windup ทำงานกับ output ตรง ๆ
  _integral += _ki * error;
  // anti-windup: จำกัด integral ไม่ให้เกินช่วง output
  if (_integral > _outMax) _integral = _outMax;
  else if (_integral < _outMin) _integral = _outMin;

  // ส่วน Derivative: คิดจากการเปลี่ยนแปลงของ input (derivative on measurement)
  // ช่วยลดอาการกระชากตอนเปลี่ยน setpoint
  float dInput = input - _lastInput;

  // รวมผล PID
  float output = (_kp * error) + _integral - (_kd * dInput);

  // จำกัด output ให้อยู่ในช่วงที่กำหนด
  if (output > _outMax) output = _outMax;
  else if (output < _outMin) output = _outMin;

  _output    = output;
  _lastInput = input;
  _lastTime  = now;

  return true;
}

void SimplePID::reset()
{
  _integral  = 0.0f;
  _lastInput = 0.0f;
  _output    = 0.0f;
  _lastTime  = millis();
}
