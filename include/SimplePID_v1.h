/*
 * SimplePID_v1 - ไลบรารี PID อย่างง่าย (header-only) สำหรับ Arduino Uno
 * --------------------------------------------------------------------------
 * เวอร์ชันนี้ "ผูก input/output เป็น pointer"
 *  - ส่ง address ของตัวแปร input และ output เข้ามาตอนสร้าง
 *  - คลาสจะอ่านค่า input และเขียนค่า output ผ่าน pointer ให้อัตโนมัติ
 *    ไม่ต้องคอยรับ-ส่งค่ากลับเอง
 *  - มีฟังก์ชัน setTunings() แยกต่างหาก อัปเดต Kp, Ki, Kd ได้ตลอดเวลา
 *    (เหมาะกับ Adaptive control / จูนค่าระหว่างรัน)
 *  - กำหนดทิศทางได้ด้วย setControllerDirection(PID_DIRECT / PID_REVERSE)
 *
 * Optimize สำหรับ Arduino Uno:
 *  - ใช้ float (32-bit) (Uno ไม่มี FPU, double = float อยู่แล้ว)
 *  - มี anti-windup กันค่า integral สะสมเกินขอบเขต output
 *  - จับเวลาด้วย millis() ภายใน ใช้การลบ unsigned กัน millis() ล้น
 *
 * วิธีใช้:
 *   float temperature, heaterPwm;
 *   SimplePID_v1 pid(&temperature, &heaterPwm, 20.0f, 0.5f, 5.0f);
 *   pid.setSetpoint(75.0f);
 *   pid.setOutputLimits(0, 255);
 *   pid.setSampleTime(500);
 *   ...
 *   // ใน loop():
 *   temperature = analogRead(A0) * (100.0f / 1023.0f);
 *   if (pid.compute()) analogWrite(pin, (int)heaterPwm);
 *
 *   // อัปเดต gain ได้ทุกเมื่อ:
 *   pid.setTunings(25.0f, 0.8f, 6.0f);
 */

#ifndef SimplePID_v1_h
#define SimplePID_v1_h

#include <Arduino.h>

// ทิศทางการควบคุม (Action)
//  PID_DIRECT  : input ต่ำกว่า setpoint → output เพิ่ม (งานให้ความร้อน เช่น ฮีตเตอร์)
//  PID_REVERSE : input มากกว่า setpoint → output เพิ่ม (งานทำความเย็น เช่น พัดลม/คอมเพรสเซอร์)
#define PID_DIRECT  0
#define PID_REVERSE 1

class SimplePID_v1
{
  private:
    float *_input;             // pointer ไปยังตัวแปร input
    float *_output;            // pointer ไปยังตัวแปร output

    float _kp, _ki, _kd;       // ค่า gain
    float _setpoint;           // ค่าเป้าหมาย
    float _outMin, _outMax;    // ช่วง output

    float _iSum;               // ผลรวม error ดิบ (ยังไม่คูณ Ki) เพื่อให้เปลี่ยน Ki ได้โดยไม่ต้อง reset
    float _lastInput;          // input รอบก่อน (ใช้ derivative on measurement)

    int _dir;                  // ทิศทาง: +1 = DIRECT, -1 = REVERSE

    unsigned long _sampleTime; // คาบเวลา (ms)
    unsigned long _lastTime;   // เวลาคำนวณครั้งก่อน

  public:
    // ผูก input/output ด้วย pointer พร้อมกำหนด gain เริ่มต้น
    SimplePID_v1(float *input, float *output, float kp, float ki, float kd)
    {
      _input  = input;
      _output = output;

      setTunings(kp, ki, kd);

      _setpoint   = 0.0f;
      _outMin     = 0.0f;
      _outMax     = 255.0f;   // เหมาะกับ analogWrite() ของ Arduino Uno

      _iSum       = 0.0f;
      _lastInput  = 0.0f;

      _dir        = +1;       // DIRECT เป็นค่าเริ่มต้น

      _sampleTime = 100;      // 100 ms
      _lastTime   = 0;
    }

    // ----- ฟังก์ชันกำหนด/อัปเดตค่า PID (เรียกได้ตลอดเวลา) -----
    void setTunings(float kp, float ki, float kd)
    {
      if (kp < 0.0f) kp = 0.0f;
      if (ki < 0.0f) ki = 0.0f;
      if (kd < 0.0f) kd = 0.0f;
      _kp = kp;
      _ki = ki;
      _kd = kd;
    }
    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }

    // ตั้งค่าเป้าหมาย (setpoint) เช่น อุณหภูมิที่ต้องการ
    void setSetpoint(float setpoint) { _setpoint = setpoint; }
    float getSetpoint() const { return _setpoint; }

    // เปลี่ยน pointer ที่ผูกไว้ภายหลังได้ (ถ้าต้องการ)
    void setInput(float *input)   { _input = input; }
    void setOutput(float *output) { _output = output; }

    // กำหนดทิศทางการควบคุม (PID_DIRECT หรือ PID_REVERSE)
    //  - DIRECT  : input < setpoint → output เพิ่ม (ให้ความร้อน)
    //  - REVERSE : input > setpoint → output เพิ่ม (ทำความเย็น)
    // เรียกได้ตลอดเวลา (จะ flip ค่า integral ที่สะสมไว้ให้อัตโนมัติ ไม่กระตุก)
    void setControllerDirection(int direction)
    {
      int newDir = (direction == PID_REVERSE) ? -1 : +1;
      if (newDir != _dir)
      {
        _iSum = -_iSum;   // สลับทิศทาง error ที่สะสมไว้ ให้ต่อเนื่อง
        _dir  = newDir;
      }
    }
    int getDirection() const { return (_dir < 0) ? PID_REVERSE : PID_DIRECT; }

    // กำหนดช่วง output ที่อนุญาต (min, max) ค่าเริ่มต้น 0-255
    void setOutputLimits(float minOut, float maxOut)
    {
      if (minOut >= maxOut) return; // ค่าไม่ถูกต้อง ไม่ทำอะไร

      _outMin = minOut;
      _outMax = maxOut;

      // จำกัด output ปัจจุบันและ integral term ให้อยู่ในช่วงใหม่ทันที (anti-windup)
      if (_output != nullptr)
      {
        if (*_output > _outMax) *_output = _outMax;
        else if (*_output < _outMin) *_output = _outMin;
      }
      clampIntegral();
    }

    // กำหนดคาบเวลาในการคำนวณ (มิลลิวินาที) ค่าเริ่มต้น 100 ms
    void setSampleTime(unsigned long sampleTimeMs)
    {
      if (sampleTimeMs > 0) _sampleTime = sampleTimeMs;
    }

    // คำนวณ PID เรียกใน loop() ทุกรอบได้เลย
    //  - อ่านค่าจาก *_input และเขียนผลลง *_output ให้อัตโนมัติ
    //  - จะคำนวณจริงเมื่อครบ sample time เท่านั้น
    //  - คืนค่า true เมื่อมีการอัปเดต output ใหม่
    bool compute()
    {
      if (_input == nullptr || _output == nullptr) return false;

      unsigned long now = millis();
      unsigned long timeChange = now - _lastTime; // ลบแบบ unsigned กัน millis() ล้น

      if (timeChange < _sampleTime) return false;  // ยังไม่ถึงเวลาคำนวณ

      float input = *_input;
      float error = _dir * (_setpoint - input);   // คูณทิศทาง (DIRECT/REVERSE)

      // ส่วน Integral: เก็บผลรวม error ดิบ (ยังไม่คูณ Ki)
      // ข้อดี: เปลี่ยน Ki ระหว่างรันได้เลย integral term ปรับตามทันที ไม่ต้อง reset
      _iSum += error;
      clampIntegral();           // anti-windup: กัน integral term เกินช่วง output

      // ส่วน Derivative: คิดจากการเปลี่ยนแปลงของ input (derivative on measurement)
      // คูณทิศทางด้วยเพื่อให้สอดคล้องกับ DIRECT/REVERSE
      float dInput = _dir * (input - _lastInput);

      // รวมผล PID
      float output = (_kp * error) + (_ki * _iSum) - (_kd * dInput);

      // จำกัด output ให้อยู่ในช่วงที่กำหนด
      if (output > _outMax) output = _outMax;
      else if (output < _outMin) output = _outMin;

      *_output   = output;   // เขียนผลลงตัวแปรที่ผูกไว้
      _lastInput = input;
      _lastTime  = now;

      return true;
    }

    // รีเซ็ตค่าภายใน (integral, derivative) เช่น ตอนเริ่มทำงานใหม่
    void reset()
    {
      _iSum      = 0.0f;
      _lastInput = (_input != nullptr) ? *_input : 0.0f;
      if (_output != nullptr) *_output = 0.0f;
      _lastTime  = millis();
    }

  private:
    // anti-windup แบบ back-calculation: กัน integral term (_ki * _iSum)
    // ไม่ให้เกินช่วง output แล้วย้อนคำนวณ _iSum กลับ
    void clampIntegral()
    {
      if (_ki <= 0.0f) return;            // ไม่มี I ไม่ต้องทำ (กันหารด้วยศูนย์)
      float iTerm = _ki * _iSum;
      if (iTerm > _outMax)      _iSum = _outMax / _ki;
      else if (iTerm < _outMin) _iSum = _outMin / _ki;
    }

};

#endif
