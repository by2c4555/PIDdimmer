/*
 * SimplePID - ไลบรารี PID อย่างง่าย ปรับให้เหมาะกับ Arduino Uno
 * -------------------------------------------------------------
 * ออกแบบสำหรับงานควบคุมความร้อน (เช่น หม้อต้มน้ำ)
 *  - รับค่า input เป็นอุณหภูมิ 0-100 องศา
 *  - output กำหนด limit range ได้เอง (เช่น 0-255 สำหรับ analogWrite/PWM)
 *
 * Optimize สำหรับ Arduino Uno:
 *  - ใช้ float (32-bit) แทน double เพื่อความชัดเจน (Uno ไม่มี FPU,
 *    double = float อยู่แล้ว) และคำนวณ integer/float ให้น้อยที่สุด
 *  - ใช้ค่าแบบ value ไม่ใช้ pointer ลดการใช้ RAM และอ้อม
 *  - มี anti-windup กันค่า integral สะสมเกินขอบเขต output
 *  - จับเวลาด้วย millis() ภายใน ไม่ต้องคำนวณ dt เอง
 */

#ifndef SimplePID_h
#define SimplePID_h

#include <Arduino.h>

class SimplePID
{
  public:
    // กำหนดค่า gain เริ่มต้น Kp, Ki, Kd
    SimplePID(float kp, float ki, float kd);

    // ตั้งค่าเป้าหมาย (setpoint) เช่น อุณหภูมิที่ต้องการ
    void setSetpoint(float setpoint);
    float getSetpoint() const { return _setpoint; }

    // ปรับค่า gain ระหว่างทำงานได้
    void setTunings(float kp, float ki, float kd);
    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }

    // กำหนดช่วง output ที่อนุญาต (min, max) ค่าเริ่มต้น 0-255
    void setOutputLimits(float minOut, float maxOut);

    // กำหนดคาบเวลาในการคำนวณ (มิลลิวินาที) ค่าเริ่มต้น 100  ms
    void setSampleTime(unsigned long sampleTimeMs);

    // คำนวณ PID เรียกใน loop() ทุกรอบได้เลย
    //  - จะคำนวณจริงเมื่อครบ sample time เท่านั้น
    //  - คืนค่า true เมื่อมีการอัปเดต output ใหม่
    bool compute(float input);

    // อ่านค่า output ล่าสุด
    float getOutput() const { return _output; }

    // รีเซ็ตค่าภายใน (integral, derivative) เช่น ตอนเริ่มทำงานใหม่
    void reset();

  private:
    float _kp, _ki, _kd;       // ค่า gain
    float _setpoint;           // ค่าเป้าหมาย
    float _output;             // output ล่าสุด
    float _outMin, _outMax;    // ช่วง output

    float _integral;           // ค่าสะสมของ integral
    float _lastInput;          // input รอบก่อน (ใช้ derivative on measurement)

    unsigned long _sampleTime; // คาบเวลา (ms)
    unsigned long _lastTime;   // เวลาคำนวณครั้งก่อน
};

#endif
