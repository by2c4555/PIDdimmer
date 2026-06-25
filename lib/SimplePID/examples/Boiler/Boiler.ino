/*
 * ตัวอย่างการใช้ SimplePID ควบคุมความร้อนหม้อต้มน้ำ (Arduino Uno)
 * --------------------------------------------------------------
 *  - Sensor input : อุณหภูมิ 0-100 องศา (ต่อที่ A0)
 *  - Output       : PWM 0-255 ขับ heater (ต่อที่ D3)
 */

#include <SimplePID.h>

const uint8_t SENSOR_PIN = A0;  // ขาอ่านอุณหภูมิ
const uint8_t HEATER_PIN = 3;   // ขา PWM ขับฮีตเตอร์ (D3 รองรับ analogWrite)

// สร้าง PID พร้อมค่า gain เริ่มต้น Kp, Ki, Kd
SimplePID pid(20.0f, 0.5f, 5.0f);

void setup()
{
  Serial.begin(9600);
  pinMode(HEATER_PIN, OUTPUT);

  pid.setSetpoint(75.0f);          // อุณหภูมิเป้าหมาย 75 องศา
  pid.setOutputLimits(0, 255);     // จำกัด output ตามที่ต้องการ (เช่น 0-200 ก็ได้)
  pid.setSampleTime(500);          // คำนวณทุก 500 ms

  analogWrite(HEATER_PIN, 0);
}

void loop()
{
  // แปลงค่า analog (0-1023) เป็นอุณหภูมิ 0-100 องศา
  float temperature = analogRead(SENSOR_PIN) * (100.0f / 1023.0f);

  // คำนวณ PID (จะอัปเดต output เมื่อครบ sample time)
  if (pid.compute(temperature))
  {
    analogWrite(HEATER_PIN, (int)pid.getOutput());

    // แสดงผล: setpoint, อุณหภูมิจริง, output
    Serial.print(pid.getSetpoint());
    Serial.print(',');
    Serial.print(temperature);
    Serial.print(',');
    Serial.println(pid.getOutput());
  }
}
