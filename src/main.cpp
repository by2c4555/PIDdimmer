#include <LiquidCrystal_I2C.h>
#include <Adafruit_MAX31865.h>  // https://github.com/adafruit/Adafruit_MAX31865/tree/master
                                // https://learn.adafruit.com/adafruit-max31865-rtd-pt100-amplifier?view=all
#include <RBDdimmer.h>
#include <SimplePID_v1.h>




// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);
// The value of the RTD, in ohms, read from the MAX31865
uint16_t rtd;
// The fault code returned by the MAX31865
uint8_t fault;
// The ratio of the RTD resistance to the reference resistor
float ratio;




// Create a dimmerLamp object
#define DIMMER_PIN 3
#define ZC_PIN 2
dimmerLamp dimmer(DIMMER_PIN);



// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);
 
//Define Variables we'll be connecting to
float Input, Output, ReadTemp;;
// วิธีการหาค่า Manual Tuning 
// 1. ปิดค่า Ki และ Kd ให้เป็น 0
// 2. หาค่า Kp: ค่อยๆ ปรับ Kp เพิ่มขึ้นจนอุณหภูมิวิ่งขึ้นไปนิ่งต่ำกว่า 67 °C เล็กน้อย (เช่น นิ่งที่ 64 °C หรือ 65 °C) โดยไม่มีการแกว่งเลย
// 3. เพิ่มค่า  Kd: ค่อยๆ ใส่ค่า Kd เพื่อช่วยให้กราฟการเข้าสู่ Setpoint นุ่มนวลขึ้นและชันน้อยลงในช่วงปลาย
// 4. เพิ่มค่า Ki เป็นลำดับสุดท้าย: ค่อยๆ เพิ่มค่า Ki ทีละนิด เพื่อดึงอุณหภูมิที่นิ่งค้างอยู่ (เช่น 65 °C) ให้ไต่ขึ้นไปถึง 67 °C อย่างช้าๆ
float Setpoint = 67; // Setpoint temperature in Celsius
float consKp=6, consKi=0.005, consKd=120.0;
// สำหรับต้มน้ำ 30 ลิตร
// Kp (Proportional Gain) ค่าประมาณ 4.0 ถึง 6.0 
// Ki (Integral Gain)  ค่าประมาณ 0.001 ถึง 0.005
// Kd (Derivative Gain)  ค่าประมาณ  80.0 ถึง 120.0
//Specify the links and initial tuning parameters
SimplePID_v1 myPID(&Input, &Output, consKp, consKi, consKd);


void UpdateDimmerPeriodically(int Power) 
{
  // This function checks if 500 milliseconds have passed since the last dimmer reset. If so, it resets the dimmer state and updates the power based on the latest PID output.
  static unsigned long lastDimmerReset = 0; 
  const unsigned long dimmerResetInterval = 500; // 500 milliseconds
  
  unsigned long currentMillis = millis();

  // Check if 500 milliseconds have passed since the last dimmer reset
  if (currentMillis - lastDimmerReset >= dimmerResetInterval) 
  {
    lastDimmerReset = currentMillis; // Update the last reset time
    
    // Reset the dimmer state
    dimmer.setState(OFF);
    delay(10); // Short delay to ensure the dimmer state is reset
    dimmer.setState(ON);
    
    // Update the dimmer power based on the latest PID output
    dimmer.setPower(Power); 
  }
}


void setup()
{
  Serial.begin(9600);
  delay(1000);
  Serial.println("PID Control of PT100 with MAX31865 and Dimmer");
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary  
  delay(1000);
  Serial.println("MAX31865 3-Wire PT100 RTD Sensor --Initialized");  
  lcd.begin(20, 4);       //     lcd.begin(); for Arduino IDE        
  lcd.backlight();
  lcd.print("PID Control");
  delay(1000);
  Serial.println("LCD Display Initialized");

  dimmer.begin(NORMAL_MODE, ON);
  delay(1000);
  Serial.println("Dimmer Initialized");


//turn the PID on
  myPID.setSetpoint(Setpoint);
  myPID.setOutputLimits(0, 85); // Limit output to 0-85 for dimmer
  myPID.setSampleTime(1000); // Set sample time to 1 second
  myPID.setControllerDirection(PID_DIRECT);
  delay(1000);
  Serial.println("PID Initialized");

}


void loop()
{
// Check and print any faults
ReadTemp = thermo.temperature(RNOMINAL, RREF);
fault = thermo.readFault();
if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    rtd = thermo.readRTD();
    Serial.print("RTD value: "); Serial.println(rtd);
    ratio = rtd;
    ratio /= 32768;
    Serial.print("Ratio = "); Serial.println(ratio,8);
    Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
    Serial.print("Temperature = "); Serial.println(ReadTemp,2);
    Serial.print("PID Compute output : ");
    Serial.println(Output);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
  }else {
  // Read the temperature from the MAX31865
  Input = ReadTemp;
  // Calculate PID
    myPID.compute();
  // Display results on the LCD
    lcd.clear(); // clear the display
    lcd.setCursor(0, 0); // set the cursor to column 0, line 0
    lcd.print("Temp=");
    lcd.print(Input, 0);   // print double directly with 0 decimals
    lcd.print('C');
    lcd.print(" Set=");
    lcd.print(Setpoint, 0);   // print double directly with 0 decimals
    lcd.print('C');
    lcd.setCursor(0, 1); // set the cursor to column 0, line 1
    lcd.print("Output Power = ");
    lcd.print(Output, 0);   // print double directly with 0 decimals
    lcd.print('%');
    // Update the dimmer power based on PID output
    UpdateDimmerPeriodically((int)Output); // Cast Output to int for dimmer power
  }
}
