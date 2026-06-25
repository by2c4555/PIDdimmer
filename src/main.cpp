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
float Setpoint = 30;
float consKp=80, consKi=2.4, consKd=1.15;
float Input, Output, ReadTemp;;
//Specify the links and initial tuning parameters
SimplePID_v1 myPID(&Input, &Output, consKp, consKi, consKd);




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
    lcd.print("  Kp=");
    lcd.print(consKp, 0);   // print double directly with 0 decimals
    lcd.print("  Ki=");
    lcd.print(consKi, 1);   // print double directly with 1 decimal
    lcd.setCursor(0, 2);
    lcd.print("  Kd=");
    lcd.print(consKd, 1);   // print double directly with 1 decimal
    
    // Update the dimmer power based on PID output
    dimmer.setPower(Output);

  }
}
