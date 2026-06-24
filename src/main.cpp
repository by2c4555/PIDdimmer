#include <LiquidCrystal_I2C.h>
#include <Adafruit_MAX31865.h>  // https://github.com/adafruit/Adafruit_MAX31865/tree/master
#include <RBDdimmer.h>
#include <PID_v1.h>




// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// Create a dimmerLamp object
#define DIMMER_PIN 3
#define ZC_PIN 2
dimmerLamp dimmer(DIMMER_PIN);



// set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 20, 4);
 
//Define Variables we'll be connecting to
double Setpoint = 30;
double consKp=80, consKi=2.4, consKd=1.15;
double Input, Output;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);




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
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);  // Output will be between 0 and 100l
  myPID.SetTunings(consKp, consKi, consKd);
  delay(1000);
  Serial.println("PID Initialized");

}


void loop()
{
// Check and print any faults
uint8_t fault = thermo.readFault();
if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    Serial.print("Temperature Input : ");
    Serial.println(Input);
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
  // Read Thermo Temp (convert raw RTD reading to temperature in °C)
    Input = thermo.temperature(RNOMINAL, RREF);
  // Calculate PID
    myPID.Compute();
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
  delay(1000);  
}
