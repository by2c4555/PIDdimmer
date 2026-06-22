#include <LiquidCrystal_I2C.h>
#include <Adafruit_MAX31865.h>
#include <RBDdimmer.h>
#include <PID_v1.h>



#define FAN_CONTROL_PIN  7
#define WATER_CONTROL_PIN  8
#define OVERHEAT 31


// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
// Use software SPI: CS, DI, DO, CLK
// Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 thermo = Adafruit_MAX31865(5);


/// 
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
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary    
  lcd.begin(20, 4);       //     lcd.begin(); for Arduino IDE        
  lcd.backlight();
  lcd.print("PID Control");
  dimmer.begin(NORMAL_MODE, ON);

  pinMode(FAN_CONTROL_PIN, OUTPUT);
  pinMode(WATER_CONTROL_PIN, OUTPUT);


//turn the PID on
  myPID.SetMode(REVERSE);
  myPID.SetTunings(consKp, consKi, consKd);

}


void loop()
{
// Check and print any faults
uint8_t fault = thermo.readFault();
if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
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
  } else 
{

// Read Thermo Temp (convert raw RTD reading to temperature in °C)
  Input = thermo.temperature(RNOMINAL, RREF);
// Calculate PID
  myPID.Compute();

// Display results
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" Temp1=");
  lcd.print(Input, 2);   // print double directly with 2 decimals
  lcd.print((char)223);  // degree symbol
  lcd.print('C');
  
 

  Serial.print("Temperature Input : ");
  Serial.println(Input);
  Serial.print("PID Compute output : ");
  Serial.println(Output);


  double write = map(Output, 0, 255, 0, 100);
  double fanspeed = map(Output, 0, 255, 100, 0);
  //Serial.print("PID mapping output : ");
  //Serial.println(write);


  dimmer.setPower(write);
  if(Input > OVERHEAT  ) {
    analogWrite(FAN_CONTROL_PIN, fanspeed);
    }
}

   
  delay(1000);  
}