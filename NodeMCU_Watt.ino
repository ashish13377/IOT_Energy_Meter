#include <ConsentiumThings.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include "ACS712.h"
#include "DHT.h"
#define NETWRK_LED 2
#define ADC_BITS 12
#include <WiFi.h>

#include <HTTPClient.h>
ConsentiumThings board;  // create ConsentiumThing object

const char *ssid = "home_automation"; // add WiFi SSID
const char *pass = "Ashish@22777"; // add WiFi password

// const char *ssid = "Realme 9 Pro";  // add WiFi SSID
// const char *pass = "uB8Pnc3q";      // add WiFi password

// const char *ssid = "AEIE IIOT Lab";  // add WiFi SSID
// const char *pass = "hitk@aeie18";    // add WiFi password


const long interval = 1;                               // take 5 seconds of delay
const char *key = "1f73a0579d8cf47487d81e0e0b729e15";  // Write api key
const char *boardkey = "646a6efacdcd3fbe";

ACS712 sensor(ACS712_05B, 34);
LiquidCrystal_I2C lcd(0x27, 16, 2);

float FinalRMSVoltage;
float FinalRMSCurrent;
float FinalPower;
int FinalFz;
float t;
float h;
float FinalBillAmount;

int VoltageAnalogInputPin = 35;  // Which pin to measure voltage Value (Pin A0 is reserved for button function)
float voltageSampleRead = 0;     /* to read the value of a sample in analog including voltageOffset1 */
float voltageLastSample = 0;     /* to count time for each sample. Technically 1 milli second 1 sample is taken */
float voltageSampleSum = 0;      /* accumulation of sample readings */
float voltageSampleCount = 0;    /* to count number of sample. */
float voltageMean;               /* to calculate the average value from all samples, in analog values*/
float RMSVoltageMean;            /* square roof of voltageMean without offset value, in analog value*/
float adjustRMSVoltageMean;


float voltageOffset1 = 870.00;  // to Offset deviation and accuracy. Offset any fake current when no current operates.
                                // Offset will automatically callibrate when SELECT Button on the LCD Display Shield is pressed.
                                // If you do not have LCD Display Shield, look into serial monitor to add or minus the value manually and key in here.
                                // 26 means add 26 to all analog value measured.
float voltageOffset2 = 65.40;   // too offset value due to calculation error from squared and square root


unsigned long last_time = 0;
unsigned long current_time = 0;
float Wh = 0;



int DHTPIN = 19;
const int relayPin = 4;

DHT dht(DHTPIN, DHT11);

void setup() {
  analogReadResolution(ADC_BITS);
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Set relay pin as an output
  pinMode(relayPin, OUTPUT);

  // Initialize the LCD
  lcd.init();

  // Turn on the backlight
  lcd.backlight();

  lcd.setCursor(3, 0);
  lcd.print("IoT Energy");
  lcd.setCursor(5, 1);
  lcd.print(" Meter");
  delay(3000);
  lcd.clear();

  Serial.println("Calibrating... Ensure that no current flows through the sensor at this moment");
  sensor.calibrate();
  Serial.println("Done!");

  dht.begin();
  board.begin();               // init. IoT boad
  board.initWiFi(ssid, pass);  // begin WiFi connection
}

void bill() {
  last_time = current_time;
  current_time = millis();
  Wh = Wh + FinalPower * ((current_time - last_time) / 3600000.0);
  FinalBillAmount = Wh * 3.50 / 1000;

  Serial.println(Wh);
  Serial.println(FinalBillAmount);
}


void power_on() {
  // do nothing

  // Make a GET request
  HTTPClient http;
  http.begin("https://consentiuminc.onrender.com/api/users/getuser/641537e1c93de279309433c7");
  int httpCode = http.GET();

  // Check the response code
  if (httpCode == HTTP_CODE_OK) {
    // Parse the response body
    String responseBody = http.getString();
    Serial.println(responseBody);

    // Turn the relay on or off based on the "state" value
    if (responseBody == "HIGH") {
      digitalWrite(relayPin, HIGH);
      Serial.println("Relay turned on");
    } else {
      digitalWrite(relayPin, LOW);
      Serial.println("Relay turned off");
    }
  } else {
    Serial.println("Error on HTTP request");
  }
        http.end();

}

void Send_Sensor() {
  h = dht.readHumidity();
  t = dht.readTemperature();

  Serial.print("Temperature - ");
  Serial.println(t);
  Serial.print("Humidity - ");
  Serial.println(h);
}

//AC Current Measurement
void current() {
  float I = sensor.getCurrentAC();
  if (I < 0.08) {
    I = 0;
  };
  FinalRMSCurrent = I;
  // print the current value
  Serial.print("Current: ");
  Serial.print(FinalRMSCurrent);
  Serial.println(" A");
  Serial.println("---------------------------------");
  lcd.setCursor(0, 1);
  lcd.print("I:");
  lcd.print(I, 2);
  lcd.print("A");
}

//AC Power Measurement
void power() {
  FinalPower = FinalRMSVoltage * FinalRMSCurrent;
  Serial.print("Power: ");
  Serial.print(FinalPower);
  Serial.println(" W");

  lcd.setCursor(8, 1);
  lcd.print(" P:");
  lcd.print(FinalPower, 1);
  lcd.print("W");
}

void update_things() {

  float sensor_val[] = { FinalRMSVoltage, FinalRMSCurrent, FinalPower, t, h, Wh, FinalBillAmount };        // sensor data array
  String info_buff[] = { "Voltage", "Current", "Power", "Temperature", "Humidity", "kilowatt", "Bill" };  // sensor info. array

  int sensor_num = sizeof(sensor_val) / sizeof(sensor_val[0]);  // number of sensors connected

  board.sendREST(key, boardkey, sensor_num, info_buff, sensor_val, LOW_PRE, interval);  // send over REST with delay with desired prescision
}


void voltage() {
  /* 1- AC Voltage Measurement */
  if (micros() >= voltageLastSample + 1000) /* every 0.2 milli second taking 1 reading */
  {
    voltageSampleRead = (analogRead(VoltageAnalogInputPin) - 2048) - voltageOffset1; /* read the sample value including offset value*/
    voltageSampleSum = voltageSampleSum + sq(voltageSampleRead);                     /* accumulate total analog values for each sample readings*/
    voltageSampleCount = voltageSampleCount + 1;                                     /* to move on to the next following count */
    voltageLastSample = micros();                                                    /* to reset the time again so that next cycle can start again*/
  }

  if (voltageSampleCount == 1000) /* after 1000 count or 200 milli seconds (0.2 second), do the calculation and display value*/
  {
    voltageMean = voltageSampleSum / voltageSampleCount; /* calculate average value of all sample readings taken*/
    RMSVoltageMean = (sqrt(voltageMean)) * 1.5;          // The value X 1.5 means the ratio towards the module amplification.
    // adjustRMSVoltageMean = RMSVoltageMean + voltageOffset2;                       /* square root of the average value including offset value */ /* square root of the average value*/
    FinalRMSVoltage = RMSVoltageMean - voltageOffset2; /* this is the final RMS voltage*/
    if (FinalRMSVoltage <= 6)                          /* to eliminate any possible ghost value*/
    { FinalRMSVoltage = 0; }
    // print the current value
    Serial.print("Voltages: ");
    Serial.print(FinalRMSVoltage);
    Serial.println("V");

    Serial.print("Frequency: ");
    Serial.print(FinalFz);
    Serial.println("Hz");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("V:");
    lcd.print(FinalRMSVoltage, 2);
    lcd.print("V ");

    FinalFz = 50;
    if (FinalRMSVoltage == 0.00) {
      FinalFz = 00;
    };

    lcd.setCursor(9, 0);
    lcd.print(" F:");
    lcd.print(FinalFz, 1);
    lcd.print("Hz");
    current();
    power();
    Send_Sensor();
    power_on();
    bill();
    update_things();
    voltageSampleSum = 0;   /* to reset accumulate sample values for the next cycle */
    voltageSampleCount = 0; /* to reset number of sample for the next cycle */
  }
}



void loop() {
  // put your main code here, to run repeatedly:
  voltage();
}
