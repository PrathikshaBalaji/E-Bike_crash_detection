//THIS EXAMPLE SHOWS HOW VVM501 ESP32 4G LTE MODULE CAN USE TO SEND AND RECEIVE SMS AND CALL AFTER CRASH IS DETECETED
//FOR VVM501 PRODUCT DETAILS VISIT www.vv-mobility.com

//servo
#include <ESP32Servo.h>
Servo servo;
static const int servoPin=13;

//gsm module
#define RXD2 27    //VVM501 MODULE RXD INTERNALLY CONNECTED
#define TXD2 26    //VVM501 MODULE TXD INTERNALLY CONNECTED
#define powerPin 4 //VVM501 MODULE ESP32 PIN D4 CONNECTED TO POWER PIN OF A7670C CHIPSET, INTERNALLY CONNECTED
int rx = -1;
#define SerialAT Serial1
String rxString;
int _timeout;
String _buffer;
String number = "+919876543210"; //REPLACE WITH YOUR NUMBER
void SendMessage();
void callNumber();

//ADXL
 #include <Wire.h>
 #include <Adafruit_Sensor.h>

#include<math.h>
const int xpin = A3;
const int ypin = A6;
// Assuming you intended to use zpin, define it
//const int zpin = 14;

// GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
/* Create object named bt of the class SoftwareSerial */
SoftwareSerial GPS_SoftSerial(5, 4);/* (Rx, Tx) */
/* Create an object named gps of the class TinyGPSPlus */
TinyGPSPlus gps;      

char coord_string[32];
// char time_string[32];
static void smartDelay(unsigned long ms);
//GPS variable definition
unsigned long start;
double lat_val, lng_val;
// uint8_t hr_val, min_val, sec_val;
bool loc_valid;// time_valid;
//timezone
// uint8_t hour, minute;

void setup() {
//GSM - SETUP
  servo.attach(servoPin);

  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, LOW);
  Serial.begin(115200);
  delay(100);
  SerialAT.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(10000);

  Serial.println("Modem Reset, please wait");
  SerialAT.println("AT+CRESET");
  delay(1000);
  SerialAT.println("AT+CRESET");
  delay(20000);  // WAITING FOR SOME TIME TO CONFIGURE MODEM

  SerialAT.flush();

  Serial.println("Echo Off");
  SerialAT.println("ATE0");   //120s
  delay(1000);
  SerialAT.println("ATE0");   //120s
  rxString = SerialAT.readString();
  Serial.print("Got: ");
  Serial.println(rxString);
  rx = rxString.indexOf("OK");
  if (rx != -1)
    Serial.println("Modem Ready");
  delay(1000);

  Serial.println("SIM card check");
  SerialAT.println("AT+CPIN?"); //9s
  rxString = SerialAT.readString();
  Serial.print("Got: ");
  Serial.println(rxString);
  rx = rxString.indexOf("+CPIN: READY");
  if (rx != -1)
    Serial.println("SIM Card Ready");
  delay(1000);
  
  // accel.setRange(ADXL335_RANGE_16_G);
  // ADXL 335- SETUP
  pinMode(xpin, INPUT);
  pinMode(ypin, INPUT);

  //  GPS - SETUP
  GPS_SoftSerial.begin(9600); /* Define baud rate for software serial communication */
}

void loop() {
  // SERVO CONTROL
  for(int pos=0;pos<=180;pos++)
  {
    servo.write(pos);
    delay(1);
  }
  for(int pos=180;pos>=0;pos--)
  {
    servo.write(pos);
    delay(1);
  }
  
         smartDelay(1000);  /* Generate precise delay of 1ms */
        lat_val = gps.location.lat();  /* Get latitude data */
        loc_valid = gps.location.isValid();/* Check if valid location data is available */
        lng_val = gps.location.lng();  /* Get longtitude data */
    
        if (!loc_valid)
        {          
         Serial.print("Latitude : ");
         Serial.println("*****");
         Serial.print("Longitude : ");
         Serial.println("*****");
        }
        else if(loc_valid)
        {
          
        DegMinSec(lat_val);
        Serial.print("Latitude in Decimal Degrees : ");
        Serial.println(lat_val, 6);
        DegMinSec(lng_val); /* Convert the decimal degree value into degrees minutes seconds form */
        Serial.print("Longitude in Decimal Degrees : ");
        Serial.println(lng_val, 6);
        sprintf(coord_string, "Crash Detected!!\nhttps:www.google.com/maps/search/?api=1&query=%0.6lf,%0.6lf\n",lat_val,lng_val);
        Serial.print(coord_string);
        }
        
 
  int xvalue = analogRead(xpin);
  int yvalue = analogRead(ypin);
  delay(500);
  Serial.println(xvalue);
  double xg = (xvalue - 1820.0) / 390.0;
  double yg = (yvalue - 1820.0) / 390.0;
  Serial.println(xg);
  Serial.println(yg);
  if ((xg > 1.0 || xg < -1.0 )/*&&(lati!=0.0 && longi!=0.0)*/) {
        
    Serial.println("Crash Detected");
    servo.detach();
    delay(1000);
    callNumber();
    delay(2000);
    SendMessage();
  }

}

void SendMessage()
{
  Serial.println ("Sending Message");
  SerialAT.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);
  Serial.println ("Set SMS Number");
  SerialAT.println("AT+CMGS=\"" + number + "\"\r"); //Mobile phone number to send message
  delay(1000);
  SerialAT.println(coord_string);
  delay(100);
  SerialAT.println((char)26);// ASCII code of CTRL+Z
  delay(1000);
  _buffer = _readSerial();
  
  smartDelay(1000);
  
void callNumber() {
  SerialAT.print (F("ATD"));
  SerialAT.print (number);
  SerialAT.print (F(";\r\n"));
  _buffer = _readSerial();
  Serial.println(_buffer);
}

String _readSerial() {
  _timeout = 0;
  while  (!SerialAT.available() && _timeout < 12000  )
  {
    delay(13);
    _timeout++;
  }
  if (SerialAT.available()) {
    return SerialAT.readString();
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS_SoftSerial.available())  /* Encode data read from GPS while data is available on serial port */
      gps.encode(GPS_SoftSerial.read());
/* Encode basically is used to parse the string received by the GPS and to store it in a buffer so that information can be extracted from it */
  } while (millis() - start < ms);
}
