#include "LoRa_E32.h"
#include <Adafruit_BMP280.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <TinyGPS.h>
//#include <SoftwareSerial.h>

Adafruit_BMP280 bmp; // I2C Interface
MPU6050 mpu6050(Wire);
LoRa_E32 e32ttl(&Serial3);
//static const int RXPin = 9, TXPin = 10;
//static const uint32_t GPSBaud = 9600;
TinyGPS gps;
//SoftwareSerial ss(RXPin, TXPin);
#define GPS Serial2


long zaman1 = 0;
 
typedef struct{
  float SICAKLIK ; //BMP SICAKLIK BİLGİSİ  
  float BASINC ; // BMP BASINC BILGISI
  float YUKSEKLIK; //BMP YUKSEKLIK BILGISI
  float IVME_X;
  float IVME_Y;
  float IVME_Z;
  float GYRO_X;
  float GYRO_Y;
  float GYRO_Z;
  float ENLEM= 0 ;
  float BOYLAM= 0 ;

Signal;
Signal DATA;


void setup() {
  Serial.begin(9600);
  GPS.begin(9600);
  e32ttl.begin();
  Wire.begin();
  delay(500);
  mpu6050.begin();
  Serial.begin(9600);
  
  Serial.println(F("BMP280 test"));
  mpu6050.calcGyroOffsets(true);
  if (!bmp.begin(0X76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1); 
    }
}

void loop() {
mpu6050.update();
 
if(millis() - zaman1 > 1000){ //EĞER ŞU ANKİ ZAMAN-zaman1  1000 DEN BÜYÜKSE 

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (GPS.available()) {
      char c = GPS.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
 unsigned long age;
 if (newData) {
  float flat,flon;
  gps.f_get_position(&flat, &flon, &age);
  Serial.print("LAT=");
  Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    DATA.ENLEM = flat;
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    DATA.BOYLAM = flon;      
 }
  
  DATA.SICAKLIK= (bmp.readTemperature());
  DATA.BASINC= (bmp.readPressure()/100);
  DATA.YUKSEKLIK= (bmp.readAltitude(1011.25));
  DATA.IVME_X= mpu6050.getAccX();
  DATA.IVME_Y= mpu6050.getAccY();
  DATA.IVME_Z= mpu6050.getAccZ();
  DATA.GYRO_X= mpu6050.getGyroX();
  DATA.GYRO_Y= mpu6050.getGyroY();
  DATA.GYRO_Z= mpu6050.getGyroZ();
  
  
  ResponseStatus rs = e32ttl.sendFixedMessage(0,1,23,&DATA,sizeof(Signal));
  Serial.println(rs.getResponseDescription());

    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure()/100);
    Serial.println(" Pa");
    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1011.25)); /* Adjusted to local forecast! */
    Serial.println(" m");
    Serial.print("accX : ");Serial.print(mpu6050.getAccX());                //X EKSENİNDE İVME
    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());              //Y EKSENİNDE İVME 
    Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());            //Z EKSENİNDE İVME 
    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
    Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
    Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());
    Serial.print("ENLEM:");Serial.println(DATA.ENLEM);
    Serial.print("BOYLAM:");Serial.println(DATA.BOYLAM);

  
  Serial.println();
    
    zaman1 = millis();
  }
}
}
