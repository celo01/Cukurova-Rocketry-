/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor
  This example shows how to take Sensor Events instead of direct readings
  
  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
MPU6050 mpu6050(Wire);
float A;
float A_MAX;
float A_MIN;
float GYRO_X;
float GYRO_Y;
float GYRO_Z;
int MOSFET = 20;
int MOSFET2 = 25; // İŞLEMCİYE BAĞLANACAK DİJİTAL PİN 
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

void setup() {
  Serial.begin(9600);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 Sensor event test"));
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(MOSFET, OUTPUT);
  pinMode(MOSFET2, OUTPUT);

  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin(0x76);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();
}
);
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");
  
  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1010.00+36)); //The "1019.66" is the pressure(hPa) at sea level in day in your region
  Serial.println(" m");


 

  if ( A_MAX < A ){
     (A_MAX=A);
     }
    
      if(A_MAX > 30 ){
       
        if(A_MAX > A &&   GYRO_Y > 50  )
        {
          
        digitalWrite(MOSFET, HIGH);
        delay(100);
        digitalWrite(MOSFET, LOW);
        
          if(A < 600){
          digitalWrite(MOSFET2, HIGH);
          delay(100);
          digitalWrite(MOSFET2, LOW);
      }
        
    }
    
      }
   Serial.print("A_MAX = ");   Serial.println (A_MAX);
   Serial.print("A = ");       Serial.println (A);
   Serial.println();
   delay(1000);
   Serial.print("GYRO_X = ");   Serial.println (GYRO_X);
   Serial.print("GYRO_Y = ");   Serial.println (GYRO_Y);
   Serial.print("GYRO_Z = ");   Serial.println (GYRO_Z);
   
   // digitalWrite(MOSFET, HIGH);   // turn the LED on (HIGH is the voltage level)
   //delay(1000);               // wait for a second
   //digitalWrite(MOSFET, LOW);    // turn the LED off by making the voltage LOW
   //delay(1000); 


  
}
