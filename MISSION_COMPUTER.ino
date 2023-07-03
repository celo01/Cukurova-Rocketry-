#include "LoRa_E32.h"                         // LORA KÜTÜPHANESİ  
#include <Adafruit_BMP280.h>                  // BMP280 KÜTÜPHANESİ
#include <Wire.h>                             // I2C HABERLEŞME BAŞLATMAK İÇİN
#include <TinyGPS.h>                          // GPS KÜTÜPHANESİ NEO6M
#include "ccs811.h"  // CCS811 library


Adafruit_BMP280 bmp;                          // I2C ARAYÜZÜ BMP BAŞLATILMASI İÇİN
LoRa_E32 e32ttl(&Serial2);                    // LORA İÇİN HARDWARE SERIAL 2 KULLANIMI TANIMLAMASI  (TEENSY3.5)
CCS811 ccs811; 
TinyGPS gps;                                  // KÜTÜPHANEDEN GPS ICIN BASLAMA  
#define GPS Serial3                           // GPS İÇİN HARDWARE SERIAL 3 KULLANIMI ICIN TANIMLAMA (TEENSY 3.5)
char buf[32];                                 // GPS ICIN CHAR ATAMA

long zaman1 = 0;                              //ZAMAN BASLATMA

typedef struct {                              // TELEMETRI DE GONDERILECEK VERILERIN BELIRLENMESI VE PAKETIN OLUSTURULMAS (HANGI VERILERI YOLLAYACAKSIN??)       
  float SICAKLIK ;                            // BMP SICAKLIK BİLGİSİ
  float BASINC ;                              // BMP BASINC BILGISI
  float YUKSEKLIK;                            // BMP YUKSEKLIK BILGISI
  float ENLEM ;                               // GPS ENLEM VERISI
  float BOYLAM ;                              // GPS BOYLAM VERISI
  float CO2 ;
  float ETVOC ;

}
Signal;                                       // PAKET ISMI
Signal DATA;                                  // PAKET ICINDEKI VERININ HANGI KOMUTLA CEKILECEGI (DATA)


void setup() {                                 // KURULUMUN BASLAMASI
  Serial.begin(9600);                          // SERI HABERLESME BASLAT
  Wire.begin();                                // I2C BASLAT
  GPS.begin(9600);                             // GPS BASLAT 
  e32ttl.begin();                              // LORA BASLAT
  Serial.println(F("BMP280 test"));            // BMP TEST YAZDIRMA
  if (!bmp.begin(0X76)) {                      // BMP KONTROL
  Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  while (1);
  }                                            // BMP CALISMAZSA ERROR VERIR CALISIRSA LOOP DONMEYE BASLAR
}

void loop() {
  if (millis() - zaman1 > 1000) {             //EĞER ŞU ANKİ ZAMAN-zaman1  1000 DEN BÜYÜKSE

    bool newData = false;                     // GPS ICIN BASLATMA
    unsigned long chars;                      // GPS ICIN TANIMLAMA
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
   uint16_t eco2, etvoc, errstat, raw;
   ccs811.read(&eco2,&etvoc,&errstat,&raw);  
   
    unsigned long age;
    if (newData) {
      float flat, flon;
      gps.f_get_position(&flat, &flon, &age);
      Serial.print("LAT=");
      Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
      DATA.ENLEM = flat;
      Serial.print(" LON=");
      Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
      DATA.BOYLAM = flon;
    }

    DATA.SICAKLIK = (bmp.readTemperature());
    DATA.BASINC = (bmp.readPressure() / 100);
    DATA.YUKSEKLIK = (bmp.readAltitude(1011.25));
    // DATA.CO2 
    // DATA.ETVOC

    ResponseStatus rs = e32ttl.sendFixedMessage(0, 1, 23, &DATA, sizeof(Signal));
    Serial.println(rs.getResponseDescription());

    Serial.print(F("SICAKLIK = ")); Serial.print(bmp.readTemperature()); Serial.println(" *C");
    Serial.print(F("BASINÇ = ")); Serial.print(bmp.readPressure() / 100); Serial.println(" Pa");
    Serial.print(F("YUKSEKLİK = ")); Serial.print(bmp.readAltitude(1012.25)); Serial.println(" m");
    
    Serial.print("ENLEM:"); Serial.println(DATA.ENLEM);
    Serial.print("BOYLAM:"); Serial.println(DATA.BOYLAM);

    Serial.print("CO2=");  Serial.print(eco2);     Serial.print(" ppm  ");
    Serial.print("UOB="); Serial.print(etvoc);    Serial.print(" ppb  "); // UOB= UÇUCU ORGANİK BİLEŞEN


    
    Serial.println();

    zaman1 = millis();
  }
}
